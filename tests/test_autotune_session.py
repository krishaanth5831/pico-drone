"""
The PX4-style session around the autotune (src/flight/autotune_session.py):
start, abort rules, accept now / after landing, reject, the wobble check.

A small synthetic drone stands in for the simulator: three independent axes,
each a motor lag feeding an integrator, flown by lift.py's cascaded PIDs
through the real mixer. sim/sitl.py runs the session on the MuJoCo drone.
"""

import math
import random

from flight.autotune_session import (
    APPLIED,
    FAILED,
    IDLE,
    REJECTED,
    RESULT,
    SETTLING,
    STAGED,
    VERIFY,
    AutotuneSession,
)
from flight.mixer import mix, unmix
from flight.pid import PID

BASE = {"roll": (0.0040, 0.0020, 0.00006), "pitch": (0.0040, 0.0020, 0.00006),
        "yaw": (0.0050, 0.0010, 0.0)}
DT = 0.01


class FakeDrone:
    """Three axes: K deg/s^2 per unit command, motor lag tau, one loop of delay."""

    def __init__(self, k=(24000.0, 22000.0, 3500.0), tau=0.04, twist=(0.01, -0.02), seed=0):
        self.k, self.tau, self.twist = k, tau, twist
        self.rng = random.Random(seed)
        self.w = [0.0, 0.0, 0.0]
        self.dw = [0.0, 0.0, 0.0]
        self.tilt = [0.0, 0.0]
        self.u_prev = [0.0, 0.0, 0.0]
        self.gains_version = -1
        self.t = 0.0

    def make_pids(self, session):
        self.rate = [PID(*session.gains[n], integral_limit=50.0) for n in ("roll", "pitch", "yaw")]
        self.gains_version = session.version

    def loop(self, session, sticks=False, link_ok=True, airborne=True, unstable=False):
        """One board loop. Returns the session's excitation."""
        s = session
        if s.version != self.gains_version:
            old = self.rate if self.gains_version >= 0 else None
            self.make_pids(s)
            if old:
                for i in range(3):
                    self.rate[i]._integral = old[i]._integral
        gyro = [w + self.rng.gauss(0.0, 0.3) for w in self.w]
        exc = s.update(self.t, airborne, self.tilt, gyro, sticks, link_ok)
        sp = [-4.0 * self.tilt[0], -4.0 * self.tilt[1], 0.0]
        cmd = [self.rate[i].update(sp[i], gyro[i], DT) + exc[i] for i in range(3)]
        out = mix(0.6, cmd[0] + s.trim[0], cmd[1] + s.trim[1], cmd[2], idle=0.4)
        s.record(out)
        delivered = unmix(out)[1:]
        a = math.exp(-DT / self.tau)
        for i in range(3):
            twist = self.twist[i] if i < 2 else 0.0
            # Delivered command acts one loop later, plus a constant twist
            # (the off-centre CoM) that the trim has to cancel.
            gain = self.k[i] * (3.0 if unstable else 1.0)
            self.dw[i] = a * self.dw[i] + (1 - a) * gain * DT * (self.u_prev[i] + twist)
            self.w[i] += self.dw[i]
            self.u_prev[i] = delivered[i]
        self.tilt[0] += self.w[0] * DT
        self.tilt[1] += self.w[1] * DT
        self.t += DT
        s.observe([w + self.rng.gauss(0.0, 0.3) for w in self.w], DT)
        return exc


def run_until(drone, session, states, limit_s=60.0, **kw):
    end = drone.t + limit_s
    while session.state not in states and drone.t < end:
        drone.loop(session, **kw)


def full_run(include_yaw=True):
    s = AutotuneSession(BASE)
    d = FakeDrone()
    d.make_pids(s)
    for _ in range(100):          # hover a second on the old gains first
        d.loop(s)
    ok, why = s.start(d.t, airborne=True, include_yaw=include_yaw)
    assert ok, why
    assert s.state == SETTLING
    run_until(d, s, (RESULT, FAILED))
    return s, d


def test_full_run_identifies_and_offers_a_result():
    s, d = full_run()
    assert s.state == RESULT, s.reason
    assert s.progress() == 1.0
    res = s.result
    for name in ("roll", "pitch", "yaw"):
        assert res["axes"][name]["ok"], res["axes"][name]["why"]
    # A weaker drone than the reference (K 24000 vs ~33800) gets more gain.
    assert res["axes"]["roll"]["scale"] > 1.2
    # The CoM twist is found: trim cancels +0.01 roll, -0.02 pitch.
    assert abs(res["trim_new"][0] + 0.01) < 0.004
    assert abs(res["trim_new"][1] - 0.02) < 0.004
    # Nothing applied yet: gains and trim in use are still the old ones.
    assert s.gains == BASE and s.trim == (0.0, 0.0) and s.version == 0
    msgs = " ".join(m for _, m in s.pop_texts())
    assert "roll: K" in msgs and "pitch: K" in msgs and "finished" in msgs


def test_progress_only_goes_up():
    s = AutotuneSession(BASE)
    d = FakeDrone()
    d.make_pids(s)
    s.start(d.t, airborne=True)
    last = 0.0
    while s.state not in (RESULT, FAILED):
        d.loop(s)
        p = s.progress()
        assert p >= last - 1e-9
        last = p
    assert last == 1.0


def test_yaw_can_be_skipped():
    s, _ = full_run(include_yaw=False)
    assert s.state == RESULT
    assert s.result["axes"]["yaw"]["why"] == "skipped"
    assert s.result["axes"]["yaw"]["new"] == list(BASE["yaw"])


def test_accept_now_applies_then_verifies():
    s, d = full_run()
    ok, _ = s.accept(d.t, "now")
    assert ok and s.state == VERIFY and s.version == 1
    assert s.gains["roll"] != BASE["roll"]
    run_until(d, s, (APPLIED, FAILED), limit_s=10.0)
    assert s.state == APPLIED, s.reason
    gains, trim = s.take_save()
    assert gains == s.gains and trim == s.trim
    assert s.take_save() is None          # handed over once


def test_verify_reverts_gains_that_wobble():
    s, d = full_run()
    s.accept(d.t, "now")
    # The new gains turn out far too hot on the real thing: 3x the authority
    # the fit saw, so the loop rings.
    run_until(d, s, (APPLIED, FAILED), limit_s=10.0, unstable=True)
    assert s.state == FAILED
    assert "old gains back" in s.reason
    assert s.gains == BASE and s.version == 2   # applied, then reverted
    assert s.take_save() is None


def test_accept_after_landing_waits_for_the_landing():
    s, d = full_run()
    s.accept(d.t, "landing")
    assert s.state == STAGED and s.gains == BASE
    s.landed()
    assert s.state == APPLIED and s.gains != BASE
    assert s.take_save() is not None


def test_reject_keeps_the_old_gains():
    s, d = full_run()
    assert s.reject()[0]
    assert s.state == REJECTED and s.gains == BASE
    s.landed()
    assert s.gains == BASE and s.take_save() is None


def test_stick_input_aborts():
    s = AutotuneSession(BASE)
    d = FakeDrone()
    d.make_pids(s)
    s.start(d.t, airborne=True)
    run_until(d, s, ("roll",))
    for _ in range(50):
        d.loop(s)
    exc = d.loop(s, sticks=True)
    assert s.state == FAILED and "stick" in s.reason
    assert exc == (0.0, 0.0, 0.0)
    assert d.loop(s) == (0.0, 0.0, 0.0)        # stays quiet afterwards


def test_link_loss_and_landing_abort():
    for kw, word in (({"link_ok": False}, "link"), ({"airborne": False}, "landed")):
        s = AutotuneSession(BASE)
        d = FakeDrone()
        d.make_pids(s)
        s.start(d.t, airborne=True)
        run_until(d, s, ("pitch",))
        d.loop(s, **kw)
        assert s.state == FAILED and word in s.reason


def test_tilt_aborts():
    s = AutotuneSession(BASE)
    d = FakeDrone()
    d.make_pids(s)
    s.start(d.t, airborne=True)
    run_until(d, s, ("roll",))
    d.tilt[0] = 30.0
    d.loop(s)
    assert s.state == FAILED and "tilted" in s.reason


def test_will_not_start_on_the_ground_or_twice():
    s = AutotuneSession(BASE)
    assert s.start(0.0, airborne=False) == (False, "take off and hover first")
    assert s.state == IDLE
    assert s.start(0.0, airborne=True)[0]
    assert not s.start(0.1, airborne=True)[0]


def test_never_steady_times_out():
    s = AutotuneSession(BASE)
    s.start(0.0, airborne=True)
    t = 0.0
    while s.state == SETTLING and t < 20.0:
        t += DT
        s.update(t, True, (0.0, 0.0), (60.0, 0.0, 0.0))   # spinning: never steady
    assert s.state == FAILED and "steady" in s.reason
