"""
Autotune maths (src/flight/autotune.py) on synthetic plants, no simulator.

The closed-loop behaviour on simulated drones is checked by
sim/autotune_sim.py --check; these tests pin down the pieces.
"""

import math
import random

from flight.autotune import AXES, Autotune, AxisModel, scale_gains
from flight.mixer import mix, unmix


def fly_plant(model, a, b, c, n=1500, noise=0.0, seed=0):
    """
    Drive a known discrete plant  dw[k] = a dw[k-1] + b0 u[k-1] + b1 u[k-2] + c
    with a random square wave, feeding the model exactly as the board does:
    command() for what was sent this loop, then observe() the next gyro read.
    """
    rng = random.Random(seed)
    dt = 0.01
    w, dw = 0.0, 0.0
    u_hist = [0.0, 0.0, 0.0]
    u, hold = 0.05, 0
    for _ in range(n):
        if hold <= 0:
            u, hold = -u, rng.randint(5, 25)
        hold -= 1
        model.command(u)
        u_hist = [u] + u_hist[:-1]
        dw = a * dw + b[0] * u_hist[0] + b[1] * u_hist[1] + c
        w += dw
        model.observe(w + rng.gauss(0.0, noise), dt)
    return dt


def test_identifies_a_known_plant():
    tau, dt, K, trim = 0.040, 0.01, 30000.0, 0.02
    a = math.exp(-dt / tau)
    bsum = (1 - a) * K * dt
    b = (0.55 * bsum, 0.45 * bsum)       # response split over two loops, like the real drone
    c = -trim * bsum
    m = AxisModel()
    fly_plant(m, a, b, c)
    r = m.result()
    assert r is not None, m.why
    assert abs(r["K"] / K - 1) < 0.02
    assert abs(r["tau_s"] / tau - 1) < 0.05
    assert abs(r["trim"] - trim) < 0.002
    # Centroid of the taps: 0.45 of a loop.
    assert abs(r["dead_s"] - 0.45 * dt) < 0.002
    assert r["r2"] > 0.99


def test_identifies_through_gyro_noise():
    tau, dt, K = 0.040, 0.01, 30000.0
    a = math.exp(-dt / tau)
    bsum = (1 - a) * K * dt
    m = AxisModel()
    fly_plant(m, a, (0.5 * bsum, 0.5 * bsum), 0.0, noise=0.5)
    r = m.result()
    assert r is not None, m.why
    assert abs(r["K"] / K - 1) < 0.10


def test_pure_noise_is_rejected_with_a_reason():
    m = AxisModel()
    rng = random.Random(1)
    w = 0.0
    for _ in range(1000):
        m.command(rng.choice((-0.05, 0.05)))
        w += rng.gauss(0.0, 3.0)
        m.observe(w, 0.01)
    assert m.result() is None
    assert m.why


def test_too_few_samples_is_rejected():
    m = AxisModel()
    for k in range(20):
        m.command(0.05)
        m.observe(float(k), 0.01)
    assert m.result() is None
    assert "few samples" in m.why


def test_unmix_inverts_mix_when_nothing_clips():
    out = mix(0.6, 0.05, -0.03, 0.08, idle=0.4)
    thr, roll, pitch, yaw = unmix(out)
    assert abs(thr - 0.6) < 1e-9
    assert abs(roll - 0.05) < 1e-9
    assert abs(pitch + 0.03) < 1e-9
    assert abs(yaw - 0.08) < 1e-9


def test_unmix_reports_less_than_asked_when_a_motor_clips():
    # Hovering at 0.45 with idle 0.40: a 0.10 roll can only push 0.05 down.
    out = mix(0.45, 0.10, 0.0, 0.0, idle=0.4)
    _, roll, _, _ = unmix(out)
    assert 0.0 < roll < 0.10


def test_scale_gains_follows_authority():
    base = (0.004, 0.002, 0.00006)
    ref = (30000.0, 0.05)
    model = {"K": 20000.0, "lag_s": 0.05}           # 2/3 the authority -> 1.5x gain
    gains, s = scale_gains(base, model, ref)
    assert abs(s - 1.5) < 1e-9
    assert abs(gains[0] - 0.006) < 1e-12


def test_scale_gains_backs_off_for_a_slower_drone_but_never_speeds_up():
    base = (0.004, 0.002, 0.00006)
    ref = (30000.0, 0.05)
    _, slower = scale_gains(base, {"K": 30000.0, "lag_s": 0.10}, ref)
    _, faster = scale_gains(base, {"K": 30000.0, "lag_s": 0.025}, ref)
    assert abs(slower - 0.5) < 1e-9
    assert faster == 1.0


def test_scale_gains_is_clamped():
    base = (0.004, 0.002, 0.00006)
    ref = (30000.0, 0.05)
    _, s = scale_gains(base, {"K": 1000.0, "lag_s": 0.05}, ref)
    assert s == 2.0
    _, s = scale_gains(base, {"K": 1e6, "lag_s": 0.05}, ref)
    assert s == 0.5


def test_excitation_runs_each_axis_in_turn_then_stops():
    at = Autotune()
    seen = []
    t = 0.0
    while not at.done:
        exc = at.excitation(t)
        if at.exciting and any(exc):
            axis = AXES[[i for i in range(3) if exc[i] != 0.0][0]]
            if not seen or seen[-1] != axis:
                seen.append(axis)
        t += 0.01
    assert seen == list(AXES)
    assert abs(t - at.total_s) < 0.05
    assert at.excitation(t + 1.0) == (0.0, 0.0, 0.0)


def test_results_keep_gains_for_an_axis_that_did_not_fit():
    at = Autotune()
    base = {"roll": (0.004, 0.002, 0.00006), "pitch": (0.004, 0.002, 0.00006),
            "yaw": (0.005, 0.001, 0.0)}
    res = at.results(base)
    for name in AXES:
        assert res[name]["gains"] is None
        assert res[name]["why"]
