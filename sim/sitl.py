"""
The simulated drone as a vehicle the ground station can talk to - this repo's
version of PX4's SITL (software in the loop).

    cd sim
    uv run python sitl.py                              # terminal 1: the drone (window)
    uv run python ../gcs/gcs.py                        # terminal 2: the ground station
    uv run python sitl.py --extreme "weak motors, heavy"   # a drone that needs retuning
    uv run python sitl.py --check                      # no windows: scripted end-to-end test

It is fly.py (same room, balloons, wind, keys) flying lift.py's PID, plus
the board's autotune session (src/flight/autotune_session.py) and ground-
station link (src/flight/ground_station.py, src/drivers/gcs_link.py) - the
same files firmware/autotune.py runs on the Pico. So the flow is the real one:

  1. Take off and hover: hold W (altitude hold and loiter are on).
  2. In the ground station: Autotune -> Start. Hands off the keys: touching
     the arrows or A/D aborts it, like moving the sticks on a PX4 drone.
  3. It wiggles roll, pitch, yaw (~25 s), then the ground station shows the
     result: before/after step responses, gains, trim.
  4. Accept "now" (flies the new gains, checks them for 3 s, reverts if they
     wobble), Accept "after landing", or Reject.
  5. Applied gains are saved to runs/sitl_gains.json - the sim's /gains.json
     - and loaded the next time sitl.py starts.

The vehicle listens on UDP 14555 and answers whoever talks to it.
"""

import json
import sys
from pathlib import Path

import numpy as np

import fly
from pico_drone_sim.controllers import LIFT_GAINS

# isort: split
# (pico_drone_sim above puts ../src on sys.path, so these import after it)
from drivers.gcs_link import VEHICLE_PORT, Link

from flight.autotune_session import AXES, RESULT, AutotuneSession
from flight.ground_station import GroundStation

HERE = Path(__file__).resolve().parent
GAINS_FILE = HERE / "runs" / "sitl_gains.json"
BASE = {name: LIFT_GAINS[name + "_rate"] for name in AXES}


def load_gains(path):
    """(gains, trim) from a gains file, or lift.py's defaults. Same format as the board's /gains.json."""
    try:
        raw = json.loads(Path(path).read_text())
        gains = {n: tuple(float(v) for v in raw["gains"][n]) for n in AXES}
        return gains, (float(raw["trim"][0]), float(raw["trim"][1])), "from %s" % path
    except (OSError, ValueError, KeyError, IndexError, TypeError):
        return dict(BASE), (0.0, 0.0), "lift.py defaults"


class SitlFlight(fly.Flight):
    def __init__(self, args):
        self.session = None
        super().__init__(args)
        self.gains_file = Path(args.gains_file)
        gains, trim, source = load_gains(self.gains_file)
        print("PID gains:", source, flush=True)
        self.session = AutotuneSession(BASE, gains, trim)
        self.link = Link(port=args.port)
        self.gs = GroundStation(self.link, self.session, vehicle="sim")
        self._version = None
        self._exc = (0.0, 0.0, 0.0)
        self._was_landed = True
        self.apply_session()

    # --- gains in use follow the session -----------------------------------------------
    def apply_session(self):
        s = self.session
        pid = self.controllers["pid"]
        for name in AXES:
            kp, ki, kd = s.gains[name]
            pid.g[name + "_rate"] = (kp, ki, kd)
            # Change the live PIDs in place: their integrators keep holding the trim.
            pid.rate[name].kp, pid.rate[name].ki, pid.rate[name].kd = kp, ki, kd
        self.pid_trim = s.trim
        if self.ctrl_name == "pid":
            self.env.trim = s.trim
        self._version = s.version

    def reset(self):
        if self.session is not None and self.session.running:
            self.session.abort("reset")
        super().reset()

    def cycle_controller(self):
        super().cycle_controller()
        if self.session.running and self.ctrl_name != "pid":
            self.session.abort("controller switched away from the PID")

    # --- one loop ---------------------------------------------------------------------
    def step(self, stick):
        env, s = self.env, self.session
        now = env.t
        z, _ = self.height()
        state = {
            "armed": not self.disarmed, "airborne": env.airborne,
            "att": [env.est[0] - env.level[0], env.est[1] - env.level[1], env.drone.attitude_deg()[2]],
            "rates": list(env.gyro), "thr": env.stick_throttle,
            "out": [env.out[m] for m in (1, 2, 3, 4)], "hz": 1.0 / env.period, "z": z,
            "bat": self.battery.volts if self.battery_on else None,
            "cant_tune": None if self.ctrl_name == "pid" else
            "the autotune tunes the PID - press C in the sim window until it says lift.py PID",
        }
        if "disarm" in self.gs.service(now, state):
            self.disarmed = "disarmed from the ground station"
            self.hold.z_sp, self.manual = 0.0, 0.0
        # The pilot touching tilt or yaw aborts, like PX4's stick check. Climb
        # keys don't: altitude hold just moves its target.
        sticks = bool(stick["roll"] or stick["pitch"] or stick["yaw"])
        tilt = (env.est[0] - env.level[0], env.est[1] - env.level[1])
        self._exc = s.update(now, env.airborne, tilt, env.gyro, sticks, self.gs.link_ok(now))
        super().step(stick)

        if s.version != self._version:
            self.apply_session()
        landed = not env.airborne and env.stick_throttle == 0.0
        if landed and not self._was_landed:
            s.landed()
            if s.version != self._version:
                self.apply_session()
        self._was_landed = landed
        save = s.take_save()
        if save is not None:
            gains, trim = save
            self.gains_file.parent.mkdir(exist_ok=True)
            self.gains_file.write_text(json.dumps({"gains": {n: list(gains[n]) for n in AXES},
                                                   "trim": list(trim)}, indent=2))
            s.say("saved to %s" % self.gains_file.name)

    def adjust_cmd(self, cmd):
        if self.ctrl_name != "pid":
            return cmd
        return cmd + np.array(self._exc)

    def after_step(self):
        self.session.record(self.env.out)
        self.session.observe(self.env.gyro, self.env.period)

    def extra_rows(self):
        s = self.session
        link = ("connected %s:%d" % self.link.peer if self.gs.link_ok(self.env.t)
                else "waiting on UDP %d - run gcs/gcs.py" % self.link.port)
        tune = "%s %3.0f%%" % (s.state.upper(), 100 * s.progress())
        if s.reason and s.state == "failed":
            tune += "  (%s)" % s.reason[:60]
        g = s.gains["roll"]
        return [("ground station", link), ("autotune", tune),
                ("roll rate gains", "%.4f %.4f %.5f  trim %+.3f %+.3f" % (g[0], g[1], g[2], *s.trim))]


# --- headless end-to-end check -------------------------------------------------------------
def run_check(args):
    """
    The whole flow through real UDP sockets, scripted: a fake ground station
    (the same Link class gcs.py uses) talks to the simulated drone in this
    process, on sim time. Fails loudly if any step doesn't happen.
    """
    out = HERE / "runs" / "sitl_check_gains.json"
    if out.exists():
        out.unlink()
    args.gains_file = str(out)
    args.port = 0
    v = SitlFlight(args)
    gcs = Link(port=0, peer=("127.0.0.1", v.link.port))
    inbox = []
    hb = {"on": True, "last": -1.0}
    failures = []

    def tick(climb=0.0):
        t = v.env.t
        if hb["on"] and t - hb["last"] >= 0.2:
            gcs.send({"t": "hb"})
            hb["last"] = t
        v.step({"roll": 0, "pitch": 0, "yaw": 0, "climb": climb})
        for m in gcs.poll(v.env.t):
            inbox.append(m)
            if m["t"] == "txt":
                print("   [%.1f s] %s" % (v.env.t, m["msg"]))
            elif m["t"] == "ack" and not m["ok"]:
                print("   [%.1f s] %s refused: %s" % (v.env.t, m["c"], m["why"]))

    def until(cond, limit_s, climb=0.0):
        end = v.env.t + limit_s
        while not cond() and v.env.t < end:
            tick(climb)
        return cond()

    def expect(ok, what):
        print("%s %s" % ("PASS" if ok else "FAIL", what))
        if not ok:
            failures.append(what)

    def cmd(c, **kw):
        gcs.send(dict({"t": "cmd", "c": c}, **kw))

    print("drone:", fly.P.describe(v.params), "| balloons" if v.env.balloons else "| lumped relief")
    until(lambda: False, 2.0, climb=1.0)     # alt hold up to ~1 m
    until(lambda: False, 4.0)
    expect(v.env.airborne and any(m["t"] == "tel" for m in inbox), "airborne, telemetry arriving")

    # 1. A full autotune, accepted now, verified, saved.
    cmd("at_start", amp=args.amp, yaw=True)
    expect(until(lambda: v.session.state in (RESULT, "failed"), 60.0) and v.session.state == RESULT,
           "autotune finished with a result (%s)" % (v.session.reason or v.session.state))
    if v.session.state != RESULT:
        return failures
    until(lambda: any(m["t"] == "res" for m in inbox), 1.0)
    res = [m for m in inbox if m["t"] == "res"][-1]["res"]
    print("   axis   K found / ref    lag   R^2   scale  new KP")
    for name in AXES:
        a = res["axes"][name]
        if a["ok"]:
            print("   %-5s %7.0f / %5.0f  %4.0fms  %.2f  x%.2f  %.4f"
                  % (name, a["K"], a["K_ref"], a["lag_ms"], a["r2"], a["scale"], a["new"][0]))
        else:
            print("   %-5s not tuned: %s" % (name, a["why"]))
    print("   trim %s -> %s" % (res["trim_old"], [round(x, 3) for x in res["trim_new"]]))
    expect(res["axes"]["roll"]["ok"] and res["axes"]["pitch"]["ok"], "roll and pitch identified (R^2 >= 0.90)")
    if not res["axes"]["yaw"]["ok"]:
        # Not a failure: a refused axis keeps its old gains, as PX4 does. On a
        # weak drone, a bigger --amp usually gets yaw a clean fit.
        print("   note: yaw kept its old gains (%s)" % res["axes"]["yaw"]["why"])
    cmd("accept", when="now")
    expect(until(lambda: v.session.state in ("applied", "failed"), 6.0) and v.session.state == "applied",
           "accepted now: new gains verified in flight (%s)" % (v.session.reason or v.session.state))
    until(lambda: False, 0.2)
    expect(out.exists() and json.loads(out.read_text())["gains"]["roll"][0] == v.session.gains["roll"][0],
           "applied gains written to %s and flying" % out.name)
    tuned = dict(v.session.gains)

    # 2. A second run is rejected: nothing changes. It also mustn't compound
    #    (it scales lift.py's defaults again, not the gains it is flying).
    cmd("at_start", amp=args.amp, yaw=False)
    until(lambda: v.session.state in (RESULT, "failed"), 60.0)
    if v.session.state == RESULT:
        r2 = v.session.result["axes"]["roll"]
        # Within 25 %: the fit itself scatters ~10 % run to run in gusts and balloons.
    expect(abs(r2["new"][0] / tuned["roll"][0] - 1.0) < 0.25,
               "a second autotune lands near the first (roll KP %.4f vs %.4f), no compounding"
               % (r2["new"][0], tuned["roll"][0]))
    cmd("reject")
    until(lambda: False, 0.3)
    expect(v.session.state == "rejected" and v.session.gains == tuned, "reject keeps the flying gains")

    # 3. Link loss mid-run aborts it, old gains keep flying.
    cmd("at_start", amp=args.amp, yaw=True)
    until(lambda: v.session.state == "pitch", 30.0)
    hb["on"] = False
    until(lambda: v.session.state == "failed", 3.0)
    expect(v.session.state == "failed" and "link" in (v.session.reason or ""), "link loss aborts the autotune")
    expect(v.session.gains == tuned and v.env.airborne, "...and it keeps flying on its gains")
    hb["on"] = True

    # 4. Stick input aborts.
    until(lambda: False, 0.5)
    cmd("at_start", amp=args.amp, yaw=True)
    until(lambda: v.session.state == "roll", 10.0)
    v.step({"roll": 1, "pitch": 0, "yaw": 0, "climb": 0})
    expect(v.session.state == "failed" and "stick" in (v.session.reason or ""), "stick input aborts")

    # 5. Disarm from the ground station.
    cmd("disarm")
    until(lambda: False, 0.2)
    expect(bool(v.disarmed), "disarm from the ground station stops the motors")
    out.unlink(missing_ok=True)
    return failures


def main():
    ap = fly.make_parser(__doc__, ctrl="pid")
    ap.add_argument("--port", type=int, default=VEHICLE_PORT, help="UDP port to listen on")
    ap.add_argument("--gains-file", default=str(GAINS_FILE), help="where applied gains are saved / loaded")
    ap.add_argument("--no-loiter", action="store_true", help="start without the sim-only position hold")
    ap.add_argument("--check", action="store_true", help="no window: scripted end-to-end test")
    ap.add_argument("--amp", type=float, default=0.06, help="--check: excitation amplitude (PX4's MC_AT_SYSID_AMP)")
    args = ap.parse_args()
    # Loiter on by default: 25 s of wiggling would otherwise drift it into a wall.
    args.loiter = not args.no_loiter
    if args.check:
        failures = run_check(args)
        print("\n%s" % ("ALL PASSED" if not failures else "%d FAILED" % len(failures)))
        sys.exit(1 if failures else 0)
    f = SitlFlight(args)
    print("SITL drone on UDP %d. Click the window, hold W to take off, then start the autotune "
          "from the ground station (gcs/gcs.py). ESC quits." % f.link.port, flush=True)
    fly.run_window(f, title="pico-drone SITL")


if __name__ == "__main__":
    main()
