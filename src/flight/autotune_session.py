"""
The autotune as PX4 runs it: a session the ground station starts, watches,
and ends - around the identification in flight/autotune.py.

PX4 (mc_autotune_attitude_control + QGroundControl's Autotune button) works
like this, and so does this file:

  1. The pilot hovers and presses Autotune in the ground station.
  2. The vehicle waits until it is steady, then wiggles roll, pitch and yaw in
     turn on top of its normal control, identifying each axis (progress bar).
  3. Anything odd ABORTS it and the drone just keeps flying on its old gains:
     a tilt past 25 deg, the pilot touching the sticks, the ground station
     link dropping, landing.
  4. New gains come out. PX4's MC_AT_APPLY decides what happens to them;
     here the ground station shows the result and the pilot picks:
       accept "now"      fly the new gains straight away for VERIFY_S seconds
                         and compare the wobble with the old gains' hover;
                         worse -> put the old gains back automatically
                         (PX4's MC_AT_APPLY = 2)
       accept "landing"  apply them once the drone has landed (MC_AT_APPLY = 1)
       reject            keep the old gains
     Gains only become permanent (the board's /gains.json) once applied.

Each loop, in the board's order:

    exc = session.update(t, airborne, tilt, gyro, sticks, link_ok)
    ... rate PIDs on session.gains, outputs + exc + session.trim -> mix -> motors
    session.record(out)
    ... next loop, after reading the gyro:
    session.observe(gyro, dt)

and when session.version changes, give the PIDs session.gains (keep their
integrators). session.take_save() hands over gains to write to /gains.json,
once, when they have been applied.

Pure Python, MicroPython-safe: the board, the simulator and the tests all run
this same file.
"""

import math

from flight.autotune import AXES, REFERENCE, Autotune

# States, as the ground station shows them.
IDLE = "idle"
SETTLING = "settling"          # waiting for a steady hover before exciting
RESULT = "result"              # done: waiting for accept / reject
VERIFY = "verify"              # new gains flying, being checked for wobble
STAGED = "staged"              # accepted, applied after landing
APPLIED = "applied"
REJECTED = "rejected"
FAILED = "failed"
RUNNING = (SETTLING, VERIFY) + AXES

ABORT_TILT_DEG = 25.0     # same limit autotune_sim.py flies with
SETTLE_S = 1.0            # this long steady before exciting...
SETTLE_TIMEOUT_S = 8.0    # ...or give up
SETTLE_TILT_DEG = 10.0    # "steady": tilt and rates under these
SETTLE_RATE_DPS = 40.0
VERIFY_S = 3.0            # how long the new gains are watched after "accept now"
# The wobble check: high-passed roll/pitch gyro RMS. New gains fail if they
# wobble more than OSC_RATIO x the old gains' steady hover AND by more than
# OSC_FLOOR_DPS (so a dead-calm baseline doesn't make normal noise a fail).
OSC_HP_HZ = 3.0
OSC_RATIO = 2.0
OSC_FLOOR_DPS = 4.0
AMPLITUDE_LIMITS = (0.02, 0.12)   # PX4's MC_AT_SYSID_AMP, in mixer units
MAX_TEXTS = 20


class AutotuneSession:
    def __init__(self, base, gains=None, trim=(0.0, 0.0)):
        """
        base:  {"roll": (kp, ki, kd), "pitch": ..., "yaw": ...} - lift.py's
               defaults, the gains the autotune scales (see Autotune.results).
        gains: the gains flying now (default: base), e.g. from /gains.json.
        trim:  (roll, pitch) trim flying now.
        """
        self.base = base
        self.gains = dict(gains or base)
        self.trim = (trim[0], trim[1])
        self.version = 0          # bumps whenever gains / trim in use change
        self.state = IDLE
        self.reason = None        # why it failed / aborted
        self.result = None        # see _finish
        self.texts = []           # (severity, message) for the ground station
        self._at = None
        self._new = None          # (gains, trim) from the last result
        self._prev = None         # (gains, trim) to go back to if verify fails
        self._save = None
        self._t = None
        self._last_axis = None
        self._osc_lp = [0.0, 0.0]
        self._osc_sum = 0.0
        self._osc_n = 0
        self.baseline_dps = None
        self.verify_dps = None

    # --- ground station side ---------------------------------------------------------
    def say(self, msg, severity="info"):
        self.texts.append((severity, "Autotune: " + msg))
        if len(self.texts) > MAX_TEXTS:
            self.texts.pop(0)

    def pop_texts(self):
        out, self.texts = self.texts, []
        return out

    @property
    def running(self):
        return self.state in RUNNING

    def start(self, t, airborne, amplitude=0.06, include_yaw=True):
        """Returns (ok, why not)."""
        if self.running:
            return False, "already running"
        if self.state in (RESULT, STAGED):
            return False, "accept or reject the last result first"
        if not airborne:
            return False, "take off and hover first"
        lo, hi = AMPLITUDE_LIMITS
        amplitude = lo if amplitude < lo else (hi if amplitude > hi else amplitude)
        axes = AXES if include_yaw else ("roll", "pitch")
        self._at = Autotune(amplitude=amplitude, axes=axes)
        self.state = SETTLING
        self.reason = None
        self.result = None
        self._t_start = t
        self._steady_since = None
        self._last_axis = None
        self._osc_reset()
        self.say("started (amplitude %.2f, %s) - hold a steady hover"
                 % (amplitude, "roll, pitch, yaw" if include_yaw else "roll, pitch"))
        return True, None

    def abort(self, why):
        if not self.running:
            return
        if self.state == VERIFY:
            self._revert("verify aborted (%s) - old gains back" % why)
            return
        self.state = FAILED
        self.reason = why
        self.say("aborted - %s. Old gains still flying." % why, "warn")

    def accept(self, t, when):
        """when: "now" (fly + verify) or "landing" (apply after landing). Returns (ok, why not)."""
        if self.state != RESULT:
            return False, "no result to accept"
        if when == "now":
            self._prev = (dict(self.gains), self.trim)
            self._apply(*self._new)
            self.state = VERIFY
            self._t_verify = t
            self._osc_reset()
            self.say("new gains flying - checking for wobble for %.0f s" % VERIFY_S)
        elif when == "landing":
            self.state = STAGED
            self.say("new gains will be applied after landing")
        else:
            return False, "accept when? 'now' or 'landing'"
        return True, None

    def reject(self):
        if self.state not in (RESULT, STAGED):
            return False, "nothing to reject"
        self.state = REJECTED
        self._new = None
        self.say("rejected - keeping the old gains")
        return True, None

    def landed(self):
        """Call once when the drone lands / disarms: applies staged gains."""
        if self.running:
            self.abort("landed")
        if self.state == STAGED:
            self._apply(*self._new)
            self.state = APPLIED
            self._save = self._new
            self.say("landed - new gains applied")

    def take_save(self):
        """(gains, trim) to make permanent, once, after they were applied; else None."""
        out, self._save = self._save, None
        return out

    def progress(self):
        """0..1 for the ground station's progress bar."""
        if self.state == SETTLING:
            return 0.05 * min(1.0, self._settled_for() / SETTLE_S)
        if self.state in AXES:
            at = self._at
            if at.t0 is None:
                return 0.05
            return 0.05 + 0.9 * min(1.0, (self._t - at.t0) / at.total_s)
        if self.state in (RESULT, STAGED, VERIFY, APPLIED):
            return 1.0
        return 0.0

    # --- control loop side --------------------------------------------------------
    def update(self, t, airborne, tilt, gyro, sticks=False, link_ok=True):
        """
        tilt: (roll, pitch) deg from level, as the board estimates it.
        gyro: (x, y, z) deg/s. sticks: the pilot is giving input.
        Returns the (roll, pitch, yaw) excitation to ADD to the rate PID outputs.
        """
        dt = 0.0 if self._t is None else t - self._t
        self._t = t
        if not self.running:
            return (0.0, 0.0, 0.0)
        if not airborne:
            self.abort("landed")
        elif not link_ok:
            self.abort("ground station link lost")
        elif sticks:
            self.abort("stick input")
        elif abs(tilt[0]) > ABORT_TILT_DEG or abs(tilt[1]) > ABORT_TILT_DEG:
            self.abort("tilted past %.0f deg" % ABORT_TILT_DEG)
        if not self.running:
            return (0.0, 0.0, 0.0)
        self._osc_update(gyro, dt)

        if self.state == SETTLING:
            steady = (abs(tilt[0]) < SETTLE_TILT_DEG and abs(tilt[1]) < SETTLE_TILT_DEG
                      and abs(gyro[0]) < SETTLE_RATE_DPS and abs(gyro[1]) < SETTLE_RATE_DPS
                      and abs(gyro[2]) < SETTLE_RATE_DPS)
            if not steady:
                self._steady_since = None
                self._osc_reset()
            elif self._steady_since is None:
                self._steady_since = t
            if self._settled_for() >= SETTLE_S:
                # The old gains' wobble in a steady hover: the yardstick for verify.
                self.baseline_dps = self._osc_rms()
                self.state = self._at.axes[0]
            elif t - self._t_start > SETTLE_TIMEOUT_S:
                self.abort("never steady enough to start (needs tilt < %.0f deg, rates < %.0f deg/s)"
                           % (SETTLE_TILT_DEG, SETTLE_RATE_DPS))
            return (0.0, 0.0, 0.0)

        if self.state == VERIFY:
            if t - self._t_verify >= VERIFY_S:
                self.verify_dps = self._osc_rms()
                limit = max(OSC_RATIO * self.baseline_dps, self.baseline_dps + OSC_FLOOR_DPS)
                if self.verify_dps > limit:
                    self._revert("wobble %.1f deg/s vs %.1f with the old gains - old gains back"
                                 % (self.verify_dps, self.baseline_dps))
                else:
                    self.state = APPLIED
                    self._save = (dict(self.gains), self.trim)
                    self.say("verified (wobble %.1f deg/s, old gains %.1f) - new gains kept"
                             % (self.verify_dps, self.baseline_dps))
            return (0.0, 0.0, 0.0)

        # Exciting one axis.
        at = self._at
        exc = at.excitation(t)
        if at.axis != self._last_axis:
            if self._last_axis is not None:
                self._report_axis(self._last_axis)
            self._last_axis = at.axis
            if at.axis is not None:
                self.say("%s..." % at.axis)
        if at.done:
            self._finish()
            return (0.0, 0.0, 0.0)
        self.state = at.axis
        return exc

    def record(self, out):
        if self.state in AXES:
            self._at.record(out)

    def observe(self, gyro, dt):
        if self.state in AXES:
            self._at.observe(gyro, dt)

    # --- internals -------------------------------------------------------------------
    def _settled_for(self):
        return 0.0 if self._steady_since is None else self._t - self._steady_since

    def _osc_reset(self):
        self._osc_sum = 0.0
        self._osc_n = 0

    def _osc_update(self, gyro, dt):
        if dt <= 0.0:
            return
        a = 1.0 - math.exp(-2.0 * math.pi * OSC_HP_HZ * dt)
        for i in range(2):
            self._osc_lp[i] += a * (gyro[i] - self._osc_lp[i])
            hp = gyro[i] - self._osc_lp[i]
            self._osc_sum += hp * hp
        self._osc_n += 1

    def _osc_rms(self):
        return math.sqrt(self._osc_sum / (2 * self._osc_n)) if self._osc_n else 0.0

    def _report_axis(self, name):
        m = self._at.models[name].result()
        if m is None:
            self.say("%s: no usable fit - %s" % (name, self._at.models[name].why), "warn")
        else:
            self.say("%s: K %.0f (reference %.0f), lag %.0f ms, R^2 %.2f"
                     % (name, m["K"], REFERENCE[name][0], m["lag_s"] * 1000.0, m["r2"]))

    def _finish(self):
        res = self._at.results(self.base)
        gains = dict(self.gains)
        trim = [self.trim[0], self.trim[1]]
        axes, n_ok = {}, 0
        for i in range(len(AXES)):
            name = AXES[i]
            r = res[name]
            m = r["model"]
            entry = {"ok": r["gains"] is not None, "why": r["why"], "old": list(self.gains[name])}
            if r["gains"] is not None:
                n_ok += 1
                gains[name] = r["gains"]
                entry["new"] = list(r["gains"])
                entry["scale"] = r["scale"]
            else:
                entry["new"] = list(self.gains[name])
            if m is not None:
                entry.update({"K": m["K"], "K_ref": REFERENCE[name][0], "lag_ms": m["lag_s"] * 1000.0,
                              "r2": m["r2"], "trim": m["trim"],
                              "model": {"a": m["a"], "b": list(m["b"]), "dt": m["dt"]}})
                # The fit's trim is the whole command that cancels the twist
                # (the delivered command it saw already included today's trim).
                if i < 2:
                    trim[i] = m["trim"]
            axes[name] = entry
        if n_ok == 0:
            self.state = FAILED
            self.reason = "no axis identified"
            self.say("failed - no axis identified, old gains still flying", "error")
            self.result = {"axes": axes, "trim_old": list(self.trim), "trim_new": list(self.trim)}
            return
        self._new = (gains, (trim[0], trim[1]))
        self.result = {"axes": axes, "trim_old": list(self.trim), "trim_new": trim}
        self.state = RESULT
        self.say("finished (%d of %d axes) - review and accept or reject" % (n_ok, len(self._at.axes)))

    def _apply(self, gains, trim):
        self.gains = dict(gains)
        self.trim = (trim[0], trim[1])
        self.version += 1

    def _revert(self, msg):
        self._apply(*self._prev)
        self.state = FAILED
        self.reason = msg
        self.say(msg, "warn")
