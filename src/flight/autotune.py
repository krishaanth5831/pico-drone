"""
PX4-style attitude autotune: excite one axis at a time while flying, identify
how the drone responds, and adjust the rate-loop gains to match.

This is the same idea as PX4's mc_autotune_attitude_control (the "Autotune"
button in QGroundControl), cut down to what a MicroPython loop can carry:

1. Fly level on the current gains (the PIDs keep it upright throughout).
2. For roll, then pitch, then yaw: add a square wave to that axis's command
   (switching at pseudo-random intervals, so it excites a band of
   frequencies), and fit a model of the response with recursive least squares
   (RLS) every loop.
3. Compare the model with the reference drone the default gains were made
   for, and scale the gains so this drone's rate loop behaves like that one's.
   Also out of the fit: the trim, i.e. how far the wires moved the balance.

The model, per axis, is a motor lag feeding an integrator:

    command u --[motor + prop lag, time constant tau, small delay]--> torque
             --[1 / inertia]--> angular acceleration --[integrate]--> rate

The board holds each command for one loop, and the motors start reacting a
few ms (the compute latency) into it. Sampled at the loop rate, on the gyro
rate w (deg/s) at loop k, that becomes exactly

    dw[k] = a * dw[k-1] + b0 * u[k-1] + b1 * u[k-2] + b2 * u[k-3] + c

with dw[k] = w[k] - w[k-1], and u[k-1] the command delivered in the loop that
ended with reading w[k]:

  a = exp(-dt/tau)          how much of the last acceleration carries over
  b0 + b1 + b2 = (1-a)*K*dt K = steady angular acceleration per unit command
                            (how they split up says how late the response is)
  c                         a constant twist: the CoM is off-centre, or a motor
                            is weaker. -c/(b0+b1+b2) is the command that
                            cancels it, i.e. the trim.

What the simulator taught this file (sim/autotune_sim.py):
- One input tap with a whole-loop delay is not enough: with a 35 ms lag at
  100 Hz, b0 and b1 come out about equal, and a single b read K 35-45 % off.
- u is the command the motors actually DELIVERED (mixer.unmix of the clipped
  outputs), not the one asked for. A drone hovering near the idle floor, or
  needing a big trim, clips a motor most loops; skipping those threw away
  over 90 % of the data on such drones.
- Textbook pole placement (pick a rise time, solve for KP/KI/KD) lost to the
  hand-set defaults at every rise time tried: the real loop has more lag than
  a three-pole model holds, and the 0.12-throttle headroom to idle means
  saturation, not linear bandwidth, decides whether it survives a bump. So the
  gains are anchored instead: the defaults are known good on the reference
  drone, and they are scaled by how much more or less authority this one has.

Pure Python, no numpy: the board and the simulator run this same file.
"""

import math

from flight.mixer import unmix

AXES = ("roll", "pitch", "yaw")
TAPS = 3              # input taps b0..b2: covers up to ~2 loops of latency
N_PARAMS = TAPS + 2   # a, b0..b2, c
PREFILTER_HZ = 15.0   # see AxisModel.observe

# What this autotune identifies on the simulator's nominal drone
# (sim/drone_params.json), the drone lift.py's default gains fly well:
# (K in deg/s^2 per unit command, lag = tau + dead time in s). Identified,
# not true values, so the fit's own small biases cancel out.
# Regenerate with: cd sim && uv run python autotune_sim.py --reference
REFERENCE = {
    "roll": (33793, 0.0562),    # true K 33659; spread over 5 runs 5 %
    "pitch": (31481, 0.0570),   # true K 30478; 5 %
    "yaw": (4584, 0.0509),      # true K 4992; 2 %
}
SCALE_LIMITS = (0.5, 2.0)   # never more than halve or double a gain in one go
# A fit must explain at least this share of the signal's variance. In the sim,
# good fits score 0.92-1.00; yaw fits that read K 45-70 % low (noise swamping
# yaw's small signal) scored 0.71-0.85 - and a K read low means gain pushed UP.
MIN_R2 = 0.90


class RLS:
    """Recursive least squares for y = th . phi with exponential forgetting."""

    def __init__(self, n, forget=0.998, p0=1e4):
        self.n = n
        self.forget = forget
        self.th = [0.0] * n
        self.P = [[p0 if i == j else 0.0 for j in range(n)] for i in range(n)]
        self.err2 = None   # running mean of squared prediction error
        self.count = 0

    def update(self, phi, y):
        n, P, th, lam = self.n, self.P, self.th, self.forget
        # Prediction error with the parameters from BEFORE this sample.
        e = y
        for i in range(n):
            e -= th[i] * phi[i]
        Pphi = [0.0] * n
        for i in range(n):
            row, acc = P[i], 0.0
            for j in range(n):
                acc += row[j] * phi[j]
            Pphi[i] = acc
        denom = lam
        for i in range(n):
            denom += phi[i] * Pphi[i]
        k = [Pphi[i] / denom for i in range(n)]
        for i in range(n):
            th[i] += k[i] * e
        # P is symmetric, so Pphi doubles as phi' P.
        for i in range(n):
            row, ki = P[i], k[i]
            for j in range(n):
                row[j] = (row[j] - ki * Pphi[j]) / lam
        self.err2 = e * e if self.err2 is None else 0.99 * self.err2 + 0.01 * e * e
        self.count += 1
        return e




class AxisModel:
    """Identification for one axis."""

    def __init__(self, prefilter_hz=PREFILTER_HZ):
        self.fit = RLS(N_PARAMS)
        self.prefilter_hz = prefilter_hz
        self.u_last = 0.0                   # command delivered this loop
        self.uf_hist = [0.0] * TAPS         # prefiltered u[k-1], u[k-2], u[k-3]
        self.w_prev = None
        self.dwf = None                     # prefiltered dw
        self.dt_sum = 0.0
        self.dt_n = 0
        self.why = None                     # why result() gave None
        self.y_mean = 0.0                   # running mean / mean square of the
        self.y_sq = 0.0                     # fitted signal, for result()'s R^2

    def observe(self, w, dt):
        """A new gyro reading (deg/s) for this axis, dt seconds after the last."""
        # Same first-order low-pass on the command and on dw. Applying one
        # filter to both sides leaves the model's a, b and c unchanged, but
        # strips the gyro noise that differencing amplifies - noise that
        # otherwise drags `a` toward 0 (the fit then sees no lag at all).
        alpha = 1.0 - math.exp(-2.0 * math.pi * self.prefilter_hz * dt)
        u0 = self.uf_hist[0]
        self.uf_hist = [u0 + alpha * (self.u_last - u0)] + self.uf_hist[:-1]
        if self.w_prev is not None:
            dw = w - self.w_prev
            if self.dwf is None:
                self.dwf = dw
            else:
                dwf_prev = self.dwf
                self.dwf = dwf_prev + alpha * (dw - dwf_prev)
                self.fit.update([dwf_prev] + self.uf_hist + [1.0], self.dwf)
                self.y_mean += 0.01 * (self.dwf - self.y_mean)
                self.y_sq += 0.01 * (self.dwf * self.dwf - self.y_sq)
                self.dt_sum += dt
                self.dt_n += 1
        self.w_prev = w

    def command(self, u):
        """The command the motors actually delivered this loop (see Autotune.record)."""
        self.u_last = u

    def result(self):
        """The fit as a dict, or None (and self.why says why) if nothing usable was identified."""
        if self.fit.count < 100:
            self.why = "too few samples (%d)" % self.fit.count
            return None
        th = self.fit.th
        a, b, c = th[0], th[1:1 + TAPS], th[-1]
        bsum = 0.0
        centroid = 0.0
        for i in range(TAPS):
            bsum += b[i]
            centroid += i * b[i]
        dt = self.dt_sum / self.dt_n
        if not (0.0 < a < 0.999):
            self.why = "no motor lag found (a = %.3f)" % a
            return None
        if bsum <= 0.0:
            self.why = "response has the wrong sign (check the mixer and IMU mounting)"
            return None
        tau = -dt / math.log(a)
        # Where the response "lands" among the taps, in loops: about 0.5 for
        # the hold alone, plus the latency. Becomes the dead time.
        dead_loops = min(float(TAPS - 1), max(0.0, centroid / bsum))
        # Share of dw's variance the model explains, over the last ~100 loops
        # (same averaging as the RLS error).
        var = self.y_sq - self.y_mean * self.y_mean
        r2 = 1.0 - self.fit.err2 / var if var > 0.0 else 0.0
        if r2 < MIN_R2:
            self.why = "fit too noisy (R^2 %.2f < %.2f)" % (r2, MIN_R2)
            return None
        self.why = None
        return {"r2": r2, "a": a, "b": list(b), "c": c, "dt": dt, "tau_s": tau,
                "dead_s": dead_loops * dt, "lag_s": tau + dead_loops * dt,
                "K": bsum / ((1.0 - a) * dt), "trim": -c / bsum,
                "samples": self.fit.count, "fit_err_dps": math.sqrt(self.fit.err2)}


def scale_gains(base, model, reference, limits=SCALE_LIMITS):
    """
    Rate-loop (kp, ki, kd) for this drone, from `base` - gains that fly the
    reference drone well - and the identified model.

    Loop gain is K * (kp, ki, kd), so multiplying the gains by K_ref / K keeps
    the rate loop's response the same as on the reference: a heavier drone
    or weaker motors (smaller K) gets proportionally more gain. If the
    response is also slower (longer lag), the gain comes down by
    lag_ref / lag as well - never up, a faster drone keeps the safe margin.
    Clamped to `limits`. Returns ((kp, ki, kd), scale).
    """
    k_ref, lag_ref = reference
    s = k_ref / model["K"]
    if model["lag_s"] > lag_ref:
        s *= lag_ref / model["lag_s"]
    s = max(limits[0], min(limits[1], s))
    return (base[0] * s, base[1] * s, base[2] * s), s


class Autotune:
    """
    Runs the excitation schedule and the identification. In the control loop:

        exc = at.excitation(t)              # (roll, pitch, yaw) to ADD to the PID output
        ...  out = mix(throttle, pid_roll + exc[0], ...)
        at.record(out)                      # the four motor outputs actually written
        ... next loop, after reading the gyro:
        at.observe(gyro, dt)

    `at.done` turns True after the last axis; then `at.results(default_gains)`.
    """

    def __init__(self, amplitude=0.06, axis_s=6.0, settle_s=1.5, yaw_gain=1.5, seed=1, axes=AXES):
        self.amplitude = amplitude
        # Which axes to excite, in order. PX4 can leave yaw out; so can this.
        self.axes = tuple(axes)
        self.axis_s = axis_s
        self.settle_s = settle_s
        # Yaw has ~6x less authority per unit, so excite harder - but not too
        # hard: the yaw PID pushes back on the rate it causes, and above ~1.5x
        # that drives motors to the idle floor (sim: K read 30 % low at 2.5x).
        self.yaw_gain = yaw_gain
        self.models = {name: AxisModel() for name in AXES}
        self._seed = seed
        self._sign = 1.0
        self._next_flip = 0.0
        self.t0 = None
        self.done = False
        self.axis = None

    def _rand(self):
        # Small LCG: deterministic, and needs no `random` module on the board.
        self._seed = (self._seed * 1103515245 + 12345) & 0x7FFFFFFF
        return self._seed / 0x7FFFFFFF

    def excitation(self, t):
        if self.t0 is None:
            self.t0 = t
        t = t - self.t0
        slot = self.settle_s + self.axis_s
        i = int(t // slot)
        if i >= len(self.axes):
            self.done, self.axis = True, None
            return (0.0, 0.0, 0.0)
        self.axis = self.axes[i]
        local = t - i * slot
        if local < self.settle_s:
            self._next_flip = t
            return (0.0, 0.0, 0.0)
        if t >= self._next_flip:
            self._sign = -self._sign
            # Hold each level 50-250 ms: a spread of frequencies around the
            # loop's bandwidth, which is what the fit needs to see.
            self._next_flip = t + 0.05 + 0.2 * self._rand()
        amp = self.amplitude * (self.yaw_gain if self.axis == "yaw" else 1.0) * self._sign
        out = [0.0, 0.0, 0.0]
        out[AXES.index(self.axis)] = amp
        return tuple(out)

    @property
    def exciting(self):
        return self.axis is not None and not self.done

    def observe(self, gyro, dt):
        if self.exciting:
            self.models[self.axis].observe(gyro[AXES.index(self.axis)], dt)

    def record(self, out):
        """`out`: the {motor: throttle} dict from mix() that was just written to the motors."""
        if self.exciting:
            delivered = unmix(out)   # (throttle, roll, pitch, yaw)
            self.models[self.axis].command(delivered[1 + AXES.index(self.axis)])

    @property
    def total_s(self):
        return len(self.axes) * (self.settle_s + self.axis_s)

    def results(self, base, reference=None):
        """
        base: {"roll": (kp, ki, kd), "pitch": ..., "yaw": ...} - the gains that
        fly the REFERENCE drone well, i.e. lift.py's defaults. Not the gains
        flown right now: after one autotune those are already scaled, and
        scaling them again would compound.

        Returns {axis: {"model", "gains", "scale", "why"}}. An axis that did
        not identify has gains None (keep what you have) and says why.
        """
        reference = reference or REFERENCE
        out = {}
        for name in AXES:
            if name not in self.axes:
                out[name] = {"model": None, "gains": None, "scale": None, "why": "skipped"}
                continue
            m = self.models[name].result()
            ref = reference[name]
            if m is None or ref[0] <= 0.0:
                why = self.models[name].why if m is None else "no reference for this axis yet"
                out[name] = {"model": m, "gains": None, "scale": None, "why": why}
                continue
            gains, s = scale_gains(base[name], m, ref)
            out[name] = {"model": m, "gains": gains, "scale": s, "why": None}
        return out
