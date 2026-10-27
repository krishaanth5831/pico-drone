"""
Predicted rate-loop step response from an autotune model - what the ground
station draws before you accept new gains (QGC shows the same kind of plot
after a PX4 autotune).

The autotune fits, per axis (see flight/autotune.py):

    dw[k] = a * dw[k-1] + b0 * u[k-1] + b1 * u[k-2] + b2 * u[k-3]

with w the gyro rate (deg/s) and u the rate PID's output. Closing the loop
with the board's own PID class (derivative on measurement, same clamps) and
asking for a step in rate shows how the old and the new gains would fly THIS
drone. It is a linear model: motors never saturate in it, and the CoM twist
(the trim) is left out, so read it for shape - speed and overshoot - not as a
promise.

Pure Python: the ground station imports it, and the tests check it.
"""

from flight.pid import PID

STEP_DPS = 100.0       # a firm stick flick on the rate loop
DURATION_S = 0.5


def step_response(model, gains, step=STEP_DPS, seconds=DURATION_S, integral_limit=50.0):
    """
    model: {"a", "b": [b0, b1, b2], "dt"} from an autotune result.
    gains: (kp, ki, kd).
    Returns (times, rates): the gyro rate after a `step` deg/s request at t=0.
    """
    a, b, dt = model["a"], model["b"], model["dt"]
    pid = PID(gains[0], gains[1], gains[2], integral_limit=integral_limit)
    u_hist = [0.0] * len(b)      # u[k-1], u[k-2], ... most recent first
    w, dw = 0.0, 0.0
    times, rates = [], []
    for k in range(int(seconds / dt) + 1):
        times.append(k * dt)
        rates.append(w)
        u = pid.update(step, w, dt)      # the board reads w, commands u...
        u_hist = [u] + u_hist[:-1]       # ...which the motors deliver next loop
        dw = a * dw + sum(b[i] * u_hist[i] for i in range(len(b)))
        w += dw
    return times, rates


def crossing(times, rates, level):
    """First time the response reaches `level`, interpolated between loops (None if never)."""
    for i in range(1, len(rates)):
        if rates[i] >= level > rates[i - 1]:
            f = (level - rates[i - 1]) / (rates[i] - rates[i - 1])
            return times[i - 1] + f * (times[i] - times[i - 1])
    return None


def metrics(times, rates, step=STEP_DPS):
    """(rise time 10-90 % in s or None, overshoot in % of the step)."""
    t10 = crossing(times, rates, 0.1 * step)
    t90 = crossing(times, rates, 0.9 * step)
    rise = t90 - t10 if t10 is not None and t90 is not None else None
    over = max(0.0, (max(rates) - step) / step * 100.0) if rates else 0.0
    return rise, over
