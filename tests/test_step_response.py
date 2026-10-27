"""The ground station's predicted step response (src/flight/step_response.py)."""

import math

from flight.autotune import scale_gains
from flight.step_response import metrics, step_response

LIFT_ROLL = (0.0040, 0.0020, 0.00006)


def model(k, tau=0.035, dt=0.01):
    """A drone with authority k (deg/s^2 per unit command): the autotune's model form."""
    a = math.exp(-dt / tau)
    g = (1 - a) * k * dt
    return {"a": a, "b": [0.5 * g, 0.5 * g, 0.0], "dt": dt}


def test_a_stronger_drone_responds_faster_on_the_same_gains():
    weak = metrics(*step_response(model(17000), LIFT_ROLL))
    strong = metrics(*step_response(model(45000), LIFT_ROLL))
    assert weak[0] is not None and strong[0] is not None
    assert strong[0] < weak[0]


def test_scaled_gains_bring_a_weak_drone_back_toward_the_reference():
    ref = metrics(*step_response(model(33793), LIFT_ROLL))[0]
    weak_old = metrics(*step_response(model(17000), LIFT_ROLL))[0]
    m = dict(model(17000), K=17000.0, lag_s=0.056)
    new, _ = scale_gains(LIFT_ROLL, m, (33793.0, 0.0562))
    weak_new = metrics(*step_response(model(17000), new))[0]
    assert abs(weak_new - ref) < abs(weak_old - ref)


def test_settles_on_the_step():
    t, r = step_response(model(33793), LIFT_ROLL, seconds=2.0)
    assert abs(r[-1] - 100.0) < 5.0
    assert t[1] == 0.01 and r[0] == 0.0
