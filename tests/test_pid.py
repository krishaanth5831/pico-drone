"""PID controller, focusing on the two failure modes that matter in flight."""

import pytest

from flight.pid import PID


def test_proportional_only_response():
    pid = PID(kp=2.0, ki=0.0, kd=0.0, output_limit=10.0)
    assert pid.update(setpoint=1.0, measurement=0.0, dt=0.01) == pytest.approx(2.0)


def test_default_output_limit_is_unity():
    # Mixer inputs are normalised to -1..1, so the default clamp must match.
    pid = PID(kp=2.0, ki=0.0, kd=0.0)
    assert pid.update(setpoint=1.0, measurement=0.0, dt=0.01) == pytest.approx(1.0)


def test_zero_error_gives_zero_output():
    pid = PID(kp=2.0, ki=0.5, kd=0.1)
    assert pid.update(0.0, 0.0, 0.01) == pytest.approx(0.0)


def test_integral_accumulates_over_time():
    pid = PID(kp=0.0, ki=1.0, kd=0.0)
    first = pid.update(1.0, 0.0, 0.1)
    second = pid.update(1.0, 0.0, 0.1)
    assert second > first


def test_integral_is_clamped_against_windup():
    # A long saturation must not let the integrator run away, or the aircraft
    # keeps correcting long after the error is gone.
    pid = PID(kp=0.0, ki=10.0, kd=0.0, integral_limit=0.5, output_limit=100.0)
    for _ in range(500):
        pid.update(1.0, 0.0, 0.01)
    assert pid._integral == pytest.approx(0.5)
    assert pid.update(1.0, 0.0, 0.01) == pytest.approx(5.0)


def test_output_is_clamped():
    pid = PID(kp=1000.0, ki=0.0, kd=0.0, output_limit=1.0)
    assert pid.update(1.0, 0.0, 0.01) == pytest.approx(1.0)
    assert pid.update(-1.0, 0.0, 0.01) == pytest.approx(-1.0)


def test_no_derivative_kick_on_setpoint_step():
    # Differentiating error would produce a huge spike when the setpoint jumps.
    # Differentiating the measurement must not.
    pid = PID(kp=0.0, ki=0.0, kd=1.0, output_limit=1e6)
    pid.update(0.0, 0.0, 0.01)
    kick = pid.update(100.0, 0.0, 0.01)  # setpoint steps, measurement steady
    assert kick == pytest.approx(0.0)


def test_derivative_responds_to_measurement_change():
    pid = PID(kp=0.0, ki=0.0, kd=1.0, output_limit=1e6)
    pid.update(0.0, 0.0, 0.01)
    # Measurement rising means D opposes it, so the term is negative.
    assert pid.update(0.0, 1.0, 0.01) < 0.0


def test_reset_clears_state():
    pid = PID(kp=1.0, ki=1.0, kd=1.0)
    for _ in range(10):
        pid.update(1.0, 0.0, 0.01)
    pid.reset()
    assert pid._integral == 0.0
    assert pid._last_measurement is None


def test_zero_dt_is_safe():
    # A duplicated timestamp must not divide by zero and take the loop down.
    pid = PID(kp=1.0, ki=1.0, kd=1.0)
    assert pid.update(1.0, 0.0, 0.0) == 0.0
