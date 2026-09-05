"""wrap_deg_error - the wraparound behavior is the entire point of this function."""

import pytest

from flight.angles import wrap_deg_error


def test_no_wraparound_needed():
    assert wrap_deg_error(30, 10) == pytest.approx(20)
    assert wrap_deg_error(10, 30) == pytest.approx(-20)


def test_zero_error():
    assert wrap_deg_error(45, 45) == pytest.approx(0)


def test_crossing_zero_forward():
    # Pointed at 359, target 1: shortest path is +2, not -358.
    assert wrap_deg_error(1, 359) == pytest.approx(2)


def test_crossing_zero_backward():
    assert wrap_deg_error(359, 1) == pytest.approx(-2)


def test_exactly_opposite_returns_positive_180():
    # 180 is the one case with two equally-short paths; the convention here is +180.
    assert wrap_deg_error(180, 0) == pytest.approx(180)


def test_result_always_in_valid_range():
    import random

    random.seed(0)
    for _ in range(200):
        target = random.uniform(0, 360)
        current = random.uniform(0, 360)
        error = wrap_deg_error(target, current)
        assert -180.0 < error <= 180.0


def test_inputs_outside_0_360_still_work():
    # A raw heading reading is always 0-360, but a caller might pass a setpoint
    # that drifted outside that range through repeated addition - must not break.
    assert wrap_deg_error(370, 10) == pytest.approx(0)
    assert wrap_deg_error(-10, 10) == pytest.approx(-20)
