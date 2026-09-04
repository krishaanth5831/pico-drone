"""Motor mixer. Sign errors here produce an aircraft that flips on takeoff."""

import pytest

from flight.mixer import MOTOR_DIRECTIONS, mix


def test_level_hover_is_symmetric():
    out = mix(0.5, 0.0, 0.0, 0.0)
    assert set(out) == {1, 2, 3, 4}
    assert all(value == pytest.approx(0.5) for value in out.values())


def test_roll_right_lifts_left_side():
    # M2 rear-left and M3 front-left speed up; M1 and M4 on the right slow down.
    out = mix(0.5, 0.2, 0.0, 0.0)
    assert out[2] > 0.5 and out[3] > 0.5
    assert out[1] < 0.5 and out[4] < 0.5


def test_pitch_forward_lifts_rear():
    out = mix(0.5, 0.0, 0.2, 0.0)
    assert out[2] > 0.5 and out[4] > 0.5   # rear pair
    assert out[1] < 0.5 and out[3] < 0.5   # front pair


def test_yaw_speeds_up_the_counter_rotating_pair():
    # To yaw right you accelerate the motors spinning the opposite way. M1 and M2
    # are the CCW pair, so a positive yaw command must raise exactly those.
    out = mix(0.5, 0.0, 0.0, 0.2)
    ccw = [m for m, d in MOTOR_DIRECTIONS.items() if d == "CCW"]
    cw = [m for m, d in MOTOR_DIRECTIONS.items() if d == "CW"]
    assert all(out[m] > 0.5 for m in ccw)
    assert all(out[m] < 0.5 for m in cw)


def test_diagonal_pairs_rotate_together():
    # M1/M2 and M3/M4 are the diagonals; each pair must share a direction or the
    # yaw torques will not cancel in the hover.
    assert MOTOR_DIRECTIONS[1] == MOTOR_DIRECTIONS[2]
    assert MOTOR_DIRECTIONS[3] == MOTOR_DIRECTIONS[4]
    assert MOTOR_DIRECTIONS[1] != MOTOR_DIRECTIONS[3]


def test_output_always_within_range():
    for throttle in (0.0, 0.25, 0.5, 0.75, 1.0):
        for axis in (-1.0, -0.5, 0.0, 0.5, 1.0):
            for out in (
                mix(throttle, axis, 0, 0),
                mix(throttle, 0, axis, 0),
                mix(throttle, 0, 0, axis),
                mix(throttle, axis, axis, axis),
            ):
                assert all(0.0 <= v <= 1.0 for v in out.values())


def test_saturation_preserves_attitude_differences():
    # At high throttle plus a big roll command the naive sum exceeds 1.0.
    # Desaturation must shift all four down together, keeping the *differences*
    # intact - clipping one motor instead would distort the commanded torque.
    out = mix(0.95, 0.3, 0.0, 0.0)
    assert max(out.values()) <= 1.0
    assert out[2] - out[1] == pytest.approx(0.6, abs=1e-6)
    assert out[3] - out[4] == pytest.approx(0.6, abs=1e-6)


def test_idle_floor_applied_only_when_throttle_up():
    assert all(v >= 0.1 for v in mix(0.3, 0, 0, 0, idle=0.1).values())
    # Throttle at zero means truly stopped, regardless of idle.
    assert all(v == 0.0 for v in mix(0.0, 0, 0, 0, idle=0.1).values())
