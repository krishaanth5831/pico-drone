"""Complementary filter."""

import math

import pytest

from flight.fusion import ComplementaryFilter

GRAVITY = 9.80665
LEVEL = (0.0, 0.0, GRAVITY)


def test_level_reads_zero():
    f = ComplementaryFilter()
    roll, pitch = f.update(LEVEL, (0.0, 0.0, 0.0), 0.002)
    assert roll == pytest.approx(0.0, abs=1e-6)
    assert pitch == pytest.approx(0.0, abs=1e-6)


def test_seeds_from_accelerometer_on_first_call():
    # Starting from a 30 degree tilt, the first update must adopt that angle
    # outright rather than easing toward it from zero.
    f = ComplementaryFilter()
    tilt = math.radians(30.0)
    accel = (0.0, GRAVITY * math.sin(tilt), GRAVITY * math.cos(tilt))
    roll, _ = f.update(accel, (0.0, 0.0, 0.0), 0.002)
    assert math.degrees(roll) == pytest.approx(30.0, abs=0.5)


def test_gyro_integrates_between_accel_corrections():
    f = ComplementaryFilter(alpha=1.0)  # gyro only
    f.update(LEVEL, (0.0, 0.0, 0.0), 0.002)
    for _ in range(100):
        f.update(LEVEL, (90.0, 0.0, 0.0), 0.01)  # 90 deg/s for 1 s
    assert f.roll_deg == pytest.approx(90.0, abs=1.0)


def test_accelerometer_pulls_estimate_back():
    # With drift-free accel data the estimate must converge to level, not
    # wander off with the gyro's bias.
    f = ComplementaryFilter(alpha=0.98)
    f.update(LEVEL, (0.0, 0.0, 0.0), 0.002)
    for _ in range(2000):
        f.update(LEVEL, (5.0, 0.0, 0.0), 0.002)  # 5 deg/s of pure bias
    assert abs(f.roll_deg) < 15.0


def test_degree_properties_match_radians():
    f = ComplementaryFilter()
    f.update((0.0, 3.0, 9.0), (0.0, 0.0, 0.0), 0.002)
    assert f.roll_deg == pytest.approx(math.degrees(f.roll))
    assert f.pitch_deg == pytest.approx(math.degrees(f.pitch))
