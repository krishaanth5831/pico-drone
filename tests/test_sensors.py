"""IMU and magnetometer drivers, driven through the fake I2C bus."""

import machine
import pytest

from drivers import hmc5883l
from drivers.mpu6050 import MPU6050


def _mpu_bus(who_am_i=0x68):
    return machine.I2C(registers={0x68: {0x75: bytes([who_am_i])}})


def test_mpu_rejects_wrong_who_am_i():
    bus = machine.I2C(registers={0x68: {0x75: b"\xFF"}})
    with pytest.raises(OSError, match="WHO_AM_I"):
        MPU6050(i2c=bus)


@pytest.mark.parametrize("who", [0x68, 0x70, 0x71, 0x73])
def test_mpu_accepts_known_variants(who):
    imu = MPU6050(i2c=_mpu_bus(who))
    assert imu.who_am_i == who


def test_mpu_wakes_the_chip():
    bus = _mpu_bus()
    MPU6050(i2c=bus)
    # PWR_MGMT_1 written: first 0x00 to clear sleep, then 0x01 for the gyro clock.
    power_writes = [data for addr, reg, data in bus.writes if reg == 0x6B]
    assert power_writes == [b"\x00", b"\x01"]


def test_mpu_scaling_matches_selected_range():
    bus = _mpu_bus()
    # 0x4000 raw on a +/-2 g range is exactly 1 g.
    bus.registers[0x68][0x3B] = (
        (0x4000).to_bytes(2, "big") + b"\x00" * 12
    )
    imu = MPU6050(i2c=bus, accel_range=2, gyro_range=250)
    accel, _, _ = imu.read()
    assert accel[0] == pytest.approx(9.80665, rel=1e-3)


def test_gyro_bias_is_subtracted():
    bus = _mpu_bus()
    bus.registers[0x68][0x3B] = b"\x00" * 8 + (131).to_bytes(2, "big") + b"\x00" * 4
    imu = MPU6050(i2c=bus, gyro_range=250)
    _, gyro_before, _ = imu.read()
    assert gyro_before[0] == pytest.approx(1.0, rel=1e-2)
    imu.gyro_bias = (1.0, 0.0, 0.0)
    _, gyro_after, _ = imu.read()
    assert gyro_after[0] == pytest.approx(0.0, abs=1e-6)


def test_detect_finds_honeywell_hmc5883l():
    bus = machine.I2C(registers={0x1E: {0x0A: b"H43"}})
    assert isinstance(hmc5883l.detect(bus), hmc5883l.HMC5883L)


def test_detect_finds_qmc5883l_clone():
    bus = machine.I2C(registers={0x0D: {}})
    assert isinstance(hmc5883l.detect(bus), hmc5883l.QMC5883L)


def test_detect_error_names_both_expected_addresses():
    bus = machine.I2C(registers={0x68: {}})
    with pytest.raises(OSError, match="0x1E"):
        hmc5883l.detect(bus)


def test_hmc_data_register_order_is_x_z_y():
    # The HMC5883L returns X, Z, Y - not X, Y, Z. Reading it in the obvious
    # order gives a compass that is wrong by 90 degrees but looks plausible.
    bus = machine.I2C(registers={0x1E: {0x0A: b"H43"}})
    mag = hmc5883l.detect(bus)
    bus.registers[0x1E][0x03] = (
        (100).to_bytes(2, "big") + (300).to_bytes(2, "big") + (200).to_bytes(2, "big")
    )
    x, y, z = mag.read_raw_ut()
    assert x < y < z, "expected X=100 Y=200 Z=300 after X,Z,Y unpacking"


def test_heading_is_zero_to_360():
    bus = machine.I2C(registers={0x1E: {0x0A: b"H43"}})
    mag = hmc5883l.detect(bus)
    for xr, yr in ((100, 0), (0, 100), (-100, 0), (0, -100)):
        bus.registers[0x1E][0x03] = (
            xr.to_bytes(2, "big", signed=True)
            + b"\x00\x00"
            + yr.to_bytes(2, "big", signed=True)
        )
        assert 0.0 <= mag.heading() < 360.0
