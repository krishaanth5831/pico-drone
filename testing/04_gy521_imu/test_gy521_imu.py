"""
GY-521 / MPU6050 bring-up.

Scans I2C, identifies the chip, calibrates the gyro at rest, then streams a live
attitude estimate through the complementary filter so you can tilt the board and
watch the numbers respond.

The onboard LED pulses for as long as this runs. No motors are touched.
"""

import sys
import time

sys.path.append("/")

from machine import I2C, Pin  # noqa: E402

import config  # noqa: E402
from drivers.heartbeat import Heartbeat  # noqa: E402
from drivers.mpu6050 import MPU6050  # noqa: E402
from flight.fusion import ComplementaryFilter  # noqa: E402

print("\n=== GY-521 / MPU6050 test ===")
print("LED pulses for as long as this runs\n")

with Heartbeat():
    i2c = I2C(
        config.IMU_I2C_ID,
        sda=Pin(config.IMU_SDA_PIN),
        scl=Pin(config.IMU_SCL_PIN),
        freq=config.I2C_FREQ,
    )

    found = i2c.scan()
    print("I2C devices :", [hex(a) for a in found])
    if config.IMU_ADDR not in found:
        raise SystemExit(
            "no MPU6050 at 0x%02X - check SDA on GP%d (pin 6) and SCL on GP%d (pin 7)"
            % (config.IMU_ADDR, config.IMU_SDA_PIN, config.IMU_SCL_PIN)
        )

    imu = MPU6050(i2c=i2c)
    names = {0x68: "MPU6050", 0x70: "MPU6500", 0x71: "MPU9250", 0x73: "MPU9255"}
    print("WHO_AM_I    : 0x%02X (%s)" % (imu.who_am_i, names.get(imu.who_am_i, "clone")))

    _, _, temperature = imu.read()
    print("temperature : %.1f C" % temperature)

    print("\ncalibrating gyro - hold completely still...")
    bias = imu.calibrate_gyro()
    print("  bias %6.2f %6.2f %6.2f deg/s" % bias)

    fusion = ComplementaryFilter()
    print("\nstreaming - tilt the board, ctrl-C to stop")

    last = time.ticks_us()
    last_print = time.ticks_ms()

    try:
        while True:
            now = time.ticks_us()
            dt = time.ticks_diff(now, last) / 1_000_000.0
            last = now

            accel, gyro, _ = imu.read()
            fusion.update(accel, gyro, dt)

            if time.ticks_diff(time.ticks_ms(), last_print) > 150:
                last_print = time.ticks_ms()
                magnitude = (accel[0] ** 2 + accel[1] ** 2 + accel[2] ** 2) ** 0.5
                print(
                    "  roll %+6.1f  pitch %+6.1f  |  gyro %5.1f %5.1f %5.1f  |  accel %5.2f"
                    % (
                        fusion.roll_deg,
                        fusion.pitch_deg,
                        gyro[0],
                        gyro[1],
                        gyro[2],
                        magnitude,
                    )
                )

            time.sleep_ms(2)

    except KeyboardInterrupt:
        print("\nstopped")

print("LED off - script ended")
