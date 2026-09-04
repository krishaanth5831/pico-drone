"""
HMC5883L / QMC5883L magnetometer bring-up.

Detects which of the two chips is fitted, runs a min/max calibration, then
streams live compass heading so you can rotate the board and watch it track.

The onboard LED pulses for as long as this runs. No motors are touched. For a
meaningful calibration, run it with the aircraft assembled and battery connected.
"""

import sys
import time

sys.path.append("/")

from machine import I2C, Pin  # noqa: E402

import config  # noqa: E402
from drivers import hmc5883l  # noqa: E402
from drivers.heartbeat import Heartbeat  # noqa: E402

CALIBRATION_S = 30

print("\n=== HMC5883L / QMC5883L test ===")
print("LED pulses for as long as this runs\n")

with Heartbeat():
    i2c = I2C(
        config.IMU_I2C_ID,
        sda=Pin(config.IMU_SDA_PIN),
        scl=Pin(config.IMU_SCL_PIN),
        freq=config.I2C_FREQ,
    )
    print("I2C devices :", [hex(a) for a in i2c.scan()])

    mag = hmc5883l.detect(i2c)
    print("detected    :", type(mag).__name__)

    print("\ncalibration: rotate slowly through all orientations for %ds" % CALIBRATION_S)
    print("(a lazy figure-of-eight in all three axes)")
    mag.calibrate(seconds=CALIBRATION_S)

    print("\nstreaming - rotate the board, ctrl-C to stop")
    try:
        while True:
            x, y, z = mag.read()
            field = (x * x + y * y + z * z) ** 0.5
            print(
                "  heading %5.1f  |  x %6.1f  y %6.1f  z %6.1f  |  field %5.1f uT"
                % (mag.heading(), x, y, z, field)
            )
            time.sleep_ms(250)
    except KeyboardInterrupt:
        print("\nstopped")

print("LED off - script ended")
