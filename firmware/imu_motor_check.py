"""
Which motor knocks the IMU off the I2C bus?

Open in Thonny, press Run. Props off. Runs each motor ON ITS OWN at THROTTLE
for HOLD_S seconds while reading the GY-521 as fast as it can, and counts the
failed reads. Before and after the motors it does the same with everything
armed but nothing turning, as a baseline.

Reading the result:
  baseline 0 failed, one motor bad   -> that motor's wiring: its leads run near
                                        SDA/SCL, or its current returns through
                                        the sensors' ground
  baseline 0 failed, every motor bad -> shared power/ground: IMU VCC or GND on
                                        the motor supply (HW-131) instead of the Pico
  baseline already failing           -> not the motors at all - IMU wiring
"""

import sys
import time

sys.path.append("/")

from machine import I2C, Pin  # noqa: E402

try:
    import config  # noqa: E402
    from drivers.motors import MotorBank, ramp  # noqa: E402
    from drivers.mpu6050 import MPU6050  # noqa: E402
except ImportError as exc:
    print("\n%s\n\nLibrary not on the Pico. In Thonny View -> Files, select "
          "config.py, drivers, flight -> 'Upload to /'.\n" % exc)
    raise SystemExit()

THROTTLE = 0.40   # same as lift.py's IDLE
HOLD_S = 3.0
I2C_FREQ = 100_000


def new_imu():
    # Building a fresh MPU6050 also wakes it, in case it browned out and reset
    # into sleep mode.
    return MPU6050(i2c=I2C(config.IMU_I2C_ID, sda=Pin(config.IMU_SDA_PIN),
                           scl=Pin(config.IMU_SCL_PIN), freq=I2C_FREQ))


def hammer(imu, seconds):
    """Read the IMU for `seconds`. Returns (imu, reads, failed)."""
    reads = failed = 0
    end = time.ticks_add(time.ticks_ms(), int(seconds * 1000))
    while time.ticks_diff(end, time.ticks_ms()) > 0:
        reads += 1
        try:
            imu.read()
        except OSError:
            failed += 1
            try:
                imu = new_imu()
            except OSError:
                pass
    return imu, reads, failed


def report(label, reads, failed):
    flag = "  <-- IMU dropping out" if failed else ""
    print("%-22s %4d / %4d reads failed%s" % (label, failed, reads, flag))


print("\n=== IMU vs motors check (props OFF) ===")
try:
    imu = new_imu()
except OSError:
    print("IMU not answering even with the motors off - check SDA -> GP4 (pin 6),")
    print("SCL -> GP5 (pin 7), VCC -> pin 36, GND -> pin 38.")
    raise SystemExit()

imu, reads, failed = hammer(imu, HOLD_S)
report("motors asleep", reads, failed)

with MotorBank() as bank:          # armed: SLP high, every throttle 0
    try:
        imu, reads, failed = hammer(imu, HOLD_S)
        report("armed, none turning", reads, failed)

        for m in sorted(config.MOTOR_PINS):
            ramp(bank, m, 0.0, THROTTLE, 0.4)
            imu, reads, failed = hammer(imu, HOLD_S)
            bank.set(m, 0.0)
            report("motor %d only" % m, reads, failed)
            time.sleep(0.5)

        for m in sorted(config.MOTOR_PINS):
            ramp(bank, m, 0.0, THROTTLE, 0.4)
        imu, reads, failed = hammer(imu, HOLD_S)
        report("all four", reads, failed)
    except KeyboardInterrupt:
        print("\ninterrupted")

imu, reads, failed = hammer(imu, HOLD_S)
report("after, motors asleep", reads, failed)
print("=== done, disarmed ===")
