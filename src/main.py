"""
Entry point. Copy the contents of src/ to the Pico's filesystem and this runs at
power-on.

WHAT THIS DOES NOT DO: fly. There is no control link on this airframe yet, so
there is nothing to command it. Arming motors with no way to command or stop
them would be dangerous, so main.py deliberately stops at sensor bring-up.

Right now it:
  - holds the motors hardware-disarmed
  - brings up the IMU, magnetometer and GPS
  - streams attitude to the REPL so you can watch the fusion filter work

The onboard LED pulses for as long as this runs - a long on, short off that
reads as a glow. It is a liveness indicator: if it stops and restarts, the board
reset, which on this airframe means a brownout. A fast even blink instead means
a sensor failed to initialise; see the REPL for which.
"""

import sys
import time

sys.path.append("/")

import config  # noqa: E402
from drivers.heartbeat import Heartbeat  # noqa: E402
from drivers.motors import MotorBank  # noqa: E402
from flight.fusion import ComplementaryFilter  # noqa: E402


def bring_up():
    """Initialise every sensor, reporting rather than raising on failure."""
    sensors = {"imu": None, "mag": None, "gps": None}

    try:
        from drivers.mpu6050 import MPU6050

        imu = MPU6050()
        print("IMU     ok  (WHO_AM_I 0x%02X)" % imu.who_am_i)
        print("calibrating gyro - hold still...")
        print("  bias %.2f %.2f %.2f deg/s" % imu.calibrate_gyro())
        sensors["imu"] = imu
    except Exception as exc:  # noqa: BLE001 - report every failure, never abort
        print("IMU     FAILED:", exc)

    try:
        from drivers import hmc5883l

        sensors["mag"] = hmc5883l.detect()
        print("MAG     ok  (%s)" % type(sensors["mag"]).__name__)
    except Exception as exc:  # noqa: BLE001
        print("MAG     FAILED:", exc)

    try:
        from drivers.gps import GPS

        sensors["gps"] = GPS()
        print("GPS     ok  (listening, fix takes 30s+ outdoors)")
    except Exception as exc:  # noqa: BLE001
        print("GPS     FAILED:", exc)

    return sensors


def main():
    print("\n=== pico-drone bring-up ===")

    # Lit before anything else, so a failure during bring-up still leaves you a
    # board that is visibly alive.
    heartbeat = Heartbeat().start()

    # Motors are constructed disarmed and stay that way. Constructing MotorBank
    # is what drives SLP low, so do it first and before anything can fail.
    motors = MotorBank()
    print("MOTORS  disarmed (SLP low on GP%d)" % config.MOTOR_SLEEP_PIN)

    sensors = bring_up()

    if sensors["imu"] is None:
        print("\nno IMU - nothing useful to report. fix wiring and re-run.")
        # Switch to an even fast blink so the fault is distinguishable from the
        # normal glow across the room.
        heartbeat.stop()
        Heartbeat(mode="blink").start()
        while True:
            time.sleep_ms(500)

    fusion = ComplementaryFilter()
    imu = sensors["imu"]
    gps = sensors["gps"]

    print("\nstreaming attitude. ctrl-C to stop.\n")
    last = time.ticks_us()
    last_print = time.ticks_ms()

    try:
        while True:
            now = time.ticks_us()
            dt = time.ticks_diff(now, last) / 1_000_000.0
            last = now

            accel, gyro, _ = imu.read()
            roll, pitch = fusion.update(accel, gyro, dt)

            if gps is not None:
                gps.update()

            if time.ticks_diff(time.ticks_ms(), last_print) > 200:
                last_print = time.ticks_ms()
                line = "roll %+6.1f  pitch %+6.1f" % (
                    fusion.roll_deg,
                    fusion.pitch_deg,
                )
                if sensors["mag"] is not None:
                    line += "  hdg %5.1f" % sensors["mag"].heading(pitch, roll)
                if gps is not None and gps.fix.valid:
                    line += "  gps %.5f,%.5f" % (gps.fix.lat, gps.fix.lon)
                elif gps is not None:
                    line += "  gps %d sats" % gps.fix.satellites
                if motors.faulted():
                    line += "  [DRV FAULT]"
                print(line)

    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        # Belt and braces: the bank is already disarmed, but never leave this
        # function by any path without cutting the drivers.
        motors.disarm()
        heartbeat.stop()


if __name__ == "__main__":
    main()
