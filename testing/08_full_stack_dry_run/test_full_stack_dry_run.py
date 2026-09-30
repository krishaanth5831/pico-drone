"""
Full-stack dry run: every component working together, motors NOT attached.

This is the integration test. 01-07 each prove one part in isolation; this one
closes the whole loop - IMU -> fusion -> PID -> mixer -> MotorBank - and prints
the four throttles it WOULD send, at the real loop rate, with the drivers held
hardware-disarmed the entire time.

WHAT MAKES THIS SAFE
--------------------
SLP is never raised. MotorBank is constructed (which drives SLP low) and
arm() is never called, so both DRV8833s stay asleep and their outputs stay
high-impedance no matter what lands in the PWM registers. The throttles are
computed and applied to the PWM peripheral so the numbers you see are the real
ones, but nothing downstream of the driver chip can act on them.

  >> Motors must not be attached. There is no control link on this airframe,
  >> so there is no way to command a stop if something did spin.

WHAT IT PROVES
--------------
  - every sensor initialises and keeps returning data alongside the others,
    sharing I2C0 and UART0 without starving each other
  - the fusion filter tracks real movement
  - the PID + mixer chain produces sane, correctly-signed motor commands
  - the whole loop runs fast enough to fly on (watch the Hz figure)

WHAT IT DOES NOT PROVE
----------------------
  Anything about the PID VALUES. They are guesses - see GAINS below. Tuning
  needs a real airframe in the air, and this test cannot tell you a gain is
  wrong, only that the arithmetic runs.

HOW TO USE IT
-------------
Tilt the board by hand and watch the motor column respond. These are
CORRECTIONS, so the loop pushes back toward level: roll the board to the right
and the right-hand motors (M1 front-right, M4 rear-right) should rise while the
left-hand pair (M2, M3) falls. That asymmetry is the test passing.

If it moves the opposite way, the fix is the accelerometer axis signs in
src/flight/fusion.py, not the gains.
"""

import sys
import time

sys.path.append("/")
try:
    import config  # noqa: E402
    from drivers.heartbeat import Heartbeat  # noqa: E402
    from drivers.motors import MotorBank  # noqa: E402
    from flight.fusion import ComplementaryFilter  # noqa: E402
    from flight.mixer import mix  # noqa: E402
    from flight.pid import PID  # noqa: E402
except ImportError as exc:
    print(
        "\n%s\n\n"
        "The library modules are not on the Pico yet. These imports resolve\n"
        "against the BOARD's filesystem, not your computer's, so opening this\n"
        "file in an editor is not enough.\n\n"
        "Upload them once, then run this again:\n"
        "  Thonny   : View -> Files. In the top pane select config.py, drivers\n"
        "             and flight, right-click -> 'Upload to /'\n"
        "  Terminal : ./tools/upload.sh\n" % exc
    )
    raise SystemExit()

# --- GAINS: GUESSES, NOT TUNED ----------------------------------------------
# Starting points only, picked to be visibly responsive but too soft to
# oscillate. They have never been near an airframe in flight. Do NOT fly on
# them - tune on a test rig, then move the final numbers into src/config.py.
#
# Roll and pitch are ANGLE loops: error is degrees off level, output is a
# mixer term nominally -1..1. kp=0.020 means a 30 deg tilt asks for 0.60 of
# correction authority, which is firm without saturating the mixer.
#
# Yaw is a RATE loop: error is deg/s of rotation. There is no heading hold
# here because without a control link there is nothing to hold a heading for -
# it just damps rotation toward zero. Much smaller kp: the units are ~10x
# bigger (deg/s, not deg).
ANGLE_KP, ANGLE_KI, ANGLE_KD = 0.020, 0.010, 0.0040
YAW_KP, YAW_KI, YAW_KD = 0.0030, 0.0, 0.0

# With kp=0.020 and OUTPUT_LIMIT=0.50 the angle loop saturates at 25 deg of
# tilt: past that the PID output stops growing and the motor spread stops
# widening. Tilting through that point and watching the numbers stop moving is
# a useful thing to see, but the gain is almost certainly too hot for flight,
# where corrections live in the 0-5 deg band.
#
# Integral authority is deliberately well under the output limit. A guessed ki
# that is allowed to wind all the way to 1.0 will dominate the output and mask
# whether kp is doing anything.
INTEGRAL_LIMIT = 0.25
OUTPUT_LIMIT = 0.50

# Collective throttle to mix around. Nothing spins, so this only sets the
# operating point the corrections are added to - pick mid-range so you can see
# motors move both up and down from it.
SIM_THROTTLE = 0.45

# Setpoints. Level and not rotating: the aircraft's job here is to hold still.
TARGET_ROLL_DEG = 0.0
TARGET_PITCH_DEG = 0.0
TARGET_YAW_RATE = 0.0

PRINT_MS = 200


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
    print("\n=== full-stack dry run ===")
    print(">> MOTORS MUST NOT BE ATTACHED <<")
    print("drivers stay asleep (SLP low) for the whole run\n")

    heartbeat = Heartbeat().start()

    # Constructing the bank is what drives SLP low, so do it before anything
    # that could fail. arm() is never called anywhere in this file.
    motors = MotorBank()
    print("MOTORS  disarmed (SLP low on GP%d), arm() is never called"
          % config.MOTOR_SLEEP_PIN)

    sensors = bring_up()

    if sensors["imu"] is None:
        print("\nno IMU - the control loop has nothing to close around.")
        print("fix wiring (see testing/04_gy521_imu) and re-run.")
        heartbeat.stop()
        Heartbeat(mode="blink").start()
        try:
            while True:
                time.sleep_ms(500)
        except KeyboardInterrupt:
            heartbeat.stop()
            return

    imu = sensors["imu"]
    mag = sensors["mag"]
    gps = sensors["gps"]

    fusion = ComplementaryFilter()
    roll_pid = PID(ANGLE_KP, ANGLE_KI, ANGLE_KD, INTEGRAL_LIMIT, OUTPUT_LIMIT)
    pitch_pid = PID(ANGLE_KP, ANGLE_KI, ANGLE_KD, INTEGRAL_LIMIT, OUTPUT_LIMIT)
    yaw_pid = PID(YAW_KP, YAW_KI, YAW_KD, INTEGRAL_LIMIT, OUTPUT_LIMIT)

    print("\ngains (GUESSES - not flight-tuned)")
    print("  angle  kp %.4f  ki %.4f  kd %.4f" % (ANGLE_KP, ANGLE_KI, ANGLE_KD))
    print("  yaw    kp %.4f  ki %.4f  kd %.4f" % (YAW_KP, YAW_KI, YAW_KD))
    print("  collective throttle %.2f, MAX_DUTY %.2f"
          % (SIM_THROTTLE, config.MAX_DUTY))

    print("\ntilt the board by hand and watch M1-M4 respond.")
    print("these are corrections: roll RIGHT -> M1,M4 up, M2,M3 down.")
    print("spread widens with tilt, stops growing past ~25 deg.")
    print("ctrl-C to stop.\n")

    last = time.ticks_us()
    last_print = time.ticks_ms()
    loops = 0
    worst_dt_ms = 0.0

    try:
        while True:
            now = time.ticks_us()
            dt = time.ticks_diff(now, last) / 1_000_000.0
            last = now

            # First pass through has a meaningless dt. Skip it rather than
            # feeding a garbage interval into the integrator.
            if dt <= 0.0 or dt > 0.5:
                continue

            accel, gyro, _ = imu.read()
            fusion.update(accel, gyro, dt)

            # Angle loops on roll/pitch, rate loop on yaw. Setpoint and
            # measurement go in as-is: PID differentiates the measurement
            # internally, so it needs the real angle, not a pre-computed error.
            roll_out = roll_pid.update(TARGET_ROLL_DEG, fusion.roll_deg, dt)
            pitch_out = pitch_pid.update(TARGET_PITCH_DEG, fusion.pitch_deg, dt)
            yaw_out = yaw_pid.update(TARGET_YAW_RATE, gyro[2], dt)

            # idle=0.0 deliberately, unlike flight. MIN_START exists to stop
            # coreless motors spooling from a dead stop, but nothing is
            # spinning here and the floor is a hard clamp applied AFTER the
            # mixer desaturates - with idle=MIN_START the low pair pins at 0.20
            # from about 13 deg of tilt and the differential you are supposed
            # to be reading goes flat. Zero floor keeps every printed number
            # the true PID + mixer output.
            throttles = mix(SIM_THROTTLE, roll_out, pitch_out, yaw_out, idle=0.0)

            # Written to the PWM registers so the printed numbers are the real
            # commanded values - not a simulation running beside the hardware.
            # SLP is low, so the driver outputs stay high-impedance regardless.
            motors.set_many(throttles)

            if gps is not None:
                gps.update()

            loops += 1
            dt_ms = dt * 1000.0
            if dt_ms > worst_dt_ms:
                worst_dt_ms = dt_ms

            if time.ticks_diff(time.ticks_ms(), last_print) > PRINT_MS:
                hz = loops * 1000.0 / PRINT_MS
                loops = 0

                line = "r%+6.1f p%+6.1f" % (fusion.roll_deg, fusion.pitch_deg)
                line += " | pid r%+5.2f p%+5.2f y%+5.2f" % (
                    roll_out, pitch_out, yaw_out
                )
                line += " | M1 %.2f M2 %.2f M3 %.2f M4 %.2f" % (
                    throttles[1], throttles[2], throttles[3], throttles[4]
                )
                line += " | %4.0fHz" % hz

                if mag is not None:
                    line += " hdg%4.0f" % mag.heading(fusion.pitch, fusion.roll)
                if gps is not None:
                    line += " gps%2d" % gps.fix.satellites
                if motors.faulted():
                    line += " [DRV FAULT]"

                print(line)
                last_print = time.ticks_ms()

    except KeyboardInterrupt:
        print("\nstopped")
        print("worst loop interval: %.1f ms" % worst_dt_ms)
    finally:
        # The bank was never armed, but never leave by any path without
        # cutting the drivers anyway.
        motors.disarm()
        heartbeat.stop()
        print("=== disarmed, LED off ===")


if __name__ == "__main__":
    main()
