"""
Attitude-hold flight controller. NOT a full hover controller - read
firmware/README.md before running this, it explains exactly what this can and
cannot do and why.

In one sentence: this self-levels roll and pitch and holds a heading, at
whatever throttle firmware/tuning.py sets. It does not hold altitude or
position - nothing on this airframe measures either.

Safety model, in order of what actually stops the motors:

  1. MotorBank drives SLP low the instant it is constructed - before this file
     does anything else - so nothing can spin until arm() is explicitly called.
  2. tuning.LIVE_MOTORS must be True, or arm() is never called at all. Default
     is False: the whole control loop runs and prints what it WOULD do.
  3. With LIVE_MOTORS True, a "type ARM and press enter" prompt still has to be
     answered before arm() is called or the throttle ramp starts. This is a
     manual construction, not "with MotorBank() as bank" - that context manager
     arms unconditionally on entry, which would defeat the confirmation prompt
     entirely. Found by testing the abort path, not by inspection.
  4. Once armed, TILT_LIMIT_DEG and RUN_SECONDS auto-disarm on their own -
     there is no control link, so nothing else can tell this to stop.
  5. Ctrl-C, any exception, the early abort return, and a normal exit all route
     through a plain try/finally around bank.disarm(), so every path disarms.

None of this replaces a physical kill switch or a real control link with a
failsafe - both are still on the roadmap (docs/roadmap.md). Until then, keep a
hand on the battery connector.
"""

import sys
import time

sys.path.append("/")

from machine import I2C, Pin  # noqa: E402

try:
    import config  # noqa: E402
    import tuning  # noqa: E402
    from drivers import hmc5883l  # noqa: E402
    from drivers.heartbeat import Heartbeat  # noqa: E402
    from drivers.motors import MotorBank  # noqa: E402
    from drivers.mpu6050 import MPU6050  # noqa: E402
    from flight.angles import wrap_deg_error  # noqa: E402
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
        "  Thonny   : View -> Files. In the top pane select config.py, drivers,\n"
        "             flight and firmware/tuning.py, right-click -> 'Upload to /'\n"
        "  Terminal : ./tools/upload.sh\n" % exc
    )
    raise SystemExit()


def build_sensors():
    """IMU and magnetometer bring-up. Raises if either is missing - there is no
    sensible degraded mode for a flight controller with a dead gyro."""
    i2c = I2C(
        config.IMU_I2C_ID,
        sda=Pin(config.IMU_SDA_PIN),
        scl=Pin(config.IMU_SCL_PIN),
        freq=config.I2C_FREQ,
    )
    found = i2c.scan()
    print("I2C devices :", [hex(a) for a in found])

    imu = MPU6050(i2c=i2c)
    print("IMU         : WHO_AM_I 0x%02X" % imu.who_am_i)
    print("calibrating gyro - hold completely still...")
    print("  bias %.2f %.2f %.2f deg/s" % imu.calibrate_gyro())

    mag = hmc5883l.detect(i2c)
    mag.offset = tuning.MAG_OFFSET
    mag.scale = tuning.MAG_SCALE
    print("MAG         : %s (offset/scale from firmware/tuning.py)" % type(mag).__name__)

    return imu, mag


def confirm_arm():
    """The only arming gate this airframe has. Blocks until answered - if
    nothing is attached to the REPL, this blocks forever, which is the correct
    failure mode: no confirmation reachable means never armed."""
    print("\n!! LIVE_MOTORS is True - props must be on a restrained rig. !!")
    answer = input("Type ARM and press enter to spin up, anything else to abort: ")
    return answer.strip().upper() == "ARM"


def build_pids():
    """One PID per rate axis, plus two angle-loop PIDs. Yaw's angle equivalent
    is the heading-error P term computed inline in the main loop - heading is
    already an angle with nothing above it to cascade from."""
    rate = {
        "roll": PID(tuning.ROLL_RATE_KP, tuning.ROLL_RATE_KI, tuning.ROLL_RATE_KD,
                    integral_limit=tuning.RATE_INTEGRAL_LIMIT),
        "pitch": PID(tuning.PITCH_RATE_KP, tuning.PITCH_RATE_KI, tuning.PITCH_RATE_KD,
                     integral_limit=tuning.RATE_INTEGRAL_LIMIT),
        "yaw": PID(tuning.YAW_RATE_KP, tuning.YAW_RATE_KI, tuning.YAW_RATE_KD,
                   integral_limit=tuning.RATE_INTEGRAL_LIMIT),
    }
    angle = {
        "roll": PID(tuning.ROLL_ANGLE_KP, tuning.ROLL_ANGLE_KI, tuning.ROLL_ANGLE_KD,
                    output_limit=tuning.MAX_ANGLE_RATE_DPS),
        "pitch": PID(tuning.PITCH_ANGLE_KP, tuning.PITCH_ANGLE_KI, tuning.PITCH_ANGLE_KD,
                     output_limit=tuning.MAX_ANGLE_RATE_DPS),
    }
    return rate, angle


def control_step(imu, mag, fusion, rate_pid, angle_pid, heading_setpoint, dt):
    """
    One iteration of sensor read -> fusion -> cascaded PID -> mixer inputs.

    Returns (roll_deg, pitch_deg, heading, heading_error, roll_cmd, pitch_cmd,
    yaw_cmd) so the caller can both drive the mixer and print status without
    reading sensors twice.
    """
    accel, gyro, _ = imu.read()
    roll_deg, pitch_deg = fusion.roll_deg, fusion.pitch_deg
    fusion.update(accel, gyro, dt)
    gyro_x, gyro_y, gyro_z = gyro

    roll_rate_sp = angle_pid["roll"].update(0.0, roll_deg, dt)
    pitch_rate_sp = angle_pid["pitch"].update(0.0, pitch_deg, dt)

    heading = mag.heading(fusion.pitch, fusion.roll)
    heading_error = wrap_deg_error(heading_setpoint, heading)
    yaw_rate_sp = max(
        -tuning.MAX_YAW_RATE_DPS,
        min(tuning.MAX_YAW_RATE_DPS, tuning.HEADING_KP * heading_error),
    )

    roll_cmd = rate_pid["roll"].update(roll_rate_sp, gyro_x, dt)
    pitch_cmd = rate_pid["pitch"].update(pitch_rate_sp, gyro_y, dt)
    yaw_cmd = rate_pid["yaw"].update(yaw_rate_sp, gyro_z, dt)

    return roll_deg, pitch_deg, heading, heading_error, roll_cmd, pitch_cmd, yaw_cmd


def run():
    print("\n=== pico-drone attitude-hold flight controller ===")
    print("LIVE_MOTORS =", tuning.LIVE_MOTORS)
    if not tuning.LIVE_MOTORS:
        print("DRY RUN - motors cannot spin, MotorBank stays disarmed throughout.\n")

    imu, mag = build_sensors()
    fusion = ComplementaryFilter(alpha=tuning.COMPLEMENTARY_ALPHA)
    rate_pid, angle_pid = build_pids()

    # First reading seeds the fusion filter and locks the heading to hold.
    accel, gyro, _ = imu.read()
    fusion.update(accel, gyro, 0.002)
    heading_setpoint = mag.heading(fusion.pitch, fusion.roll)
    print("heading lock: %.1f degrees\n" % heading_setpoint)

    # Deliberately NOT "with MotorBank() as bank:" - MotorBank.__enter__ calls
    # arm() unconditionally, which would spin the motors before confirm_arm()
    # is even asked. Construct it disarmed, arm it explicitly only after a
    # real "ARM" confirmation, and guarantee disarm() on every exit path with
    # a bare try/finally instead of relying on the context manager for that.
    bank = MotorBank()
    heartbeat = Heartbeat().start()

    try:
        armed = False
        if tuning.LIVE_MOTORS:
            if confirm_arm():
                bank.arm()
                armed = True
                print("ARMED. Ramping to hover throttle over %.1fs..."
                      % tuning.THROTTLE_RAMP_SECONDS)
            else:
                print("Aborted - not armed.")
                return

        start = time.ticks_ms()
        last = time.ticks_us()
        last_print = time.ticks_ms()
        loop_count = 0
        stop_reason = "run completed"

        try:
            while True:
                now_ms = time.ticks_ms()
                elapsed_s = time.ticks_diff(now_ms, start) / 1000.0

                if elapsed_s >= tuning.RUN_SECONDS:
                    stop_reason = "RUN_SECONDS timeout (%.1fs)" % tuning.RUN_SECONDS
                    break

                now_us = time.ticks_us()
                dt = time.ticks_diff(now_us, last) / 1_000_000.0
                last = now_us
                loop_count += 1

                (roll_deg, pitch_deg, heading, heading_error,
                 roll_cmd, pitch_cmd, yaw_cmd) = control_step(
                    imu, mag, fusion, rate_pid, angle_pid, heading_setpoint, dt
                )

                if abs(roll_deg) > tuning.TILT_LIMIT_DEG or abs(pitch_deg) > tuning.TILT_LIMIT_DEG:
                    stop_reason = "TILT_LIMIT_DEG exceeded (roll %.1f pitch %.1f)" % (
                        roll_deg, pitch_deg,
                    )
                    break

                if armed:
                    throttle = min(
                        tuning.HOVER_THROTTLE,
                        tuning.HOVER_THROTTLE * elapsed_s / tuning.THROTTLE_RAMP_SECONDS,
                    )
                else:
                    throttle = tuning.HOVER_THROTTLE

                outputs = mix(throttle, roll_cmd, pitch_cmd, yaw_cmd, idle=tuning.IDLE_THROTTLE)

                if armed:
                    bank.set_many(outputs)
                    if bank.faulted():
                        stop_reason = "DRV8833 nFAULT tripped"
                        break

                if time.ticks_diff(now_ms, last_print) > 200:
                    last_print = now_ms
                    hz = loop_count / max(elapsed_s, 0.001)
                    print(
                        "t=%4.1fs  roll %+5.1f pitch %+5.1f hdg %5.1f (err %+5.1f)  "
                        "thr %.2f  m%s  %3.0fHz"
                        % (
                            elapsed_s, roll_deg, pitch_deg, heading, heading_error,
                            throttle,
                            {k: round(v, 2) for k, v in outputs.items()},
                            hz,
                        )
                    )

        except KeyboardInterrupt:
            stop_reason = "keyboard interrupt"

        elapsed_s = max(time.ticks_diff(time.ticks_ms(), start) / 1000.0, 0.001)
        print("\nstopped: %s" % stop_reason)
        print("average loop rate: %.0f Hz over %.1fs (%d iterations)"
              % (loop_count / elapsed_s, elapsed_s, loop_count))

    finally:
        # Runs on every exit path: normal completion, the early abort return,
        # any exception, Ctrl-C.
        bank.disarm()
        heartbeat.stop()


if __name__ == "__main__":
    run()
