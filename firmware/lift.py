"""
Lift test: ramp up, try to leave the ground while self-levelling, ramp down.

Open this in Thonny and press Run. Every number you should need to touch while
tuning is in the block below, in THIS file - not in tuning.py - because
tuning.py is imported from the board's copy, so editing it in Thonny without
re-uploading silently changes nothing. Edit here, press Run, done.

At the prompt:
  ARM  -> motors spin, runs the lift profile
  DRY  -> whole control loop runs and prints, motors stay asleep (SLP low)
  else -> abort

Flight profile, all automatic (there is no control link yet):
  0 -> LIFT_THROTTLE over RAMP_UP_S, hold for HOLD_S, back to 0 over RAMP_DOWN_S.
  HOLD_S = None holds forever - press Thonny's Stop button (Ctrl-C) to end it.
Auto-disarms on tilt past TILT_LIMIT_DEG, on nFAULT, on Ctrl-C, on any error.

No altitude sensor exists, so "lift" here means open-loop throttle with the
attitude loop holding it level. If it doesn't leave the ground, raise
LIFT_THROTTLE. If it shoots up, lower it. Keep a hand on the battery lead.

If it stops with "IMU stopped answering" or "IMU readings frozen": the motors
are disturbing the GY-521, which is a wiring/power problem, not code. In order:
  - 10 uF (or bigger) + 100 nF capacitor across GY-521 VCC and GND, at the board
  - SDA/SCL wires short, twisted with a GND wire, routed away from motor wires
  - fully charged battery; big (470 uF) capacitor across each DRV8833 VM/GND
  - Pico powered from the battery through the Schottky, not sharing motor wires
"""

import sys
import time

sys.path.append("/")

from machine import I2C, PWM, Pin  # noqa: E402

try:
    import config  # noqa: E402
    from drivers.heartbeat import Heartbeat  # noqa: E402
    from drivers.mpu6050 import MPU6050  # noqa: E402
    from flight.fusion import ComplementaryFilter  # noqa: E402
    from flight.mixer import mix  # noqa: E402
    from flight.pid import PID  # noqa: E402
except ImportError as exc:
    print("\n%s\n\nLibrary not on the Pico. Run ./tools/upload.sh, or in Thonny\n"
          "View -> Files, select config.py, drivers, flight -> 'Upload to /'.\n" % exc)
    raise SystemExit()


# =============================================================================
# EDIT HERE
# =============================================================================

# How the motors are wired to the DRV8833 outputs. Both modes use the same four
# Pico pins (GP10, GP11 on DRV #1 and GP12, GP13 on DRV #2); only the DRV side
# differs.
#
# "STANDARD"  (recommended - both motor leads on the driver, nothing to GND)
#     Each board: motor A across OUT1 + OUT2, motor B across OUT3 + OUT4.
#     IN1 -> GP10 / GP12, IN3 -> GP11 / GP13, IN2 and IN4 -> GND.
#     Each motor has its own H-bridge and gets up to config.MAX_DUTY (70%), and
#     its current returns through the driver's own GND pin instead of through
#     the breadboard ground the IMU sits on.
#
# "SHARED"  (motor A OUT1 -> GND, motor B OUT2 -> GND)
#     IN1 -> GP10 / GP12, IN2 -> GP11 / GP13.  IN2 must NOT be tied to GND.
#     Both motors share ONE H-bridge. The DRV8833 drives both outputs LOW when
#     IN1 and IN2 are both high, which brakes both motors, so the two PWM
#     pulses are interleaved: motor A fires at the start of each 50 us period,
#     motor B at the end. Each motor gets at most 50% of the time - roughly
#     half the thrust - and all motor current flows through the ground rail.
WIRING = "SHARED"

# --- throttle profile (0.0-1.0, scaled to the wiring's duty ceiling) --------
LIFT_THROTTLE = 0.50   # raise if it won't leave the ground, lower if it rockets
RAMP_UP_S = 2.5        # slow ramp: a step to full current browns out a 1S pack
HOLD_S = None          # time at LIFT_THROTTLE; None = hold until Stop / Ctrl-C
RAMP_DOWN_S = 2.0      # descent ramp back to zero
IDLE = 0.30            # mixer floor while throttle > 0

# --- safety -------------------------------------------------------------------
TILT_LIMIT_DEG = 35.0  # auto-disarm past this roll or pitch

# --- PID gains: guesses for a ~50-80 g coreless quad, tune by hand ------------
# Rate loop: gyro error in deg/s -> motor correction (fraction of throttle).
# 0.004 means a 100 deg/s error gives a 0.4 correction. If it wobbles fast,
# lower KP. If it feels sloppy and drifts over, raise KP. KD damps the wobble.
ROLL_RATE = (0.0040, 0.0020, 0.00006)    # (KP, KI, KD)
PITCH_RATE = (0.0040, 0.0020, 0.00006)
YAW_RATE = (0.0050, 0.0010, 0.0)

# Angle loop: tilt error in degrees -> requested rotation rate in deg/s.
# 4.0 means 10 degrees of tilt asks for 40 deg/s back towards level. Set both to
# 0.0 to tune the rate loop on its own first.
ROLL_ANGLE_KP = 4.0
PITCH_ANGLE_KP = 4.0

MAX_ANGLE_RATE_DPS = 150.0   # clamp on what the angle loop may request
RATE_I_LIMIT = 50.0          # integral clamp, deg/s * s
I_ENABLE_THROTTLE = 0.55     # integrators held at zero below this - stops them
                             # winding up while the quad still sits on the ground
TRIM_ROLL = 0.0              # constant roll correction, if it always drifts one way
TRIM_PITCH = 0.0

# =============================================================================

FULL = 65535
# Which two motors share each DRV8833, as (on IN1, on IN2/IN3). Numbers follow
# config.MOTOR_PINS and the mixer: 1 front-right, 2 rear-left, 3 front-left,
# 4 rear-right.
PAIRS = ((1, 2), (3, 4))


class Motors:
    """
    Four motors behind two DRV8833s, in either wiring mode.

    SLP goes low before any PWM exists, so there is no moment where a stale
    duty can reach a motor.
    """

    def __init__(self, wiring):
        if wiring not in ("STANDARD", "SHARED"):
            raise ValueError("WIRING must be STANDARD or SHARED")
        self.shared = wiring == "SHARED"

        self.slp = Pin(config.MOTOR_SLEEP_PIN, Pin.OUT)
        self.slp.low()

        self.fault = None
        if config.MOTOR_FAULT_PIN is not None:
            self.fault = Pin(config.MOTOR_FAULT_PIN, Pin.IN, Pin.PULL_UP)

        # Shared bridge: 50% per motor, so the two pulses can never overlap.
        # Separate bridges: the usual two-motors-per-chip thermal cap.
        self.ceiling = 0.5 if self.shared else config.MAX_DUTY

        self.pwm = {}
        for a, b in PAIRS:
            self.pwm[a] = PWM(Pin(config.MOTOR_PINS[a]), freq=config.MOTOR_PWM_FREQ,
                              duty_u16=0)
            if self.shared:
                # GP10/GP11 are PWM slice 5 channels A/B, GP12/GP13 slice 6 A/B,
                # so each pair shares one counter and stays phase-locked.
                # Channel A is high from the start of the period; channel B,
                # inverted, is high at the END of it. duty_u16=FULL inverted =
                # permanently low, i.e. off.
                self.pwm[b] = PWM(Pin(config.MOTOR_PINS[b]), freq=config.MOTOR_PWM_FREQ,
                                  duty_u16=FULL, invert=True)
            else:
                self.pwm[b] = PWM(Pin(config.MOTOR_PINS[b]), freq=config.MOTOR_PWM_FREQ,
                                  duty_u16=0)

    def _duty(self, throttle):
        throttle = 0.0 if throttle < 0.0 else (1.0 if throttle > 1.0 else throttle)
        return int(throttle * self.ceiling * FULL)

    def set_many(self, throttles):
        for a, b in PAIRS:
            self.pwm[a].duty_u16(self._duty(throttles[a]))
            if self.shared:
                self.pwm[b].duty_u16(FULL - self._duty(throttles[b]))
            else:
                self.pwm[b].duty_u16(self._duty(throttles[b]))

    def zero(self):
        self.set_many({1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0})

    def arm(self):
        self.zero()
        self.slp.high()

    def disarm(self):
        self.zero()
        self.slp.low()

    def faulted(self):
        return self.fault is not None and self.fault.value() == 0


# 100 kHz instead of config's 400 kHz: slower edges survive the motor noise on
# the wires much better, and one IMU read still takes well under a millisecond.
I2C_FREQ = 100_000

# At ~300 Hz: 30 failed reads is ~0.1 s of holding the last motor outputs blind
# before giving up; 60 identical reads is ~0.2 s of a sensor that has reset.
MAX_BAD_READS = 30
MAX_FROZEN_READS = 60


def unstick_i2c(verbose=True):
    """
    Free an I2C bus the IMU is holding hostage.

    If the Pico resets mid-read (brownout when the motors spin up, Ctrl-C, a
    soft reboot from Thonny), the MPU6050 does NOT reset - it keeps its own
    power and sits halfway through sending a byte, holding SDA low forever.
    Every later read then times out with ETIMEDOUT. Clocking SCL by hand until
    it lets go of SDA, then sending a STOP, finishes that byte and frees it.
    """
    scl = Pin(config.IMU_SCL_PIN, Pin.OPEN_DRAIN, value=1)
    sda = Pin(config.IMU_SDA_PIN, Pin.IN, Pin.PULL_UP)
    if sda.value():
        return  # bus is free
    if verbose:
        print("I2C bus stuck (SDA held low), clocking it free...")
    for _ in range(18):
        scl.value(0)
        time.sleep_us(10)
        scl.value(1)
        time.sleep_us(10)
        if sda.value():
            break
    # STOP condition: SDA rises while SCL is high.
    sda = Pin(config.IMU_SDA_PIN, Pin.OPEN_DRAIN, value=0)
    time.sleep_us(10)
    scl.value(1)
    time.sleep_us(10)
    sda.value(1)
    time.sleep_us(10)


def throttle_at(t):
    """The flight profile. Returns None once it is over."""
    if t < RAMP_UP_S:
        return LIFT_THROTTLE * t / RAMP_UP_S
    t -= RAMP_UP_S
    # No hold time set: stay at LIFT_THROTTLE forever. Thonny's Stop button
    # sends Ctrl-C, which lands in run()'s KeyboardInterrupt handler and the
    # finally block disarms - motors cut straight to zero, no ramp down.
    if HOLD_S is None or t < HOLD_S:
        return LIFT_THROTTLE
    t -= HOLD_S
    if t < RAMP_DOWN_S:
        return LIFT_THROTTLE * (1.0 - t / RAMP_DOWN_S)
    return None


def run():
    print("\n=== pico-drone lift test ===")
    if HOLD_S is None:
        print("wiring %s, throttle %.2f, profile %.1fs up / hold until Stop (Ctrl-C)"
              % (WIRING, LIFT_THROTTLE, RAMP_UP_S))
    else:
        print("wiring %s, throttle %.2f, profile %.1fs up / %.1fs hold / %.1fs down"
              % (WIRING, LIFT_THROTTLE, RAMP_UP_S, HOLD_S, RAMP_DOWN_S))

    # Motors first, so SLP is low before anything that can fail or block.
    motors = Motors(WIRING)
    heartbeat = Heartbeat().start()

    try:
        unstick_i2c()
        i2c = I2C(config.IMU_I2C_ID, sda=Pin(config.IMU_SDA_PIN),
                  scl=Pin(config.IMU_SCL_PIN), freq=I2C_FREQ)
        try:
            imu = MPU6050(i2c=i2c)
        except OSError:
            print("\nIMU not answering on I2C (found: %s)." % [hex(a) for a in i2c.scan()])
            print("Unplug USB AND battery for 5 s so the IMU fully powers off, then")
            print("check the GY-521 SDA -> GP4 (pin 6) and SCL -> GP5 (pin 7) wires.")
            return
        print("IMU ok")
        print("Calibrating gyro, keep the quad flat and still...")
        imu.calibrate_gyro()

        rate = {
            "roll": PID(*ROLL_RATE, integral_limit=RATE_I_LIMIT),
            "pitch": PID(*PITCH_RATE, integral_limit=RATE_I_LIMIT),
            "yaw": PID(*YAW_RATE, integral_limit=RATE_I_LIMIT),
        }
        angle = {
            "roll": PID(ROLL_ANGLE_KP, 0.0, 0.0, output_limit=MAX_ANGLE_RATE_DPS),
            "pitch": PID(PITCH_ANGLE_KP, 0.0, 0.0, output_limit=MAX_ANGLE_RATE_DPS),
        }
        fusion = ComplementaryFilter(alpha=0.98)
        accel, gyro, _ = imu.read()
        fusion.update(accel, gyro, 0.002)

        answer = input("\nProps on, clear area. Type ARM to fly, DRY for a motors-off "
                       "run, anything else aborts: ").strip().upper()
        if answer == "ARM":
            live = True
        elif answer == "DRY":
            live = False
            print("DRY run - SLP stays low, nothing can spin.")
        else:
            print("Aborted.")
            return

        if live:
            motors.arm()
            print("ARMED")

        start = time.ticks_ms()
        last = time.ticks_us()
        last_print = start
        loops = 0
        reason = "profile complete, landed"
        bad_reads = 0      # consecutive failed IMU reads
        i2c_errors = 0     # total, shown in the printout
        frozen = 0         # consecutive bit-identical IMU readings
        prev = None

        try:
            while True:
                now_ms = time.ticks_ms()
                t = time.ticks_diff(now_ms, start) / 1000.0
                throttle = throttle_at(t)
                if throttle is None:
                    break

                now_us = time.ticks_us()
                dt = time.ticks_diff(now_us, last) / 1_000_000.0
                last = now_us
                loops += 1

                # Motor current spikes corrupt the odd I2C transfer. Ride out a
                # short burst of failures with the motors held where they were,
                # re-freeing the bus each time; only give up if it stays dead.
                try:
                    accel, gyro, _ = imu.read()
                    bad_reads = 0
                except OSError:
                    bad_reads += 1
                    i2c_errors += 1
                    if bad_reads > MAX_BAD_READS:
                        reason = ("IMU stopped answering (%d failed reads in a row) - "
                                  "motor noise or voltage sag, see top of file" % bad_reads)
                        break
                    unstick_i2c(verbose=False)  # count shows in the printout
                    imu.i2c = I2C(config.IMU_I2C_ID, sda=Pin(config.IMU_SDA_PIN),
                                  scl=Pin(config.IMU_SCL_PIN), freq=I2C_FREQ)
                    continue

                # A power dip resets the MPU6050 into sleep mode, where it keeps
                # answering but returns the same frozen numbers forever. A live
                # sensor is never bit-identical this many reads in a row.
                if (accel, gyro) == prev:
                    frozen += 1
                    if frozen > MAX_FROZEN_READS:
                        reason = ("IMU readings frozen - it lost power and reset "
                                  "itself. Power/noise problem, see top of file")
                        break
                else:
                    frozen = 0
                prev = (accel, gyro)

                fusion.update(accel, gyro, dt)
                roll, pitch = fusion.roll_deg, fusion.pitch_deg

                if abs(roll) > TILT_LIMIT_DEG or abs(pitch) > TILT_LIMIT_DEG:
                    reason = "TILT LIMIT (roll %+.0f pitch %+.0f)" % (roll, pitch)
                    break

                # Still on the ground: keep the integrators empty, otherwise they
                # soak up the floor's resistance and lurch the quad on liftoff.
                if throttle < I_ENABLE_THROTTLE:
                    for pid in rate.values():
                        pid._integral = 0.0

                roll_sp = angle["roll"].update(0.0, roll, dt)
                pitch_sp = angle["pitch"].update(0.0, pitch, dt)
                roll_cmd = rate["roll"].update(roll_sp, gyro[0], dt) + TRIM_ROLL
                pitch_cmd = rate["pitch"].update(pitch_sp, gyro[1], dt) + TRIM_PITCH
                yaw_cmd = rate["yaw"].update(0.0, gyro[2], dt)

                out = mix(throttle, roll_cmd, pitch_cmd, yaw_cmd, idle=IDLE)

                if live:
                    motors.set_many(out)
                    if motors.faulted():
                        reason = "DRV8833 nFAULT (over-current or overheat)"
                        break

                if time.ticks_diff(now_ms, last_print) > 200:
                    last_print = now_ms
                    print("t=%4.1f thr %.2f  roll %+5.1f pitch %+5.1f  az %.2fg  "
                          "m1 %.2f m2 %.2f m3 %.2f m4 %.2f  %3.0fHz  i2c err %d"
                          % (t, throttle, roll, pitch, accel[2] / 9.81,
                             out[1], out[2], out[3], out[4], loops / max(t, 0.001),
                             i2c_errors))
        except KeyboardInterrupt:
            reason = "Ctrl-C"

        print("\nstopped: %s" % reason)

    finally:
        motors.disarm()
        heartbeat.stop()
        print("=== disarmed ===")


if __name__ == "__main__":
    run()
