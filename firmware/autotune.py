"""
Autotune flight: hover under the balloons and let the ground station (gcs/gcs.py
on a laptop, over the Pico 2 W's WiFi) run the PX4-style autotune.

UNTESTED ON HARDWARE. It is the same session / link code the simulator runs
(sim/sitl.py --check), but nothing here has been on a board yet. Tether it,
keep the tether slack, keep a hand on the battery lead.

Open this in Thonny and press Run. Everything to edit is in the block below.

  1. WiFi comes up first. AP mode (default): the Pico makes its own network,
     AP_SSID below; join it from the laptop. The console prints the exact
     ground-station command, e.g.  /usr/bin/python3 gcs/gcs.py --vehicle 192.168.4.1
  2. At the prompt:
       ARM  -> motors spin, ramp to HOVER_THROTTLE, hover until DISARM / Stop
       DRY  -> whole loop + WiFi runs, motors stay asleep (SLP low). Tilt the
               board by hand: right side down must read roll +, nose down
               pitch +, in the ground station. Otherwise STOP (IMU mounting).
       else -> abort
  3. Hovering, in the ground station: Autotune -> Start. ~25 s of wiggles,
     then a result: Accept (now, or on disarm) or Reject.
  4. Stop with the ground station's DISARM or Thonny's Stop (Ctrl-C). After
     the motors are off, accepted gains are written to /gains.json; lift.py
     uses them from then on (USE_SAVED_GAINS in lift.py).

There is no arm command over WiFi: arming is the prompt above, at the board.
The ground station's DISARM is software over WiFi, not a kill switch.

No altitude sensor: the throttle is open loop, as in lift.py, so the balloons
(and the tether) set the height. They also make the autotune possible at
all: it adds and takes away thrust around the hover throttle, and at full
throttle there is nothing left to add. Above MAX_TUNE_THROTTLE it refuses to
start - add balloon relief until it hovers lower.

Safety, as in lift.py: auto-disarm past TILT_LIMIT_DEG, on nFAULT, on a dead
IMU, on Ctrl-C, on any error. The autotune itself aborts (old gains keep
flying) past 25 degrees, if the ground station goes quiet for 1.5 s, or on
the ground station's Abort.
"""

import gc
import sys
import time

sys.path.append("/")

from machine import I2C, PWM, Pin  # noqa: E402

try:
    import config  # noqa: E402
    from drivers.gcs_link import VEHICLE_PORT, Link  # noqa: E402
    from drivers.heartbeat import Heartbeat  # noqa: E402
    from drivers.mpu6050 import MPU6050  # noqa: E402
    from flight import gains_file  # noqa: E402
    from flight.autotune_session import AutotuneSession  # noqa: E402
    from flight.fusion import ComplementaryFilter  # noqa: E402
    from flight.ground_station import GroundStation  # noqa: E402
    from flight.mixer import mix  # noqa: E402
    from flight.pid import PID  # noqa: E402
except ImportError as exc:
    print("\n%s\n\nLibrary not on the Pico. Run ./tools/upload.sh, or in Thonny\n"
          "View -> Files, select config.py, drivers, flight -> 'Upload to /'.\n" % exc)
    raise SystemExit()


# =============================================================================
# EDIT HERE
# =============================================================================

# Motor wiring - same meaning as in lift.py.
WIRING = "STANDARD"

# --- WiFi ---------------------------------------------------------------------
# "AP":  the Pico makes its own network (no router needed). Join AP_SSID from
#        the laptop; the Pico is 192.168.4.1.
# "STA": the Pico joins an existing network; the console prints its address.
WIFI_MODE = "AP"
AP_SSID = "pico-drone"
AP_PASSWORD = "pico-drone-tune"     # at least 8 characters
STA_SSID = ""
STA_PASSWORD = ""

# --- throttle -------------------------------------------------------------------
HOVER_THROTTLE = 0.70  # open loop: what hovers it with the balloons on
RAMP_UP_S = 2.5        # slow ramp: a step to full current browns out a 1S pack
IDLE = 0.40            # mixer floor while throttle > 0 (see lift.py)
SPINUP_S = 0.4         # motors start one at a time, each over this long
MAX_TUNE_THROTTLE = 0.85   # above this there's no headroom for the wiggles

# --- loop -----------------------------------------------------------------------
# Fixed rate, not as-fast-as-possible like lift.py: the autotune's reference
# drone was measured at 100 Hz, and its model needs an even dt.
LOOP_HZ = 100
PRINT_S = 1.0          # console line every this many seconds (USB printing is slow)

# --- level / safety -------------------------------------------------------------
LEVEL_AT_START = True  # "level" = how it sits during calibration (see lift.py)
TILT_LIMIT_DEG = 35.0  # auto-disarm past this roll or pitch

# --- default gains (= lift.py's) --------------------------------------------------
ROLL_RATE = (0.0040, 0.0020, 0.00006)    # (KP, KI, KD)
PITCH_RATE = (0.0040, 0.0020, 0.00006)
YAW_RATE = (0.0050, 0.0010, 0.0)
ROLL_ANGLE_KP = 4.0
PITCH_ANGLE_KP = 4.0
MAX_ANGLE_RATE_DPS = 150.0
RATE_I_LIMIT = 50.0
I_ENABLE_THROTTLE = 0.55
TRIM_ROLL = 0.0
TRIM_PITCH = 0.0
# True = start from the last accepted autotune (/gains.json) instead of the
# numbers above. The console says which are flying.
USE_SAVED_GAINS = True
GAINS_PATH = "/gains.json"

# =============================================================================

FULL = 65535
PAIRS = ((1, 2), (3, 4))
I2C_FREQ = 100_000
MAX_BAD_READS = 15      # ~0.15 s at 100 Hz of holding the motors blind
MAX_FROZEN_READS = 30


class Motors:
    """Four motors behind two DRV8833s (copied from lift.py: Thonny runs this file alone)."""

    def __init__(self, wiring):
        if wiring not in ("STANDARD", "SHARED"):
            raise ValueError("WIRING must be STANDARD or SHARED")
        self.shared = wiring == "SHARED"
        # SLP low before any PWM exists.
        self.slp = Pin(config.MOTOR_SLEEP_PIN, Pin.OUT)
        self.slp.low()
        self.fault = None
        if config.MOTOR_FAULT_PIN is not None:
            self.fault = Pin(config.MOTOR_FAULT_PIN, Pin.IN, Pin.PULL_UP)
        self.ceiling = 0.5 if self.shared else config.MAX_DUTY
        self.pwm = {}
        for a, b in PAIRS:
            self.pwm[a] = PWM(Pin(config.MOTOR_PINS[a]), freq=config.MOTOR_PWM_FREQ, duty_u16=0)
            if self.shared:
                self.pwm[b] = PWM(Pin(config.MOTOR_PINS[b]), freq=config.MOTOR_PWM_FREQ,
                                  duty_u16=FULL, invert=True)
            else:
                self.pwm[b] = PWM(Pin(config.MOTOR_PINS[b]), freq=config.MOTOR_PWM_FREQ, duty_u16=0)

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


def unstick_i2c(verbose=True):
    """Free an I2C bus the IMU is holding (see lift.py for the story)."""
    scl = Pin(config.IMU_SCL_PIN, Pin.OPEN_DRAIN, value=1)
    sda = Pin(config.IMU_SDA_PIN, Pin.IN, Pin.PULL_UP)
    if sda.value():
        return
    if verbose:
        print("I2C bus stuck (SDA held low), clocking it free...")
    for _ in range(18):
        scl.value(0)
        time.sleep_us(10)
        scl.value(1)
        time.sleep_us(10)
        if sda.value():
            break
    sda = Pin(config.IMU_SDA_PIN, Pin.OPEN_DRAIN, value=0)
    time.sleep_us(10)
    scl.value(1)
    time.sleep_us(10)
    sda.value(1)
    time.sleep_us(10)


def start_wifi():
    """WiFi up before the ARM prompt - never inside the control loop. Returns the IP."""
    import network

    if WIFI_MODE == "AP":
        wlan = network.WLAN(network.AP_IF)
        try:
            wlan.config(ssid=AP_SSID, key=AP_PASSWORD)
        except (TypeError, ValueError, OSError):      # older firmware spells them differently
            wlan.config(essid=AP_SSID, password=AP_PASSWORD)
        wlan.active(True)
        for _ in range(50):
            if wlan.active():
                break
            time.sleep_ms(100)
    else:
        wlan = network.WLAN(network.STA_IF)
        wlan.active(True)
        wlan.connect(STA_SSID, STA_PASSWORD)
        print("joining %s..." % STA_SSID)
        for _ in range(150):
            if wlan.isconnected():
                break
            time.sleep_ms(100)
        if not wlan.isconnected():
            raise OSError("could not join %r - check STA_SSID / STA_PASSWORD" % STA_SSID)
    # No power saving: the radio otherwise naps between beacons, which turns
    # into 100 ms gaps in the link.
    try:  # noqa: SIM105 - no contextlib on MicroPython
        wlan.config(pm=getattr(network.WLAN, "PM_NONE", 0xA11140))
    except (TypeError, ValueError, OSError):
        pass
    return wlan.ifconfig()[0]


def run():
    print("\n=== pico-drone autotune flight ===")
    # Motors first, so SLP is low before anything that can fail or block.
    motors = Motors(WIRING)
    heartbeat = Heartbeat("solid").start()
    link = None
    to_save = None
    session = None

    try:
        base = {"roll": ROLL_RATE, "pitch": PITCH_RATE, "yaw": YAW_RATE}
        if USE_SAVED_GAINS:
            gains, trim, source = gains_file.load(GAINS_PATH, base, (TRIM_ROLL, TRIM_PITCH))
        else:
            gains, trim, source = dict(base), (TRIM_ROLL, TRIM_PITCH), "EDIT HERE block (USE_SAVED_GAINS off)"
        print("rate gains: %s" % source)
        for name in ("roll", "pitch", "yaw"):
            print("  %-5s P %.4f  I %.4f  D %.5f" % ((name,) + tuple(gains[name])))
        print("  trim  roll %+.3f  pitch %+.3f" % trim)
        # The autotune always scales lift.py's defaults, so runs never compound.
        session = AutotuneSession(base, gains, trim)

        ip = start_wifi()
        link = Link(port=VEHICLE_PORT)
        gs = GroundStation(link, session, vehicle="pico")
        if WIFI_MODE == "AP":
            print("WiFi: join network %r (password %r) from the laptop" % (AP_SSID, AP_PASSWORD))
        print("ground station:  /usr/bin/python3 gcs/gcs.py --vehicle %s" % ip)

        unstick_i2c()
        i2c = I2C(config.IMU_I2C_ID, sda=Pin(config.IMU_SDA_PIN), scl=Pin(config.IMU_SCL_PIN), freq=I2C_FREQ)
        try:
            imu = MPU6050(i2c=i2c)
        except OSError:
            print("\nIMU not answering on I2C (found: %s). Power-cycle USB AND battery." %
                  [hex(a) for a in i2c.scan()])
            return
        print("IMU ok. Calibrating gyro, keep the quad flat and still...")
        imu.calibrate_gyro()

        rate = {}
        for name in ("roll", "pitch", "yaw"):
            rate[name] = PID(*session.gains[name], integral_limit=RATE_I_LIMIT)
        angle = {
            "roll": PID(ROLL_ANGLE_KP, 0.0, 0.0, output_limit=MAX_ANGLE_RATE_DPS),
            "pitch": PID(PITCH_ANGLE_KP, 0.0, 0.0, output_limit=MAX_ANGLE_RATE_DPS),
        }
        gains_version = session.version
        fusion = ComplementaryFilter(alpha=0.98)
        accel, gyro, _ = imu.read()
        fusion.update(accel, gyro, 0.002)
        level_roll = level_pitch = 0.0
        if LEVEL_AT_START:
            level_roll, level_pitch = fusion.roll_deg, fusion.pitch_deg
            print("level = roll %+.1f pitch %+.1f (as it sits now)" % (level_roll, level_pitch))

        answer = input("\nProps on, balloons on, tethered, clear area. Type ARM to fly, DRY for a "
                       "motors-off run, anything else aborts: ").strip().upper()
        if answer == "ARM":
            live = True
        elif answer == "DRY":
            live = False
            print("DRY run - SLP stays low, nothing can spin. Tilt the board: right side "
                  "down must read roll +, nose down pitch +.")
        else:
            print("Aborted.")
            return

        if live:
            motors.arm()
            print("ARMED")
            spin = {1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0}
            steps = max(1, int(SPINUP_S / 0.02))
            for m in (1, 2, 3, 4):
                for i in range(1, steps + 1):
                    spin[m] = IDLE * i / steps
                    motors.set_many(spin)
                    time.sleep_ms(20)

        period_us = 1_000_000 // LOOP_HZ
        start = time.ticks_ms()
        last = time.ticks_us()
        next_us = time.ticks_add(last, period_us)
        last_print = start
        loops = loops_at_print = 0
        worst_ms = worst_shown = 0.0
        bad_reads = frozen = i2c_errors = 0
        prev = None
        hz = float(LOOP_HZ)
        out = {1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0}
        reason = "?"

        try:
            while True:
                # --- fixed rate: wait for the next slot; if badly late, don't race to catch up.
                wait = time.ticks_diff(next_us, time.ticks_us())
                if wait > 0:
                    time.sleep_us(wait)
                elif wait < -period_us:
                    next_us = time.ticks_us()
                next_us = time.ticks_add(next_us, period_us)

                now_us = time.ticks_us()
                dt = time.ticks_diff(now_us, last) / 1_000_000.0
                last = now_us
                if dt * 1000.0 > worst_ms:
                    worst_ms = dt * 1000.0
                now_ms = time.ticks_ms()
                t = time.ticks_diff(now_ms, start) / 1000.0
                loops += 1

                throttle = HOVER_THROTTLE * min(1.0, t / RAMP_UP_S) if live else 0.0

                try:
                    accel, gyro, _ = imu.read()
                    bad_reads = 0
                except OSError:
                    bad_reads += 1
                    i2c_errors += 1
                    if bad_reads > MAX_BAD_READS:
                        reason = "IMU stopped answering (%d failed reads) - power/grounding, see lift.py" % bad_reads
                        break
                    unstick_i2c(verbose=False)
                    imu.i2c = I2C(config.IMU_I2C_ID, sda=Pin(config.IMU_SDA_PIN),
                                  scl=Pin(config.IMU_SCL_PIN), freq=I2C_FREQ)
                    continue
                if (accel, gyro) == prev:
                    frozen += 1
                    if frozen > MAX_FROZEN_READS:
                        reason = "IMU readings frozen - it lost power and reset itself"
                        break
                else:
                    frozen = 0
                prev = (accel, gyro)

                fusion.update(accel, gyro, dt)
                roll, pitch = fusion.roll_deg, fusion.pitch_deg
                if abs(roll) > TILT_LIMIT_DEG or abs(pitch) > TILT_LIMIT_DEG:
                    reason = "TILT LIMIT (roll %+.0f pitch %+.0f)" % (roll, pitch)
                    break

                # The autotune sees the gyro that answers last loop's command.
                session.observe(gyro, dt)

                # No altitude sensor: "airborne" = armed and done ramping up.
                airborne = live and t >= RAMP_UP_S and throttle >= I_ENABLE_THROTTLE
                cant = None
                if not live:
                    cant = "DRY run - motors are off. Run again and type ARM to tune."
                elif t < RAMP_UP_S:
                    cant = "still ramping up to the hover throttle"
                elif HOVER_THROTTLE > MAX_TUNE_THROTTLE:
                    cant = ("hover throttle %.2f leaves no headroom for the wiggles (max %.2f) - "
                            "add balloon relief so it hovers lower" % (HOVER_THROTTLE, MAX_TUNE_THROTTLE))
                for _sev, msg in session.texts:          # the console sees them too
                    print(msg)
                actions = gs.service(t, {
                    "armed": live, "airborne": airborne,
                    "att": [roll - level_roll, pitch - level_pitch, 0.0],
                    "rates": [gyro[0], gyro[1], gyro[2]], "thr": throttle,
                    "out": [out[1], out[2], out[3], out[4]], "hz": hz, "dtmax": worst_shown,
                    "cant_tune": cant,
                })
                if "disarm" in actions:
                    reason = "DISARM from the ground station"
                    break

                exc = session.update(t, airborne, (roll - level_roll, pitch - level_pitch), gyro,
                                     False, gs.link_ok(t))
                if session.version != gains_version:
                    # New gains (accepted now, or reverted after a wobble):
                    # change them in place, so the integrators keep the trim.
                    for name in ("roll", "pitch", "yaw"):
                        rate[name].kp, rate[name].ki, rate[name].kd = session.gains[name]
                    gains_version = session.version
                    print("rate gains changed: roll P %.4f  pitch P %.4f  yaw P %.4f" %
                          (session.gains["roll"][0], session.gains["pitch"][0], session.gains["yaw"][0]))

                if throttle < I_ENABLE_THROTTLE:
                    for pid in rate.values():
                        pid._integral = 0.0
                roll_sp = angle["roll"].update(level_roll, roll, dt)
                pitch_sp = angle["pitch"].update(level_pitch, pitch, dt)
                roll_cmd = rate["roll"].update(roll_sp, gyro[0], dt) + exc[0] + session.trim[0]
                pitch_cmd = rate["pitch"].update(pitch_sp, gyro[1], dt) + exc[1] + session.trim[1]
                yaw_cmd = rate["yaw"].update(0.0, gyro[2], dt) + exc[2]
                out = mix(throttle, roll_cmd, pitch_cmd, yaw_cmd, idle=IDLE) if live else \
                    {1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0}
                if live:
                    motors.set_many(out)
                    if motors.faulted():
                        reason = "DRV8833 nFAULT (over-current or overheat)"
                        break
                session.record(out)

                # Accepted gains: remembered now, written to flash after disarm
                # (a flash write stalls the CPU - not while flying).
                save = session.take_save()
                if save is not None:
                    to_save = save

                if time.ticks_diff(now_ms, last_print) >= PRINT_S * 1000:
                    hz = (loops - loops_at_print) * 1000.0 / time.ticks_diff(now_ms, last_print)
                    worst_shown = worst_ms
                    print("t=%5.1f thr %.2f  roll %+5.1f pitch %+5.1f  %3.0f Hz (worst %4.1f ms)  "
                          "link %s  autotune %s %3.0f%%  i2c err %d"
                          % (t, throttle, roll - level_roll, pitch - level_pitch, hz, worst_ms,
                             "ok" if gs.link_ok(t) else "--", session.state, 100 * session.progress(),
                             i2c_errors))
                    last_print = now_ms
                    loops_at_print = loops
                    worst_ms = 0.0
                # A collection every loop stays small; one big one mid-autotune
                # would be a 10+ ms hole in the data. "worst" shows the cost.
                gc.collect()
        except KeyboardInterrupt:
            reason = "Ctrl-C"
        print("\nstopped: %s" % reason)

    finally:
        motors.disarm()
        heartbeat.stop()
        print("=== disarmed ===")
        # PX4's "apply on disarm": a result accepted for landing goes in now.
        if session is not None:
            session.landed()
            for _sev, msg in session.texts:
                print(msg)
            save = session.take_save()
            if save is not None:
                to_save = save
        if to_save is not None:
            gains_file.save(GAINS_PATH, to_save[0], to_save[1])
            print("saved %s - lift.py and this script fly them from now on:" % GAINS_PATH)
            for name in ("roll", "pitch", "yaw"):
                print("  %-5s P %.4f  I %.4f  D %.5f" % ((name,) + tuple(to_save[0][name])))
            print("  trim  roll %+.3f  pitch %+.3f" % tuple(to_save[1]))
        if link is not None:
            link.close()


if __name__ == "__main__":
    run()
