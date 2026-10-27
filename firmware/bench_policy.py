"""
Policy speed test (RL milestone M0): how big a neural network can the Pico run
inside the control loop?

Props off is fine - this never arms the motors (SLP is pulled low first thing
and stays low). Upload the library first (./tools/upload.sh, or Thonny
View -> Files -> select config.py, drivers, flight -> Upload to /), then open
this file in Thonny and press Run.

What you see: one line per network size with the microseconds one policy call
takes, then - if the GY-521 answers - the time of the rest of the loop (IMU
read + complementary filter + mixer) and the loop rate you'd get with each
network added. Send me the printout; the largest network that keeps the loop
at or above TARGET_HZ is what the simulator trains.
"""

import sys
import time

sys.path.append("/")

from machine import I2C, Pin  # noqa: E402

try:
    import config  # noqa: E402
    from drivers.mpu6050 import MPU6050  # noqa: E402
    from flight.fusion import ComplementaryFilter  # noqa: E402
    from flight.mixer import mix  # noqa: E402
    from flight.policy import OBS_SIZE, Policy, build_obs  # noqa: E402
except ImportError as exc:
    print("\n%s\n\nLibrary not on the Pico. Run ./tools/upload.sh, or in Thonny\n"
          "View -> Files, select config.py, drivers, flight -> 'Upload to /'.\n" % exc)
    raise SystemExit()

# =============================================================================
# EDIT HERE
# =============================================================================
HIDDEN_SIZES = (12, 16, 24, 32)   # two hidden layers of each size are tried
CALLS = 300                       # policy calls timed per size
TARGET_HZ = 100                   # loop rate we want to keep
# =============================================================================


def fake_network(hidden):
    """Random-ish weights of the right shape. Values don't matter for timing."""
    seed = [12345]

    def rnd():
        # Tiny LCG so this needs no `random` module.
        seed[0] = (seed[0] * 1103515245 + 12345) & 0x7FFFFFFF
        return (seed[0] / 0x7FFFFFFF - 0.5) * 0.4

    sizes = (OBS_SIZE, hidden, hidden, 3)
    layers = []
    for k in range(len(sizes) - 1):
        n_in, n_out = sizes[k], sizes[k + 1]
        w = [[rnd() for _ in range(n_in)] for _ in range(n_out)]
        b = [rnd() for _ in range(n_out)]
        layers.append((w, b))
    return Policy(layers)


def time_policy(policy):
    obs = build_obs(3.0, -2.0, (10.0, -5.0, 1.0), 0.5, -0.2, 0.6, (0.0, 0.0, 0.0))
    policy.act(obs)  # warm-up (first call allocates)
    t0 = time.ticks_us()
    for _ in range(CALLS):
        policy.act(obs)
    return time.ticks_diff(time.ticks_us(), t0) / CALLS


def time_rest_of_loop():
    """IMU read + fusion + mixer, i.e. the loop without the policy. None if no IMU."""
    try:
        i2c = I2C(config.IMU_I2C_ID, sda=Pin(config.IMU_SDA_PIN),
                  scl=Pin(config.IMU_SCL_PIN), freq=100_000)
        imu = MPU6050(i2c=i2c)
    except OSError:
        return None
    fusion = ComplementaryFilter(alpha=0.98)
    n = 300
    t0 = time.ticks_us()
    for _ in range(n):
        accel, gyro, _ = imu.read()
        fusion.update(accel, gyro, 0.01)
        mix(0.5, 0.01, -0.01, 0.0, idle=0.4)
    return time.ticks_diff(time.ticks_us(), t0) / n


def run():
    # Nothing here drives a motor, but keep the drivers asleep anyway: a stale
    # PWM from an earlier crashed run must not reach a motor while we benchmark.
    slp = Pin(config.MOTOR_SLEEP_PIN, Pin.OUT)
    slp.low()
    try:
        print("\n=== policy speed test (motors asleep) ===")
        results = []
        for h in HIDDEN_SIZES:
            p = fake_network(h)
            us = time_policy(p)
            results.append((h, us))
            print("2 x %2d hidden  %5d multiply-adds  %6.0f us per call"
                  % (h, p.macs, us))

        rest = time_rest_of_loop()
        if rest is None:
            print("\nGY-521 not answering - only the policy cost is shown above.")
            print("Plug the IMU in (SDA GP4 pin 6, SCL GP5 pin 7) for the full picture.")
            return
        print("\nrest of loop (IMU + filter + mixer): %.0f us -> %.0f Hz on its own"
              % (rest, 1e6 / rest))
        best = None
        for h, us in results:
            hz = 1e6 / (rest + us)
            ok = hz >= TARGET_HZ
            if ok:
                best = h
            print("  with 2 x %2d policy: %4.0f Hz  %s" % (h, hz, "ok" if ok else "too slow"))
        if best is None:
            print("\nNo size reaches %d Hz - tell me, the sim will train at a lower rate."
                  % TARGET_HZ)
        else:
            print("\nLargest network that keeps %d Hz: 2 x %d" % (TARGET_HZ, best))
        print("(GPS parsing in lift.py costs extra on top - measured separately.)")
    finally:
        slp.low()
        print("=== done ===")


if __name__ == "__main__":
    run()
