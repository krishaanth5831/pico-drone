"""
1S LiPo power-path test: loads all four motors together and watches for the Pico
resetting.

PROPS OFF. The airframe must be restrained - this runs every motor at once.

A brownout shows up two ways: the boot counter increments (persisted to a file,
because a RAM variable would be wiped by the very event we are detecting), and
the LED heartbeat visibly restarts. The LED is the faster signal - you are
watching the multimeter, not the shell, when it happens.
"""

import sys
import time

sys.path.append("/")

import machine  # noqa: E402

import config  # noqa: E402
from drivers.heartbeat import Heartbeat  # noqa: E402
from drivers.motors import MotorBank  # noqa: E402

COUNTER_FILE = "boot_count.txt"
LOAD_THROTTLE = config.MAX_DUTY
RAMP_S = 3.0
HOLD_S = 5


def bump_boot_count():
    """Persist across resets - a RAM variable would be wiped by the very event
    we are trying to detect."""
    try:
        with open(COUNTER_FILE) as handle:
            count = int(handle.read().strip()) + 1
    except (OSError, ValueError):
        count = 1
    with open(COUNTER_FILE, "w") as handle:
        handle.write(str(count))
    return count


print("\n=== 1S LiPo power test ===")
boot = bump_boot_count()
print("boot #%d  (reset cause %s)" % (boot, machine.reset_cause()))

if boot > 1:
    print("\n!! BROWNOUT DETECTED - the board reset during the last run.")
    print("   Add or reposition the 470uF caps at each driver's VM/GND pins.")
    print("   Delete %s to reset the counter." % COUNTER_FILE)

print("\nLED pulses for as long as this runs.")
print("If the pulse stops and restarts, that IS the brownout - watch the board.")

survived = False

with Heartbeat():
    print("\nbattery: measure across the pack now")
    print("  idle, motors off   -> expect 3.7-4.2 V")
    time.sleep(5)

    print("ramping all four motors to %.2f over %.0fs" % (LOAD_THROTTLE, RAMP_S))
    print("  measure again under load -> expect >3.3 V\n")

    try:
        with MotorBank() as bank:
            steps = int(RAMP_S * 50)
            for i in range(steps + 1):
                bank.set_all(LOAD_THROTTLE * i / steps)
                time.sleep_ms(20)

            print("  motors at %.2f for %ds..." % (LOAD_THROTTLE, HOLD_S))
            for second in range(1, HOLD_S + 1):
                time.sleep(1)
                print("  still alive at %ds" % second)
                if bank.faulted():
                    print("  !! DRV8833 nFAULT tripped - thermal or over-current")

            for i in range(steps, -1, -1):
                bank.set_all(LOAD_THROTTLE * i / steps)
                time.sleep_ms(20)

        survived = True

    except KeyboardInterrupt:
        print("\ninterrupted")

if survived:
    print("\n=== PASS: no reset during load ===")
    print("boot count stayed at %d" % boot)
    print("the LED pulsed continuously throughout - no reset")
    print("delete %s before the next clean run" % COUNTER_FILE)
