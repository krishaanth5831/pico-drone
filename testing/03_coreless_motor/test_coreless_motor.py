"""
Spins each motor individually so you can confirm mapping and rotation direction.

PROPS MUST BE OFF. Motors must be physically restrained.

Uses MotorBank as a context manager, so the drivers are cut on every exit path
including Ctrl-C and any unexpected exception.
"""

import sys
import time

sys.path.append("/")

import config  # noqa: E402
from drivers.heartbeat import Heartbeat  # noqa: E402
from drivers.motors import MotorBank, ramp  # noqa: E402
from flight.mixer import MOTOR_DIRECTIONS, MOTOR_POSITIONS  # noqa: E402

HOLD_S = 1.5
RAMP_S = 1.0
PEAK = 0.55  # enough to see direction clearly, gentle on the drivers

print("\n=== coreless motor test ===")
print("MAX_DUTY %.2f, props must be OFF" % config.MAX_DUTY)
print("LED pulses for as long as this runs\n")

with Heartbeat(), MotorBank() as bank:
    try:
        for number in sorted(config.MOTOR_PINS):
            print(
                "motor %d (%s, expect %s)"
                % (number, MOTOR_POSITIONS[number], MOTOR_DIRECTIONS[number])
            )

            print("  ramping up...")
            bank.set(number, config.MIN_START)
            ramp(bank, number, config.MIN_START, PEAK, RAMP_S)

            print("  holding...")
            time.sleep(HOLD_S)

            print("  ramping down")
            ramp(bank, number, PEAK, 0.0, RAMP_S)
            bank.set(number, 0.0)

            if bank.faulted():
                print("  !! DRV8833 nFAULT tripped - over-current or thermal")

            time.sleep(1)
            print()

    except KeyboardInterrupt:
        print("\ninterrupted")

print("=== disarmed, LED off ===")
