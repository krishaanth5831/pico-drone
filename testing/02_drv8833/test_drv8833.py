"""
DRV8833 driver-board check, with no motors attached.

Steps SLP and the PWM inputs through known states and pauses so you can measure
the outputs with a multimeter. Nothing here can spin a motor - there is nothing
connected to spin.

Copy src/config.py to the Pico alongside this script, or run it from Thonny with
config.py already on the board.
"""

import sys
import time

from machine import PWM, Pin

sys.path.append("/")
import config  # noqa: E402

SETTLE_S = 6  # long enough to get a probe onto a pad

slp = Pin(config.MOTOR_SLEEP_PIN, Pin.OUT)
slp.low()  # asleep before anything else happens

fault = None
if config.MOTOR_FAULT_PIN is not None:
    fault = Pin(config.MOTOR_FAULT_PIN, Pin.IN, Pin.PULL_UP)

channels = {}
for number in (1, 2):
    pwm = PWM(Pin(config.MOTOR_PINS[number]))
    pwm.freq(config.MOTOR_PWM_FREQ)
    pwm.duty_u16(0)
    channels[number] = pwm

OUTPUT_NAMES = {1: "AOUT1", 2: "BOUT1"}

print("\n=== DRV8833 driver check ===")
print("no motors should be connected\n")

try:
    print("SLP low  -> drivers asleep")
    for name in OUTPUT_NAMES.values():
        print("  measure %s now: expect ~0 V (high impedance)" % name)
        time.sleep(SETTLE_S)

    print("\nSLP high -> drivers awake")
    slp.high()
    time.sleep_ms(10)

    for number, name in OUTPUT_NAMES.items():
        print("  %s at 50%% duty" % ("AIN1" if number == 1 else "BIN1"))
        channels[number].duty_u16(32768)
        print("  measure %s now: expect roughly 2.0-2.7 V" % name)
        time.sleep(SETTLE_S)

        print("  back to 0%%")
        channels[number].duty_u16(0)
        print("  measure %s now: expect ~0 V" % name)
        time.sleep(SETTLE_S)
        print()

    if fault is not None:
        state = "TRIPPED (over-current or thermal)" if fault.value() == 0 else "OK (high)"
        print("nFAULT  :", state)
    else:
        print("nFAULT  : not wired")

finally:
    # Every exit path cuts the drivers, including Ctrl-C in Thonny.
    for pwm in channels.values():
        pwm.duty_u16(0)
    slp.low()
    print("=== disarmed ===")
