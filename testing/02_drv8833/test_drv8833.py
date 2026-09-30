"""
DRV8833 driver-board check, with no motors attached.

Steps SLP and the PWM inputs through known states and pauses so you can measure
the outputs with a multimeter. Nothing here can spin a motor - there is nothing
connected to spin.

Covers BOTH driver boards: all four channels, one at a time, so a dead board or
a swapped input shows up here rather than at 03_coreless_motor with props on.

Copy src/config.py to the Pico alongside this script, or run it from Thonny with
config.py already on the board.
"""

import sys
import time

from machine import PWM, Pin

sys.path.append("/")
try:
    import config  # noqa: E402
    from drivers.heartbeat import Heartbeat  # noqa: E402
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

SETTLE_S = 6  # long enough to get a probe onto a pad

# Channel map. Which pad you probe depends on which board and which half of it,
# and that does NOT follow from the channel number alone - hence a table rather
# than arithmetic. Matches the wiring table in this folder's README and the
# MOTOR_PINS comments in src/config.py.
#
#   channel -> (board, input pad driven, output pad you measure)
CHANNELS = {
    1: ("DRV #1", "AIN1", "AOUT1"),
    2: ("DRV #1", "BIN1", "BOUT1"),
    3: ("DRV #2", "AIN1", "AOUT1"),
    4: ("DRV #2", "BIN1", "BOUT1"),
}

# Half throttle is what the README's expected voltages are written against, but
# never let this exceed the thermal cap from config.py. Deriving it here means
# lowering MAX_DUTY lowers this too, instead of leaving a stale literal behind.
TEST_DUTY = min(0.50, config.MAX_DUTY)

slp = Pin(config.MOTOR_SLEEP_PIN, Pin.OUT)
slp.low()  # asleep before anything else happens

fault = None
if config.MOTOR_FAULT_PIN is not None:
    fault = Pin(config.MOTOR_FAULT_PIN, Pin.IN, Pin.PULL_UP)

# Set if the walk is cut short by a fault, so the footer can say so rather than
# printing the same "disarmed" line a clean run ends with.
tripped_on = None

channels = {}
for number in CHANNELS:
    pwm = PWM(Pin(config.MOTOR_PINS[number]))
    pwm.freq(config.MOTOR_PWM_FREQ)
    pwm.duty_u16(0)
    channels[number] = pwm

# One asleep step, then two measurement pauses per channel.
total_s = SETTLE_S * (len(CHANNELS) * 2 + 1)

print("\n=== DRV8833 driver check ===")
print("no motors should be connected")
if total_s < 60:
    duration = "about %d s" % total_s
else:
    duration = "about %d min %02d s" % (total_s // 60, total_s % 60)
print("%d channels across 2 boards, %s total" % (len(CHANNELS), duration))
print("LED pulses for as long as this runs\n")

# Soft-timer based, so it keeps beating through the six-second measurement
# pauses below.
heartbeat = Heartbeat().start()

try:
    print("SLP low  -> both drivers asleep")
    for board, _, pad_out in CHANNELS.values():
        print("  measure %s %s now: expect ~0 V (high impedance)" % (board, pad_out))
        time.sleep(SETTLE_S)

    print("\nSLP high -> both drivers awake")
    slp.high()
    time.sleep_ms(10)

    for number, (board, pad_in, pad_out) in CHANNELS.items():
        print("\n-- channel %d: %s %s -> %s (GP%d) --"
              % (number, board, pad_in, pad_out, config.MOTOR_PINS[number]))

        print("  %s at %d%% duty" % (pad_in, int(TEST_DUTY * 100)))
        channels[number].duty_u16(int(65535 * TEST_DUTY))
        print("  measure %s %s now: expect roughly 2.0-2.7 V" % (board, pad_out))
        time.sleep(SETTLE_S)

        print("  back to 0%")
        channels[number].duty_u16(0)
        print("  measure %s %s now: expect ~0 V" % (board, pad_out))
        time.sleep(SETTLE_S)

        # Checked per channel, not just at the end: a short on one output trips
        # nFAULT immediately, and both boards share the line, so knowing WHICH
        # channel was live when it went low is what localises the fault.
        if fault is not None and fault.value() == 0:
            print("  !! nFAULT tripped while driving this channel")
            tripped_on = number
            break

    print()
    if fault is not None:
        state = "TRIPPED (over-current or thermal)" if fault.value() == 0 else "OK (high)"
        print("nFAULT  :", state)
    else:
        print("nFAULT  : not wired")

finally:
    # Every exit path cuts both drivers, including Ctrl-C in Thonny.
    for pwm in channels.values():
        pwm.duty_u16(0)
    slp.low()
    heartbeat.stop()
    if tripped_on is not None:
        board, _, pad_out = CHANNELS[tripped_on]
        print("=== STOPPED EARLY: nFAULT on channel %d (%s %s) ==="
              % (tripped_on, board, pad_out))
        print("=== disarmed, LED off - later channels untested ===")
    else:
        print("=== disarmed, LED off ===")
