"""
Pico 2 W bring-up check.

Confirms MicroPython is running, reports what the board thinks it is, and blinks
the onboard LED so there is physical proof that code is executing rather than
just a serial port that opened.

Run from Thonny with the interpreter set to MicroPython (Raspberry Pi Pico).
"""

import gc
import sys
import time

import machine
from machine import Pin

print("\n=== Pico 2 W bring-up ===")

print("MicroPython :", sys.version)
print("board       :", sys.implementation._machine if hasattr(sys.implementation, "_machine") else "unknown")
print("CPU freq    : %d MHz" % (machine.freq() // 1_000_000))

gc.collect()
print("free RAM    : %d bytes" % gc.mem_free())

# 1 = power-on, 2 = watchdog, 3 = hard reset. Repeated power-on resets during a
# motor test is the signature of a brownout - see testing/07_lipo_power.
print("reset cause :", machine.reset_cause())

# "LED" is the portable name: GP25 on a plain Pico, behind the CYW43 chip on a W.
led = Pin("LED", Pin.OUT)
print("LED         : blinking - watch the board")

try:
    for _ in range(3):        # three slow
        led.on()
        time.sleep_ms(400)
        led.off()
        time.sleep_ms(400)
    for _ in range(5):        # five fast
        led.on()
        time.sleep_ms(80)
        led.off()
        time.sleep_ms(80)
finally:
    led.off()

# The CYW43 driver only imports on "W" boards. Its absence is not an error here,
# but it does tell you which board is actually on the desk.
try:
    import network

    network.WLAN(network.STA_IF)
    print("WiFi chip   : present (CYW43 responded)")
except Exception as exc:  # noqa: BLE001
    print("WiFi chip   : NOT detected (%s) - plain Pico 2?" % exc)

print("=== all good ===")
