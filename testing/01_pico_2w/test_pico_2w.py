"""
Pico 2 W bring-up check.

Confirms MicroPython is running, reports what the board thinks it is, blinks an
identification pattern, then leaves the LED pulsing as a liveness indicator.

DELIBERATELY SELF-CONTAINED: this is the first test you run, before anything has
been uploaded to the board, so it imports nothing from src/. The heartbeat below
is a hand-rolled copy of drivers/heartbeat.py for that reason - every other test
imports the real one.

Run from Thonny with the interpreter set to MicroPython (Raspberry Pi Pico).
"""

import gc
import sys
import time

import machine
from machine import Pin, Timer

led = Pin("LED", Pin.OUT)

# --- inline heartbeat --------------------------------------------------------
# Soft timer, so it keeps pulsing through blocking calls. Long on / short off
# reads as a glow at a glance but still visibly beats, which is what tells you
# the board has not silently reset.
_TICK_MS = 25
_ON_MS = 900
_OFF_MS = 100
_state = {"elapsed": 0, "lit": False}


def _beat(_timer):
    _state["elapsed"] = (_state["elapsed"] + _TICK_MS) % (_ON_MS + _OFF_MS)
    lit = _state["elapsed"] < _ON_MS
    if lit != _state["lit"]:
        led.value(1 if lit else 0)
        _state["lit"] = lit


heartbeat = None

try:
    print("\n=== Pico 2 W bring-up ===")

    print("MicroPython :", sys.version)
    print(
        "board       :",
        sys.implementation._machine if hasattr(sys.implementation, "_machine") else "unknown",
    )
    print("CPU freq    : %d MHz" % (machine.freq() // 1_000_000))

    gc.collect()
    print("free RAM    : %d bytes" % gc.mem_free())

    # 1 = power-on, 2 = watchdog, 3 = hard reset. Repeated power-on resets during
    # a motor test is the signature of a brownout - see testing/07_lipo_power.
    print("reset cause :", machine.reset_cause())

    # --- identification pattern ---------------------------------------------
    # Three slow then five fast. Distinct enough that you can tell it apart from
    # the heartbeat, which proves both the LED and code execution.
    print("LED         : 3 slow + 5 fast blinks - watch the board")
    for _ in range(3):
        led.on()
        time.sleep_ms(400)
        led.off()
        time.sleep_ms(400)
    for _ in range(5):
        led.on()
        time.sleep_ms(80)
        led.off()
        time.sleep_ms(80)

    # --- heartbeat on for the remainder --------------------------------------
    _state["lit"] = True
    led.on()
    heartbeat = Timer(-1)
    heartbeat.init(mode=Timer.PERIODIC, period=_TICK_MS, callback=_beat)
    print("LED         : now pulsing - stays lit while this script runs")

    # The CYW43 driver only imports on "W" boards. Its absence is not an error
    # here, but it does tell you which board is actually on the desk.
    try:
        import network

        network.WLAN(network.STA_IF)
        print("WiFi chip   : present (CYW43 responded)")
    except Exception as exc:  # noqa: BLE001
        print("WiFi chip   : NOT detected (%s) - plain Pico 2?" % exc)

    print("\nholding 8s so you can see the pulse...")
    time.sleep(8)

    print("=== all good ===")

finally:
    # An orphaned soft timer would leave the board blinking at an idle REPL,
    # which is exactly the wrong signal.
    if heartbeat is not None:
        heartbeat.deinit()
    led.off()
    print("LED off - script ended")
