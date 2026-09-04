"""
Onboard-LED liveness indicator.

Runs on a soft timer, so it keeps going through blocking calls - the six-second
multimeter pauses in the DRV8833 test, the minutes of `wait_for_fix()` in the GPS
test. Scripts do not have to call anything from inside their loops.

WHAT THE LED PROVES, precisely:

  lit / pulsing   the MCU is powered and its scheduler is running
  went dark       the script ended, or the board lost power
  restarted       the board RESET - on this airframe that means a brownout
                  (see testing/07_lipo_power)

  It does NOT prove the main loop is progressing. A soft timer fires from the
  scheduler and carries on even if your code is stuck in a loop. This detects
  reset and power loss, which is what a coreless quad actually suffers from.

The Pico 2 W's LED hangs off the CYW43 WiFi chip rather than a GPIO, so it is
on/off only and cannot be PWM'd. "Pulse" is therefore a long on and a short off,
which reads as a glow at a glance but still visibly beats.
"""

from machine import Pin, Timer

from config import LED_PIN

# Coarse enough that talking to the CYW43 chip stays cheap, fine enough that the
# on/off split lands close to the requested milliseconds.
_TICK_MS = 25

# mode -> (on_ms, off_ms)
MODES = {
    "pulse": (900, 100),   # reads as a steady glow, but you can see it beat
    "blink": (250, 250),   # unmistakable, for across-the-room visibility
    "solid": (1, 0),       # continuously lit, no timer needed
}


class Heartbeat:
    def __init__(self, mode="pulse", pin=LED_PIN, timer_id=-1):
        if mode not in MODES:
            raise ValueError("mode must be one of %s" % sorted(MODES))
        self.mode = mode
        self.on_ms, self.off_ms = MODES[mode]
        self._led = Pin(pin, Pin.OUT)
        self._timer_id = timer_id
        self._timer = None
        self._elapsed = 0
        self._lit = False
        self.running = False

    # -- LED access ----------------------------------------------------------

    def _write(self, lit):
        # Only touch the pin on an actual change: on a "W" board every write is
        # an SPI transaction to the WiFi chip, not a register poke.
        if lit == self._lit:
            return
        try:
            self._led.value(1 if lit else 0)
            self._lit = lit
        # MicroPython ships no contextlib, so suppress() is unavailable here.
        except Exception:  # noqa: BLE001
            # A failing LED must never take down the test it is reporting on.
            pass

    # -- lifecycle -----------------------------------------------------------

    def start(self):
        if self.running:
            return self
        self._write(True)
        self.running = True

        if self.mode != "solid":
            self._elapsed = 0
            self._timer = Timer(self._timer_id)
            self._timer.init(
                mode=Timer.PERIODIC, period=_TICK_MS, callback=self._tick
            )
        return self

    def _tick(self, _timer):
        cycle = self.on_ms + self.off_ms
        self._elapsed = (self._elapsed + _TICK_MS) % cycle
        self._write(self._elapsed < self.on_ms)

    def stop(self):
        """Always call this. An orphaned timer leaves the board blinking at an
        idle REPL, which is exactly the wrong signal."""
        self.running = False
        if self._timer is not None:
            try:
                self._timer.deinit()
            except Exception:  # noqa: BLE001
                pass
            self._timer = None
        self._write(False)

    # -- context manager -----------------------------------------------------

    def __enter__(self):
        return self.start()

    def __exit__(self, exc_type, exc, tb):
        self.stop()
        return False  # never swallow the exception
