"""
GY-GPS6MV2 / u-blox NEO-6M bring-up.

Stage 1 dumps raw NMEA, which proves the wiring regardless of satellite lock.
Stage 2 parses sentences and waits for an actual fix.

Run this OUTDOORS. The NEO-6M cannot get a fix indoors.

The onboard LED pulses for as long as this runs - useful here because stage 2 can
block for minutes with nothing else to show the board is alive. No motors are
touched.
"""

import sys
import time

sys.path.append("/")

from machine import UART, Pin  # noqa: E402

try:
    import config  # noqa: E402
    from drivers.gps import GPS  # noqa: E402
    from drivers.heartbeat import Heartbeat  # noqa: E402
except ImportError as exc:
    raise SystemExit(
        "\n%s\n\n"
        "The library modules are not on the Pico yet. These imports resolve\n"
        "against the BOARD's filesystem, not your computer's, so opening this\n"
        "file in an editor is not enough.\n\n"
        "Upload them once, then run this again:\n"
        "  Thonny   : View -> Files. In the top pane select config.py, drivers\n"
        "             and flight, right-click -> 'Upload to /'\n"
        "  Terminal : ./tools/upload.sh\n" % exc
    )

RAW_SECONDS = 10
FIX_TIMEOUT = 300

print("\n=== GY-GPS6MV2 / NEO-6M test ===")
print(
    "UART%d  tx=GP%d (pin 1)  rx=GP%d (pin 2)  %d baud"
    % (config.GPS_UART_ID, config.GPS_TX_PIN, config.GPS_RX_PIN, config.GPS_BAUD)
)
print("LED pulses for as long as this runs")

# The context manager matters more here than anywhere else: both SystemExit paths
# below must still stop the heartbeat, or the board is left blinking at an idle
# REPL after the script bails out.
with Heartbeat():
    uart = UART(
        config.GPS_UART_ID,
        baudrate=config.GPS_BAUD,
        tx=Pin(config.GPS_TX_PIN),
        rx=Pin(config.GPS_RX_PIN),
    )

    # --- stage 1: raw --------------------------------------------------------
    print("\n--- raw NMEA for %ds (proves wiring) ---" % RAW_SECONDS)
    deadline = time.ticks_add(time.ticks_ms(), RAW_SECONDS * 1000)
    buffer = b""
    seen = 0

    while time.ticks_diff(deadline, time.ticks_ms()) > 0:
        if uart.any():
            buffer += uart.read() or b""
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                text = line.decode("ascii", "ignore").strip()
                if text.startswith("$"):
                    seen += 1
                    print(text)
        time.sleep_ms(20)

    if seen == 0:
        raise SystemExit(
            "\nNO DATA.\n"
            "  1. TX/RX are almost certainly swapped - GPS TX must go to Pico RX "
            "(GP%d, pin 2)\n"
            "  2. Check VCC at the module reads ~3.3 V\n"
            "  3. If your board's header order differs, trust the silkscreen"
            % config.GPS_RX_PIN
        )

    print("\n%d sentences in %ds - wiring is good" % (seen, RAW_SECONDS))

    # --- stage 2: parsed -----------------------------------------------------
    print("\n--- waiting for fix (up to %ds, outdoors only) ---" % FIX_TIMEOUT)
    gps = GPS(uart=uart)
    started = time.ticks_ms()

    fix = gps.wait_for_fix(timeout_s=FIX_TIMEOUT)
    elapsed = time.ticks_diff(time.ticks_ms(), started) // 1000

    if fix is None:
        print("\nno fix after %ds." % elapsed)
        print("  sentences parsed : %d" % gps.sentences_seen)
        print("  checksum errors  : %d" % gps.checksum_errors)
        print("  satellites seen  : %d" % gps.fix.satellites)
        print("\nindoors this is expected. outdoors, check the antenna faces UP.")
    else:
        print("\nFIX ACQUIRED after %ds" % elapsed)
        print("  latitude   : %.6f" % fix.lat)
        print("  longitude  : %.6f" % fix.lon)
        print("  altitude   : %.1f m" % (fix.alt_m or 0.0))
        print("  satellites : %d" % fix.satellites)
        print("  hdop       : %s" % fix.hdop)
        print("\nthe red PPS LED on the module should now blink once per second")

print("LED off - script ended")
