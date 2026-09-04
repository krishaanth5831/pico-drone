"""
GY-GPS6MV2 breakout / u-blox NEO-6M - NMEA over UART.

Wiring is crossed: module TX -> Pico RX, module RX -> Pico TX. Getting that
backwards gives total silence with no error, and is the single most common
mistake with this module.

The parser is deliberately minimal - GGA for position and fix quality, RMC for
speed and course. That is everything a flight controller needs; the rest of the
NMEA sentence set is noise.
"""

import time

from machine import UART, Pin

from config import GPS_BAUD, GPS_RX_PIN, GPS_TX_PIN, GPS_UART_ID


class Fix:
    """One position solution. `valid` is False until satellites are acquired."""

    __slots__ = ("lat", "lon", "alt_m", "quality", "satellites",
                 "hdop", "speed_mps", "course_deg", "utc", "valid")

    def __init__(self):
        self.lat = None
        self.lon = None
        self.alt_m = None
        self.quality = 0
        self.satellites = 0
        self.hdop = None
        self.speed_mps = None
        self.course_deg = None
        self.utc = None
        self.valid = False

    def __repr__(self):
        if not self.valid:
            return "<Fix: no fix, %d sats visible>" % self.satellites
        return "<Fix %.6f, %.6f  alt %.1fm  sats %d  hdop %s>" % (
            self.lat, self.lon, self.alt_m or 0.0, self.satellites, self.hdop
        )


def _dm_to_degrees(value, hemisphere):
    """
    NMEA gives latitude as ddmm.mmmm and longitude as dddmm.mmmm - degrees and
    minutes glued together, not a decimal degree. Split and convert.
    """
    if not value or not hemisphere:
        return None
    dot = value.find(".")
    if dot < 3:
        return None
    degrees = int(value[: dot - 2])
    minutes = float(value[dot - 2 :])
    result = degrees + minutes / 60.0
    return -result if hemisphere in ("S", "W") else result


def _checksum_ok(sentence):
    """NMEA checksum is an XOR of everything between '$' and '*'."""
    star = sentence.rfind("*")
    if star < 0 or star + 3 > len(sentence):
        return False
    calculated = 0
    for ch in sentence[1:star]:
        calculated ^= ord(ch)
    try:
        return calculated == int(sentence[star + 1 : star + 3], 16)
    except ValueError:
        return False


class GPS:
    def __init__(self, uart=None):
        self.uart = uart or UART(
            GPS_UART_ID, baudrate=GPS_BAUD, tx=Pin(GPS_TX_PIN), rx=Pin(GPS_RX_PIN)
        )
        self.fix = Fix()
        self._buffer = b""
        self.sentences_seen = 0
        self.checksum_errors = 0

    def update(self):
        """
        Drain the UART and parse whatever complete sentences arrived.

        Non-blocking - call it every loop. Returns True if the fix was updated.
        """
        updated = False
        if self.uart.any():
            self._buffer += self.uart.read() or b""

        while b"\n" in self._buffer:
            line, self._buffer = self._buffer.split(b"\n", 1)
            text = line.decode("ascii", "ignore").strip()
            if text.startswith("$") and self._parse(text):
                updated = True

        # A corrupt stream with no newline would grow without bound.
        if len(self._buffer) > 1024:
            self._buffer = b""

        return updated

    def _parse(self, sentence):
        self.sentences_seen += 1
        if not _checksum_ok(sentence):
            self.checksum_errors += 1
            return False

        fields = sentence[1 : sentence.rfind("*")].split(",")
        kind = fields[0][2:]  # strip talker ID: GP, GN, GL...

        if kind == "GGA" and len(fields) >= 15:
            self.fix.utc = fields[1] or None
            self.fix.quality = int(fields[6]) if fields[6] else 0
            self.fix.satellites = int(fields[7]) if fields[7] else 0
            self.fix.hdop = float(fields[8]) if fields[8] else None
            self.fix.valid = self.fix.quality > 0
            if self.fix.valid:
                self.fix.lat = _dm_to_degrees(fields[2], fields[3])
                self.fix.lon = _dm_to_degrees(fields[4], fields[5])
                self.fix.alt_m = float(fields[9]) if fields[9] else None
            return True

        if kind == "RMC" and len(fields) >= 10:
            self.fix.valid = fields[2] == "A"
            if self.fix.valid:
                self.fix.lat = _dm_to_degrees(fields[3], fields[4])
                self.fix.lon = _dm_to_degrees(fields[5], fields[6])
                if fields[7]:
                    self.fix.speed_mps = float(fields[7]) * 0.514444  # knots
                if fields[8]:
                    self.fix.course_deg = float(fields[8])
            return True

        return False

    def wait_for_fix(self, timeout_s=300, verbose=True):
        """
        Block until the module reports a real fix.

        Cold start outdoors is 30 s to several minutes. Indoors this will simply
        time out - the NEO-6M needs actual sky, there is no way around it.
        """
        deadline = time.ticks_add(time.ticks_ms(), int(timeout_s * 1000))
        last_report = 0
        while time.ticks_diff(deadline, time.ticks_ms()) > 0:
            self.update()
            if self.fix.valid:
                return self.fix
            now = time.ticks_ms()
            if verbose and time.ticks_diff(now, last_report) > 5000:
                print("waiting for fix... %d sats visible" % self.fix.satellites)
                last_report = now
            time.sleep_ms(100)
        return None
