"""NMEA parsing, driven through the fake UART."""

import machine

from drivers.gps import GPS, _checksum_ok, _dm_to_degrees

# Real sentences from a NEO-6M with a fix.
GGA_FIX = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47"
RMC_FIX = "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A"
GGA_NO_FIX = "$GPGGA,123519,,,,,0,00,,,M,,M,,*66"


def _gps_with(*sentences):
    uart = machine.UART()
    gps = GPS(uart=uart)
    for sentence in sentences:
        uart.feed((sentence + "\r\n").encode())
    gps.update()
    return gps


def test_degrees_minutes_conversion():
    # 4807.038 N is 48 degrees 7.038 minutes = 48.1173
    assert abs(_dm_to_degrees("4807.038", "N") - 48.1173) < 1e-4
    assert abs(_dm_to_degrees("01131.000", "E") - 11.5167) < 1e-4


def test_southern_and_western_hemispheres_are_negative():
    assert _dm_to_degrees("4807.038", "S") < 0
    assert _dm_to_degrees("01131.000", "W") < 0


def test_checksum_validation():
    assert _checksum_ok(GGA_FIX)
    assert not _checksum_ok("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9*00")


def test_gga_populates_position():
    gps = _gps_with(GGA_FIX)
    assert gps.fix.valid
    assert abs(gps.fix.lat - 48.1173) < 1e-3
    assert abs(gps.fix.lon - 11.5167) < 1e-3
    assert gps.fix.alt_m == 545.4
    assert gps.fix.satellites == 8
    assert gps.fix.hdop == 0.9


def test_no_fix_reports_invalid_but_still_counts_satellites():
    gps = _gps_with(GGA_NO_FIX)
    assert gps.fix.valid is False
    assert gps.fix.satellites == 0


def test_rmc_populates_speed_and_course():
    gps = _gps_with(RMC_FIX)
    assert gps.fix.valid
    assert abs(gps.fix.speed_mps - 22.4 * 0.514444) < 1e-3
    assert gps.fix.course_deg == 84.4


def test_corrupt_sentence_is_counted_not_raised():
    gps = _gps_with("$GPGGA,garbage,data,here*00")
    assert gps.checksum_errors == 1
    assert gps.fix.valid is False


def test_partial_sentence_waits_for_the_rest():
    # Serial data arrives in arbitrary chunks; a split sentence must not be lost.
    uart = machine.UART()
    gps = GPS(uart=uart)
    uart.feed(GGA_FIX[:20].encode())
    gps.update()
    assert gps.fix.valid is False
    uart.feed((GGA_FIX[20:] + "\r\n").encode())
    gps.update()
    assert gps.fix.valid is True


def test_buffer_does_not_grow_without_bound():
    # A module stuck emitting noise with no newline must not exhaust RAM.
    uart = machine.UART()
    gps = GPS(uart=uart)
    for _ in range(20):
        uart.feed(b"x" * 200)
        gps.update()
    assert len(gps._buffer) <= 1024


def test_uses_configured_pins_by_default():
    import config

    gps = GPS()
    assert gps.uart.tx.id == config.GPS_TX_PIN
    assert gps.uart.rx.id == config.GPS_RX_PIN
    assert gps.uart.baudrate == config.GPS_BAUD
