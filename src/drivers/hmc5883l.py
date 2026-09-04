"""
HMC5883L 3-axis magnetometer, and the QMC5883L clone that ships under the same
silkscreen.

Supplies the absolute yaw reference the MPU6050 cannot. Without it heading drifts
slowly; with it you can hold a compass course. Only needed once you want GPS
position hold or return-to-home.

Telling the two apart:
  HMC5883L (Honeywell)  I2C 0x1E, ID registers spell 'H43', data order X Z Y
  QMC5883L (QST clone)  I2C 0x0D, no ID string,             data order X Y Z

detect() handles this so callers do not have to care which one is soldered on.
"""

import math
import struct
import time

from machine import I2C, Pin

from config import I2C_FREQ, IMU_I2C_ID, IMU_SCL_PIN, IMU_SDA_PIN

# --- HMC5883L ---------------------------------------------------------------
_HMC_ADDR = 0x1E
_HMC_CONFIG_A = 0x00
_HMC_CONFIG_B = 0x01
_HMC_MODE = 0x02
_HMC_DATA = 0x03
_HMC_ID_A = 0x0A

# Gain register value -> LSB per Gauss. 0x20 (+/- 1.3 Ga) is the default and is
# the right choice near motors: Earth's field is ~0.5 Ga, so it leaves headroom
# for the distortion your own power wiring adds.
_HMC_GAINS = {
    0x00: 1370.0, 0x20: 1090.0, 0x40: 820.0, 0x60: 660.0,
    0x80: 440.0, 0xA0: 390.0, 0xC0: 330.0, 0xE0: 230.0,
}

# --- QMC5883L ---------------------------------------------------------------
_QMC_ADDR = 0x0D
_QMC_DATA = 0x00
_QMC_STATUS = 0x06
_QMC_CONTROL_1 = 0x09
_QMC_SET_RESET = 0x0B


class _Base:
    """Shared calibration and heading maths."""

    def __init__(self):
        # Hard-iron offset: constant field from magnets and DC currents on the
        # airframe. Subtracted from every reading.
        self.offset = (0.0, 0.0, 0.0)
        # Soft-iron scale: ferrous metal distorting the field into an ellipse.
        # A per-axis gain is a crude but effective correction.
        self.scale = (1.0, 1.0, 1.0)

    def read(self):
        """Calibrated (x, y, z) in microtesla."""
        raw = self.read_raw_ut()
        return tuple(
            (raw[i] - self.offset[i]) * self.scale[i] for i in range(3)
        )

    def heading(self, pitch=0.0, roll=0.0):
        """
        Compass heading in degrees, 0 = magnetic north, increasing clockwise.

        pitch and roll in radians tilt-compensate the reading. A magnetometer
        held level reads heading directly, but tilt it and the vertical component
        of Earth's field leaks into the horizontal axes - on a quad that shows up
        as heading swinging wildly whenever it banks.
        """
        x, y, z = self.read()
        cp, sp = math.cos(pitch), math.sin(pitch)
        cr, sr = math.cos(roll), math.sin(roll)
        xh = x * cp + y * sr * sp + z * cr * sp
        yh = y * cr - z * sr
        deg = math.degrees(math.atan2(-yh, xh))
        return deg + 360.0 if deg < 0 else deg

    def calibrate(self, seconds=30, sample_ms=50, verbose=True):
        """
        Min/max calibration. Rotate the board slowly through every orientation -
        a lazy figure-of-eight in all three axes - for the whole duration.

        Do this with the aircraft fully assembled and the battery connected. The
        distortion you are cancelling comes from the airframe itself, so a
        calibration done on a bare bench is worthless once it is bolted down.
        """
        lo = [1e9] * 3
        hi = [-1e9] * 3
        deadline = time.ticks_add(time.ticks_ms(), int(seconds * 1000))
        while time.ticks_diff(deadline, time.ticks_ms()) > 0:
            for i, v in enumerate(self.read_raw_ut()):
                lo[i] = min(lo[i], v)
                hi[i] = max(hi[i], v)
            time.sleep_ms(sample_ms)

        self.offset = tuple((hi[i] + lo[i]) / 2.0 for i in range(3))
        spans = [(hi[i] - lo[i]) / 2.0 for i in range(3)]
        mean = sum(spans) / 3.0
        self.scale = tuple(mean / s if s > 1e-6 else 1.0 for s in spans)
        if verbose:
            print("offset:", self.offset)
            print("scale: ", self.scale)
        return self.offset, self.scale


class HMC5883L(_Base):
    def __init__(self, i2c, addr=_HMC_ADDR, gain=0x20):
        super().__init__()
        self.i2c = i2c
        self.addr = addr
        self._lsb_per_gauss = _HMC_GAINS[gain]

        self.i2c.writeto_mem(addr, _HMC_CONFIG_A, b"\x78")  # 8 avg, 75 Hz
        self.i2c.writeto_mem(addr, _HMC_CONFIG_B, bytes([gain]))
        self.i2c.writeto_mem(addr, _HMC_MODE, b"\x00")  # continuous
        time.sleep_ms(10)

    def read_raw_ut(self):
        # Note the register order is X, Z, Y - not X, Y, Z. Getting this wrong
        # produces a compass that looks plausible and is wrong by 90 degrees.
        x, z, y = struct.unpack(">hhh", self.i2c.readfrom_mem(self.addr, _HMC_DATA, 6))
        k = 100.0 / self._lsb_per_gauss  # Gauss -> microtesla
        return (x * k, y * k, z * k)


class QMC5883L(_Base):
    def __init__(self, i2c, addr=_QMC_ADDR):
        super().__init__()
        self.i2c = i2c
        self.addr = addr
        self.i2c.writeto_mem(addr, _QMC_SET_RESET, b"\x01")
        # continuous | 200 Hz | +/- 8 G | 512 oversampling
        self.i2c.writeto_mem(addr, _QMC_CONTROL_1, b"\x1D")
        time.sleep_ms(10)
        self._lsb_per_gauss = 3000.0  # +/- 8 G range

    def read_raw_ut(self):
        x, y, z = struct.unpack("<hhh", self.i2c.readfrom_mem(self.addr, _QMC_DATA, 6))
        k = 100.0 / self._lsb_per_gauss
        return (x * k, y * k, z * k)


def detect(i2c=None):
    """
    Return whichever magnetometer is actually on the bus.

    Raises OSError if neither answers, which on this build almost always means
    the module is on the wrong I2C pins or unpowered.
    """
    if i2c is None:
        i2c = I2C(IMU_I2C_ID, sda=Pin(IMU_SDA_PIN), scl=Pin(IMU_SCL_PIN), freq=I2C_FREQ)

    found = i2c.scan()
    if _HMC_ADDR in found:
        # 'H43' confirms a genuine Honeywell part. Anything else answering at
        # 0x1E is a register-compatible clone, so drive it the same way.
        ident = i2c.readfrom_mem(_HMC_ADDR, _HMC_ID_A, 3)
        if ident != b"H43":
            print("warning: 0x1E responded but ID was %r, not b'H43'" % ident)
        return HMC5883L(i2c)
    if _QMC_ADDR in found:
        return QMC5883L(i2c)
    raise OSError(
        "no magnetometer found. saw: %s (expected 0x1E for HMC5883L "
        "or 0x0D for a QMC5883L clone)" % [hex(a) for a in found]
    )
