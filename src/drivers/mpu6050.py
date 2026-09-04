"""
GY-521 breakout / InvenSense MPU6050 - 3-axis gyro + 3-axis accelerometer.

This is the sensor the aircraft actually flies on. The gyro feeds the inner rate
loop; the accelerometer only corrects the gyro's slow drift and is far too noisy
on an airframe to be trusted directly.

Datasheet register names are kept verbatim so they can be grepped against it.
"""

import struct
import time

from machine import I2C, Pin

from config import I2C_FREQ, IMU_ADDR, IMU_I2C_ID, IMU_SCL_PIN, IMU_SDA_PIN

# Registers
_SMPLRT_DIV = 0x19
_CONFIG = 0x1A
_GYRO_CONFIG = 0x1B
_ACCEL_CONFIG = 0x1C
_INT_PIN_CFG = 0x37
_ACCEL_XOUT_H = 0x3B
_PWR_MGMT_1 = 0x6B
_WHO_AM_I = 0x75

# Full-scale ranges -> (register bits, LSB per unit)
_ACCEL_RANGES = {2: (0x00, 16384.0), 4: (0x08, 8192.0), 8: (0x10, 4096.0), 16: (0x18, 2048.0)}
_GYRO_RANGES = {250: (0x00, 131.0), 500: (0x08, 65.5), 1000: (0x10, 32.8), 2000: (0x18, 16.4)}

STANDARD_GRAVITY = 9.80665


class MPU6050:
    def __init__(self, i2c=None, addr=IMU_ADDR, accel_range=8, gyro_range=1000, dlpf=3):
        """
        accel_range: +/- g. 8 g leaves headroom for prop vibration clipping.
        gyro_range:  +/- deg/s. 1000 is comfortable for a small quad; go to 2000
                     if you ever see the rate reading saturate during a flip.
        dlpf:        digital low-pass filter, 0-6. 3 gives ~44 Hz, a reasonable
                     starting point that cuts frame vibration without adding so
                     much phase lag that the rate loop goes unstable.
        """
        self.i2c = i2c or I2C(
            IMU_I2C_ID, sda=Pin(IMU_SDA_PIN), scl=Pin(IMU_SCL_PIN), freq=I2C_FREQ
        )
        self.addr = addr

        who = self._read(_WHO_AM_I, 1)[0]
        # Genuine MPU6050 reports 0x68. Common clones (MPU6500/9250) report
        # 0x70/0x71/0x73 and are register-compatible for everything used here.
        if who not in (0x68, 0x70, 0x71, 0x73, 0x98):
            raise OSError("no MPU6050 at 0x%02X (WHO_AM_I returned 0x%02X)" % (addr, who))
        self.who_am_i = who

        # Wake from sleep and clock off gyro X - more stable than the internal
        # 8 MHz oscillator, per the datasheet's own recommendation.
        self._write(_PWR_MGMT_1, 0x00)
        time.sleep_ms(50)
        self._write(_PWR_MGMT_1, 0x01)
        time.sleep_ms(10)

        self._write(_CONFIG, dlpf & 0x07)
        self._write(_SMPLRT_DIV, 0x00)  # 1 kHz with DLPF enabled

        bits, self._accel_lsb = _ACCEL_RANGES[accel_range]
        self._write(_ACCEL_CONFIG, bits)
        bits, self._gyro_lsb = _GYRO_RANGES[gyro_range]
        self._write(_GYRO_CONFIG, bits)

        # Let the HMC5883L on the same bus be reached directly rather than
        # through the MPU's aux bus. Harmless on a bare GY-521.
        self._write(_INT_PIN_CFG, 0x02)

        self.gyro_bias = (0.0, 0.0, 0.0)
        time.sleep_ms(100)

    # -- raw I2C -------------------------------------------------------------

    def _read(self, reg, length):
        return self.i2c.readfrom_mem(self.addr, reg, length)

    def _write(self, reg, value):
        self.i2c.writeto_mem(self.addr, reg, bytes([value]))

    # -- measurements --------------------------------------------------------

    def read_raw(self):
        """All seven 16-bit words in one burst: ax ay az temp gx gy gz."""
        return struct.unpack(">hhhhhhh", self._read(_ACCEL_XOUT_H, 14))

    def read(self):
        """
        Returns (accel, gyro, temp_c) with accel in m/s^2, gyro in deg/s,
        gyro bias already subtracted.
        """
        ax, ay, az, raw_t, gx, gy, gz = self.read_raw()
        bx, by, bz = self.gyro_bias
        accel = (
            ax / self._accel_lsb * STANDARD_GRAVITY,
            ay / self._accel_lsb * STANDARD_GRAVITY,
            az / self._accel_lsb * STANDARD_GRAVITY,
        )
        gyro = (
            gx / self._gyro_lsb - bx,
            gy / self._gyro_lsb - by,
            gz / self._gyro_lsb - bz,
        )
        return accel, gyro, raw_t / 340.0 + 36.53

    def calibrate_gyro(self, samples=500, delay_ms=3):
        """
        Average the gyro at rest to find its zero offset.

        The board must be completely still. Every MPU6050 has a few deg/s of
        bias, and integrating that uncorrected walks your heading away within
        seconds.
        """
        sx = sy = sz = 0
        for _ in range(samples):
            _, _, _, _, gx, gy, gz = self.read_raw()
            sx += gx
            sy += gy
            sz += gz
            time.sleep_ms(delay_ms)
        self.gyro_bias = (
            sx / samples / self._gyro_lsb,
            sy / samples / self._gyro_lsb,
            sz / samples / self._gyro_lsb,
        )
        return self.gyro_bias
