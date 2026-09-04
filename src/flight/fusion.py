"""
Complementary filter: gyro + accelerometer -> roll and pitch.

Why not just one sensor:
  The gyro is accurate instant to instant but integrating it accumulates drift,
  so over seconds the angle wanders. The accelerometer has no drift - gravity is
  a fixed reference - but on an airframe it reads mostly prop vibration and
  manoeuvring acceleration.

  So: trust the gyro over short intervals, let the accelerometer pull the
  estimate back toward true over long ones. That is one line of algebra and it
  works well enough to fly on. A Madgwick or Mahony filter is the upgrade path
  once this is stable.
"""

import math


class ComplementaryFilter:
    def __init__(self, alpha=0.98):
        """
        alpha is the gyro's share of each update. 0.98 at 500 Hz gives a time
        constant of about 0.1 s - fast enough to reject vibration, slow enough
        that the gyro dominates through a manoeuvre.
        """
        self.alpha = alpha
        self.roll = 0.0   # radians
        self.pitch = 0.0
        self._seeded = False

    def update(self, accel, gyro, dt):
        """
        accel: (ax, ay, az) m/s^2
        gyro:  (gx, gy, gz) deg/s
        dt:    seconds

        Returns (roll, pitch) in radians.
        """
        ax, ay, az = accel
        gx, gy, _ = gyro

        # Attitude implied by gravity alone.
        accel_roll = math.atan2(ay, math.sqrt(ax * ax + az * az))
        accel_pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))

        # First call: adopt the accelerometer outright rather than easing in from
        # zero, otherwise the estimate takes seconds to converge on startup.
        if not self._seeded:
            self.roll, self.pitch = accel_roll, accel_pitch
            self._seeded = True
            return self.roll, self.pitch

        gyro_roll = self.roll + math.radians(gx) * dt
        gyro_pitch = self.pitch + math.radians(gy) * dt

        self.roll = self.alpha * gyro_roll + (1.0 - self.alpha) * accel_roll
        self.pitch = self.alpha * gyro_pitch + (1.0 - self.alpha) * accel_pitch
        return self.roll, self.pitch

    @property
    def roll_deg(self):
        return math.degrees(self.roll)

    @property
    def pitch_deg(self):
        return math.degrees(self.pitch)
