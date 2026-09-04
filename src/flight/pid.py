"""
PID controller sized for a rate loop.

Two details here matter more than the maths and are the usual cause of a quad
that flies badly for no obvious reason:

1. Integral windup. While saturated the integrator keeps accumulating, then has
   to unwind before the output responds again - which feels like a delayed,
   overshooting aircraft. Clamped here.

2. Derivative kick. Differentiating the *error* means a step change in setpoint
   produces an enormous instantaneous D term. Differentiating the *measurement*
   instead gives identical disturbance rejection with no kick.
"""


class PID:
    def __init__(self, kp, ki, kd, integral_limit=1.0, output_limit=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self.output_limit = output_limit
        self.reset()

    def reset(self):
        self._integral = 0.0
        self._last_measurement = None

    def update(self, setpoint, measurement, dt):
        """dt in seconds. Returns the control output, clamped to output_limit."""
        if dt <= 0.0:
            return 0.0

        error = setpoint - measurement

        self._integral += error * dt
        if self._integral > self.integral_limit:
            self._integral = self.integral_limit
        elif self._integral < -self.integral_limit:
            self._integral = -self.integral_limit

        # Derivative on measurement, negated - see note 2 above.
        if self._last_measurement is None:
            derivative = 0.0
        else:
            derivative = -(measurement - self._last_measurement) / dt
        self._last_measurement = measurement

        output = self.kp * error + self.ki * self._integral + self.kd * derivative

        if output > self.output_limit:
            return self.output_limit
        if output < -self.output_limit:
            return -self.output_limit
        return output
