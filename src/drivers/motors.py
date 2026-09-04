"""
Four coreless motors behind two DRV8833 dual H-bridges.

Each motor uses one channel of a driver with that channel's second input tied to
GND on the breakout, so the motor is unidirectional: PWM one pin, the other is
hard low. Fast-decay drive, which is what you want for a brushed coreless motor.

Safety model
------------
The SLP pin on both drivers is wired to one GPIO. Pulled low, the DRV8833
outputs go high-impedance and the motors are dead no matter what the PWM
registers contain. Every path out of this module - including exceptions - must
end with disarm(). Use MotorBank as a context manager and that is automatic.
"""

from machine import PWM, Pin

from config import (
    MAX_DUTY,
    MIN_START,
    MOTOR_FAULT_PIN,
    MOTOR_PINS,
    MOTOR_PWM_FREQ,
    MOTOR_SLEEP_PIN,
)

_FULL = 65535


class MotorBank:
    """All four motors, plus the hardware arm/disarm line."""

    def __init__(self, max_duty=MAX_DUTY):
        self.max_duty = max_duty

        # Bring the sleep line up before the PWMs exist, so there is no window
        # where a stale duty could reach a motor.
        self._slp = Pin(MOTOR_SLEEP_PIN, Pin.OUT)
        self._slp.low()
        self.armed = False

        self._pwm = {}
        for number, gpio in MOTOR_PINS.items():
            pwm = PWM(Pin(gpio))
            pwm.freq(MOTOR_PWM_FREQ)
            pwm.duty_u16(0)
            self._pwm[number] = pwm

        # nFAULT is open-drain: low means over-current or thermal shutdown.
        self._fault = None
        if MOTOR_FAULT_PIN is not None:
            self._fault = Pin(MOTOR_FAULT_PIN, Pin.IN, Pin.PULL_UP)

    # -- arming --------------------------------------------------------------

    def arm(self):
        """Wake both drivers. Throttles are forced to zero first."""
        self.set_all(0.0)
        self._slp.high()
        self.armed = True

    def disarm(self):
        """Zero every throttle, then cut the drivers at the hardware level."""
        for pwm in self._pwm.values():
            pwm.duty_u16(0)
        self._slp.low()
        self.armed = False

    def faulted(self):
        """True if either DRV8833 is reporting over-current or over-temperature."""
        if self._fault is None:
            return False
        return self._fault.value() == 0

    # -- throttle ------------------------------------------------------------

    def set(self, motor, throttle):
        """
        Set one motor. `throttle` is 0.0 to 1.0 and is scaled by max_duty, so
        1.0 means "as fast as this build allows", not "100% duty cycle".
        """
        if motor not in self._pwm:
            raise ValueError("no motor %s" % motor)
        throttle = 0.0 if throttle < 0.0 else (1.0 if throttle > 1.0 else throttle)
        self._pwm[motor].duty_u16(int(throttle * self.max_duty * _FULL))

    def set_all(self, throttle):
        for motor in self._pwm:
            self.set(motor, throttle)

    def set_many(self, throttles):
        """Apply a {motor_number: throttle} mapping, e.g. from the mixer."""
        for motor, throttle in throttles.items():
            self.set(motor, throttle)

    def stop(self):
        """Coast every motor but stay armed."""
        self.set_all(0.0)

    # -- context manager -----------------------------------------------------

    def __enter__(self):
        self.arm()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.disarm()
        return False  # never swallow the exception

    def deinit(self):
        self.disarm()
        for pwm in self._pwm.values():
            pwm.deinit()


def ramp(bank, motor, start, end, seconds, step_ms=20):
    """
    Ease one motor between two throttles.

    Stepping straight to a high throttle draws an inrush spike several times the
    running current. On a 1S pack that sags the rail hard enough to reset the
    Pico, so every throttle change of any size goes through here.
    """
    import time

    steps = max(1, int(seconds * 1000 / step_ms))
    for i in range(steps + 1):
        bank.set(motor, start + (end - start) * i / steps)
        time.sleep_ms(step_ms)


__all__ = ["MotorBank", "ramp", "MIN_START"]
