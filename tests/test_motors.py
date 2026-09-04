"""Motor bank behaviour, especially the safety-critical arm/disarm ordering."""

import machine
import pytest

import config
from drivers.motors import MotorBank


def test_constructed_disarmed():
    bank = MotorBank()
    assert bank.armed is False
    assert machine.Pin.instances[config.MOTOR_SLEEP_PIN].value() == 0


def test_sleep_pin_driven_low_before_any_pwm_exists():
    # If a PWM channel came up holding a stale duty while SLP was still high,
    # a motor could twitch during construction. SLP must go low first.
    machine.reset_all()
    MotorBank()
    slp = machine.Pin.instances[config.MOTOR_SLEEP_PIN]
    assert slp.history[0] == 0
    assert 1 not in slp.history, "SLP went high during construction"


def test_all_channels_start_at_zero_duty():
    MotorBank()
    assert all(p.duty_u16() == 0 for p in machine.PWM.instances)


def test_arm_raises_sleep_pin():
    bank = MotorBank()
    bank.arm()
    assert bank.armed is True
    assert machine.Pin.instances[config.MOTOR_SLEEP_PIN].value() == 1


def test_disarm_zeroes_throttles_and_drops_sleep_pin():
    bank = MotorBank()
    bank.arm()
    bank.set_all(1.0)
    assert any(p.duty_u16() > 0 for p in machine.PWM.instances)

    bank.disarm()
    assert bank.armed is False
    assert machine.Pin.instances[config.MOTOR_SLEEP_PIN].value() == 0
    assert all(p.duty_u16() == 0 for p in machine.PWM.instances)


def test_throttle_is_scaled_by_max_duty_not_raw():
    bank = MotorBank()
    bank.arm()
    bank.set(1, 1.0)
    expected = int(1.0 * config.MAX_DUTY * 65535)
    assert bank._pwm[1].duty_u16() == expected
    # Full throttle must never mean 100% duty while on two drivers.
    assert bank._pwm[1].duty_u16() < 65535


@pytest.mark.parametrize("value,expected", [(-5.0, 0.0), (0.0, 0.0), (1.0, 1.0), (99.0, 1.0)])
def test_throttle_is_clamped(value, expected):
    bank = MotorBank()
    bank.arm()
    bank.set(1, value)
    assert bank._pwm[1].duty_u16() == int(expected * config.MAX_DUTY * 65535)


def test_unknown_motor_rejected():
    bank = MotorBank()
    with pytest.raises(ValueError):
        bank.set(9, 0.5)


def test_context_manager_disarms_on_exception():
    slp = None
    with pytest.raises(RuntimeError), MotorBank() as bank:
        bank.set_all(0.8)
        slp = machine.Pin.instances[config.MOTOR_SLEEP_PIN]
        assert slp.value() == 1
        raise RuntimeError("simulated crash mid-flight")

    # The whole point: an exception must still cut the drivers.
    assert slp.value() == 0
    assert all(p.duty_u16() == 0 for p in machine.PWM.instances)


def test_fault_pin_reads_low_as_faulted():
    bank = MotorBank()
    assert bank.faulted() is False  # PULL_UP idles high
    machine.Pin.instances[config.MOTOR_FAULT_PIN].low()
    assert bank.faulted() is True


def test_set_many_applies_mixer_output():
    bank = MotorBank()
    bank.arm()
    bank.set_many({1: 0.1, 2: 0.2, 3: 0.3, 4: 0.4})
    for motor, throttle in ((1, 0.1), (2, 0.2), (3, 0.3), (4, 0.4)):
        assert bank._pwm[motor].duty_u16() == int(throttle * config.MAX_DUTY * 65535)
