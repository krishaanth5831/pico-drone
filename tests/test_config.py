"""Pin-map sanity. These catch wiring-config mistakes before they reach a board."""

import config

# Wired to the CYW43 WiFi/Bluetooth chip on Pico W / Pico 2 W boards.
CYW43_RESERVED = {23, 24, 25, 29}


def _assigned_gpios():
    pins = list(config.MOTOR_PINS.values())
    pins.append(config.MOTOR_SLEEP_PIN)
    if config.MOTOR_FAULT_PIN is not None:
        pins.append(config.MOTOR_FAULT_PIN)
    pins += [
        config.IMU_SDA_PIN,
        config.IMU_SCL_PIN,
        config.GPS_TX_PIN,
        config.GPS_RX_PIN,
    ]
    return pins


def test_no_gpio_is_assigned_twice():
    pins = _assigned_gpios()
    duplicates = {p for p in pins if pins.count(p) > 1}
    assert not duplicates, "GPIO assigned to two functions: %s" % duplicates


def test_no_cyw43_pins_used():
    clashes = CYW43_RESERVED & set(_assigned_gpios())
    assert not clashes, "GP%s belong to the WiFi chip on a Pico 2 W" % sorted(clashes)


def test_all_gpios_are_in_range():
    for pin in _assigned_gpios():
        assert 0 <= pin <= 28, "GP%d is not a valid RP2350 GPIO" % pin


def test_four_motors_numbered_one_to_four():
    assert sorted(config.MOTOR_PINS) == [1, 2, 3, 4]


def test_max_duty_respects_two_driver_thermal_limit():
    # Two DRV8833s carry two motors each and thermally shut down at sustained
    # full throttle. If this is ever raised past 0.70, the hardware note in
    # CLAUDE.md and docs/power.md needs revisiting at the same time.
    assert 0.0 < config.MAX_DUTY <= 0.70


def test_min_start_below_max_duty():
    assert 0.0 <= config.MIN_START < config.MAX_DUTY


def test_imu_and_mag_addresses_differ():
    assert config.IMU_ADDR != config.MAG_ADDR
