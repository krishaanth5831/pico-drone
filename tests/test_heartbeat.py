"""
LED heartbeat.

The mock Timer stores its callback, so these tests drive it tick by tick and
assert what the LED actually did - not merely that a timer was constructed.
"""

import machine
import pytest

from drivers.heartbeat import MODES, Heartbeat


def _led():
    import config

    return machine.Pin.instances[config.LED_PIN]


def test_lights_immediately_on_start():
    # The point of the indicator is that it is lit the moment the script begins,
    # not one timer period later.
    hb = Heartbeat().start()
    assert _led().value() == 1
    assert hb.running is True


def test_solid_mode_needs_no_timer():
    Heartbeat(mode="solid").start()
    assert _led().value() == 1
    assert machine.Timer.instances == []


def test_pulse_starts_a_periodic_timer():
    Heartbeat(mode="pulse").start()
    assert len(machine.Timer.instances) == 1
    timer = machine.Timer.instances[0]
    assert timer.mode == machine.Timer.PERIODIC
    assert timer.active is True


def test_pulse_actually_toggles_the_led():
    hb = Heartbeat(mode="pulse").start()
    timer = machine.Timer.instances[0]
    on_ms, off_ms = MODES["pulse"]

    assert _led().value() == 1
    # Tick past the "on" portion and the LED must go dark.
    timer.fire(on_ms // timer.period + 1)
    assert _led().value() == 0
    # Tick through the rest of the cycle and it must come back.
    timer.fire(off_ms // timer.period + 1)
    assert _led().value() == 1
    hb.stop()


def test_pulse_is_mostly_on():
    # It has to read as a glow, not a blink - that was the point of the mode.
    on_ms, off_ms = MODES["pulse"]
    assert on_ms > off_ms * 5


def test_stop_kills_the_timer_and_the_led():
    hb = Heartbeat(mode="pulse").start()
    timer = machine.Timer.instances[0]
    hb.stop()
    assert hb.running is False
    assert timer.active is False
    assert timer.deinit_count == 1
    assert _led().value() == 0


def test_stop_is_idempotent():
    hb = Heartbeat(mode="pulse").start()
    hb.stop()
    hb.stop()  # must not raise
    assert _led().value() == 0


def test_context_manager_stops_on_exception():
    # A crashing script must not leave the board blinking at an idle REPL.
    with pytest.raises(RuntimeError), Heartbeat(mode="pulse"):
        assert _led().value() == 1
        raise RuntimeError("simulated failure")
    assert _led().value() == 0
    assert machine.Timer.instances[0].active is False


def test_double_start_does_not_leak_a_second_timer():
    hb = Heartbeat(mode="pulse")
    hb.start()
    hb.start()
    assert len(machine.Timer.instances) == 1
    hb.stop()


def test_rejects_unknown_mode():
    with pytest.raises(ValueError, match="mode must be one of"):
        Heartbeat(mode="strobe")


@pytest.mark.parametrize("mode", sorted(MODES))
def test_every_mode_starts_and_stops_cleanly(mode):
    hb = Heartbeat(mode=mode)
    hb.start()
    assert _led().value() == 1
    hb.stop()
    assert _led().value() == 0
