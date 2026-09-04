"""
Makes `src/` importable as if it were the Pico's filesystem root, and patches the
MicroPython-only bits of `time` into the stdlib module.

On the board, `src/main.py` does `sys.path.append("/")` and imports `config` and
`drivers.*` as top-level names, because everything sits at the filesystem root.
Mirroring that here means CI exercises the same import paths the hardware uses,
rather than a rearranged copy that could drift.
"""

import sys
import time as _time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "tests" / "mocks"))  # `machine` fake wins
sys.path.insert(0, str(ROOT / "src"))


# --- MicroPython time extensions --------------------------------------------
# ticks_* are monotonic millisecond/microsecond counters that wrap. CPython has
# no equivalent, so graft on compatible versions.
if not hasattr(_time, "sleep_ms"):
    _time.sleep_ms = lambda ms: _time.sleep(ms / 1000.0)
    _time.sleep_us = lambda us: _time.sleep(us / 1_000_000.0)
    _time.ticks_ms = lambda: int(_time.monotonic() * 1000)
    _time.ticks_us = lambda: int(_time.monotonic() * 1_000_000)
    _time.ticks_diff = lambda a, b: a - b
    _time.ticks_add = lambda t, delta: t + delta


@pytest.fixture(autouse=True)
def _clean_hardware_state():
    """Every test starts with no Pins or PWMs left over from the last one."""
    import machine

    machine.reset_all()
    yield
    machine.reset_all()
