#!/usr/bin/env python3
"""
Enforces the repo's own conventions, so they cannot rot silently.

Checked here rather than by eye because these are exactly the things that decay:
a component folder added without wiring notes, a test script that never mentions
the safety step, a pin documented in one place and changed in another.

Run:  python3 tools/check_structure.py
"""

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
TESTING = ROOT / "testing"

errors = []
checked = 0


def fail(message):
    errors.append(message)


# --- every component folder is complete -------------------------------------
component_dirs = sorted(d for d in TESTING.iterdir() if d.is_dir())
if not component_dirs:
    fail("testing/ has no component folders")

for folder in component_dirs:
    checked += 1
    rel = folder.relative_to(ROOT)

    readme = folder / "README.md"
    if not readme.exists():
        fail(f"{rel}/ has no README.md")
        continue

    scripts = list(folder.glob("test_*.py"))
    if not scripts:
        fail(f"{rel}/ has no test_*.py script")

    text = readme.read_text()

    # A wiring doc without a table is prose, and prose gets misread at 1am with
    # a soldering iron in hand.
    if "|" not in text:
        fail(f"{rel}/README.md has no wiring table")

    # Physical pin numbers are what you actually count on the board.
    if not re.search(r"[Pp]hysical", text):
        fail(f"{rel}/README.md never mentions physical pin numbers")

    # Recorded preference: every procedure needs something observable.
    if not re.search(r"##\s*What you should see", text):
        fail(f"{rel}/README.md is missing a 'What you should see' section")

    for script in scripts:
        checked += 1
        body = script.read_text()
        if not body.lstrip().startswith('"""'):
            fail(f"{script.relative_to(ROOT)} has no module docstring")


# --- every bench script must show a liveness heartbeat -----------------------
# The onboard LED pulsing is the user's failsafe: it says the board is powered
# and its scheduler is running. A script that starts one and never stops it is
# worse than none at all, because it leaves the board blinking at an idle REPL.

for script in sorted(TESTING.rglob("test_*.py")):
    checked += 1
    rel = script.relative_to(ROOT)
    body = script.read_text()

    starts = "Heartbeat(" in body or "Timer(" in body
    if not starts:
        fail(f"{rel} does not start an LED heartbeat")
        continue

    stops = (
        "with Heartbeat(" in body
        or "heartbeat.stop()" in body
        or "heartbeat.deinit()" in body
    )
    if not stops:
        fail(f"{rel} starts a heartbeat but never stops it")


# --- anything that can drive motors must disarm ------------------------------
MOTOR_MARKERS = ("MotorBank", "MOTOR_SLEEP_PIN", "duty_u16")

# config.py only *names* these pins, it cannot drive anything.
GUARD_EXEMPT = {ROOT / "src" / "config.py"}

for script in sorted(TESTING.rglob("*.py")) + sorted((ROOT / "src").rglob("*.py")):
    if script in GUARD_EXEMPT:
        continue
    body = script.read_text()
    if not any(marker in body for marker in MOTOR_MARKERS):
        continue
    checked += 1
    rel = script.relative_to(ROOT)
    # "with MotorBank() as bank" also appears inside a combined with-statement
    # such as `with Heartbeat(), MotorBank() as bank:`, so match the construct
    # rather than the line start.
    has_guard = (
        "finally:" in body
        or "MotorBank() as" in body
        or "def __exit__" in body
    )
    if not has_guard:
        fail(f"{rel} can drive motors but has no finally/context-manager disarm")


# --- pin map is single-sourced ----------------------------------------------
config_text = (ROOT / "src" / "config.py").read_text()
for script in sorted(TESTING.rglob("test_*.py")):
    body = script.read_text()
    # A bare Pin(10) in a component script means the pin map was duplicated.
    hardcoded = re.findall(r"Pin\((\d+)\)", body)
    if hardcoded and "import config" not in body and "from config" not in body:
        fail(
            f"{script.relative_to(ROOT)} hardcodes GPIO {hardcoded} "
            "instead of importing from config.py"
        )

if "MOTOR_PINS" not in config_text:
    fail("src/config.py no longer defines MOTOR_PINS")


# --- report ------------------------------------------------------------------
if errors:
    print(f"structure check FAILED ({len(errors)} problem(s)):\n")
    for message in errors:
        print(f"  - {message}")
    sys.exit(1)

print(f"structure check passed ({checked} items, {len(component_dirs)} components)")
