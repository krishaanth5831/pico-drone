#!/usr/bin/env bash
# Copy the library modules to the Pico's filesystem root - deliberately
# EXCLUDING main.py.
#
# The scripts under testing/ import config, drivers/ and flight/. Those imports
# resolve against the BOARD's filesystem, not your computer's - so running a test
# from the editor without uploading first fails with:
#
#     ImportError: no module named 'config'
#
# main.py is left off on purpose. MicroPython auto-runs a file named main.py on
# EVERY boot and EVERY soft-reboot - and Thonny's Run button always soft-reboots
# the board first, before running whatever script you have open. If main.py were
# on the board, every test run would re-enter it instead of your script. Its
# sensor bring-up blocks forever in two places (no-IMU and the streaming loop),
# so the symptom is total silence: the shell just prints
#
#     MPY: soft reboot
#
# over and over, because the board never gets past main.py to run your test.
# This script actively removes main.py from the board if it finds one there,
# for exactly that reason - see remove_main_py() below.
#
# Usage:
#   ./tools/upload.sh            upload, then list what is on the board
#   ./tools/upload.sh --list     just show what is on the board
#   ./tools/upload.sh --clean    wipe the board's root first, then upload

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

# Prefer the repo venv, fall back to whatever is on PATH.
if [ -x .venv/bin/mpremote ]; then
  MPR=(.venv/bin/mpremote)
elif command -v mpremote >/dev/null 2>&1; then
  MPR=(mpremote)
else
  cat <<'MSG'
mpremote is not installed.

  pip install mpremote          (or: pipx install mpremote)

Alternatively upload from Thonny, which needs no extra tools:
  1. View -> Files, so both panes are visible
  2. In the top pane select  config.py, drivers, flight
  3. Right-click -> "Upload to /"
MSG
  exit 1
fi

if ! "${MPR[@]}" connect list 2>/dev/null | grep -q .; then
  echo "No Pico detected. Check the USB cable and that MicroPython is flashed."
  echo "See testing/01_pico_2w/README.md"
  exit 1
fi

list_board() {
  echo
  echo "--- on the board ---"
  "${MPR[@]}" fs ls : || true
  for d in drivers flight; do
    echo "--- /$d ---"
    "${MPR[@]}" fs ls ":$d" 2>/dev/null || echo "  (missing)"
  done
}

remove_main_py() {
  # Runs quietly if main.py is not there. If it IS there - most likely from a
  # board flashed before this fix existed - it must go, or every test run below
  # silently re-enters it instead of the script you meant to run.
  if "${MPR[@]}" fs ls : 2>/dev/null | grep -q "main.py"; then
    echo "removing main.py from the board (it auto-runs and would hijack every test)"
    "${MPR[@]}" fs rm :main.py
  fi
}

if [ "${1:-}" = "--list" ]; then
  if "${MPR[@]}" fs ls : 2>/dev/null | grep -q "main.py"; then
    echo "!! main.py is on the board - it will auto-run on every soft-reboot"
    echo "!! and hijack every test run. Fix with: ./tools/upload.sh"
  fi
  list_board
  exit 0
fi

if [ "${1:-}" = "--clean" ]; then
  echo "wiping board root..."
  "${MPR[@]}" run - <<'PY' || true
import os
def rm(path=""):
    for name, kind, *_ in os.ilistdir(path or "/"):
        full = (path + "/" + name) if path else name
        if kind == 0x4000:
            rm(full)
            os.rmdir(full)
        else:
            os.remove(full)
rm()
print("board wiped")
PY
fi

echo "uploading to the board root..."
remove_main_py
"${MPR[@]}" fs cp src/config.py :

for d in drivers flight; do
  "${MPR[@]}" fs mkdir ":$d" 2>/dev/null || true
  for f in src/$d/*.py; do
    "${MPR[@]}" fs cp "$f" ":$d/$(basename "$f")"
  done
done

echo "done."
list_board

cat <<'MSG'

Now open any script under testing/ in Thonny and press Run.

main.py is deliberately NOT on the board - it auto-runs on every soft-reboot
and would hijack every test. Upload it yourself only when you actually want it
running standalone (mpremote fs cp src/main.py :), and remove it again before
going back to bench testing.
MSG
