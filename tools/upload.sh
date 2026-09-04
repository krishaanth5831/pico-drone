#!/usr/bin/env bash
# Copy the library modules to the Pico's filesystem root.
#
# The scripts under testing/ import config, drivers/ and flight/. Those imports
# resolve against the BOARD's filesystem, not your computer's - so running a test
# from the editor without uploading first fails with:
#
#     ImportError: no module named 'config'
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

if [ "${1:-}" = "--list" ]; then
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
"${MPR[@]}" fs cp src/config.py : 
"${MPR[@]}" fs cp src/main.py :

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
MSG
