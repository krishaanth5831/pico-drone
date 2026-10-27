"""
Simulator for the pico-drone. The board's own control code (src/flight/: the
complementary filter, PID, mixer and policy runner) is imported and run as-is,
so what is trained and tuned here is the code that flies.
"""

import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
_SRC = str(REPO / "src")
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)
