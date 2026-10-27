"""
The autotune's saved gains: /gains.json on the board, runs/sitl_gains.json in
the simulator. Same format both places:

    {"gains": {"roll": [kp, ki, kd], "pitch": [...], "yaw": [...]},
     "trim": [roll, pitch]}

load() never fails: a missing, half-written or nonsense file gives back the
defaults it was handed, plus a sentence saying why, which the scripts print -
so it is always visible which gains are flying.

save() writes a temporary file and renames it over the old one, so a brownout
mid-write leaves either the old file or the new one, never half of each.
Call it with the motors OFF: a flash write stalls the CPU for tens of ms.

Pure Python, MicroPython-safe (json + os only).
"""

import json
import os

AXES = ("roll", "pitch", "yaw")
# A saved gain more than this far from the default is not something the
# autotune made (it moves gains at most 0.5x-2x per run, and runs don't
# compound because each scales the defaults) - treat the file as corrupt.
SANE_RATIO = (0.2, 5.0)
MAX_TRIM = 0.3


def _number(v):
    v = float(v)
    if v != v or v in (float("inf"), float("-inf")):
        raise ValueError("not a number")
    return v


def parse(raw, base):
    """(gains, trim) from a decoded file, or raises ValueError / KeyError / TypeError / IndexError."""
    gains = {}
    for name in AXES:
        g = raw["gains"][name]
        if len(g) != 3:
            raise ValueError("%s needs 3 gains" % name)
        g = (_number(g[0]), _number(g[1]), _number(g[2]))
        for i in range(3):
            b = base[name][i]
            if g[i] < 0.0:
                raise ValueError("%s gain below zero" % name)
            if b > 0.0 and not (SANE_RATIO[0] <= g[i] / b <= SANE_RATIO[1]):
                raise ValueError("%s gain %.5f is far from the default %.5f" % (name, g[i], b))
            if b == 0.0 and g[i] != 0.0:
                raise ValueError("%s gain should be 0" % name)
        gains[name] = g
    trim = (_number(raw["trim"][0]), _number(raw["trim"][1]))
    if abs(trim[0]) > MAX_TRIM or abs(trim[1]) > MAX_TRIM:
        raise ValueError("trim out of range")
    return gains, trim


def load(path, base, trim=(0.0, 0.0)):
    """
    (gains, trim, source). base / trim: what to fly without a usable file.
    source: a short sentence for the console, e.g. "saved autotune gains
    (/gains.json)" or "defaults - /gains.json: trim out of range".
    """
    try:
        with open(path) as f:
            raw = json.loads(f.read())
    except OSError:
        return dict(base), (trim[0], trim[1]), "defaults (no %s yet)" % path
    except ValueError:
        return dict(base), (trim[0], trim[1]), "defaults - %s is not valid JSON, ignored" % path
    try:
        gains, saved_trim = parse(raw, base)
    except (ValueError, KeyError, TypeError, IndexError) as exc:
        return dict(base), (trim[0], trim[1]), "defaults - %s ignored: %s" % (path, exc)
    return gains, saved_trim, "saved autotune gains (%s)" % path


def save(path, gains, trim):
    """Write atomically: temp file, then rename over the old one."""
    data = {"gains": {}, "trim": [trim[0], trim[1]]}
    for name in AXES:
        data["gains"][name] = [gains[name][0], gains[name][1], gains[name][2]]
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        f.write(json.dumps(data))
    try:
        os.rename(tmp, path)
    except OSError:
        # A filesystem that won't rename over an existing file.
        try:  # noqa: SIM105 - no contextlib on MicroPython
            os.remove(path)
        except OSError:
            pass
        os.rename(tmp, path)
