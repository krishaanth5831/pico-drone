"""Saved autotune gains (src/flight/gains_file.py): load never fails, save is atomic."""

import json

from flight import gains_file

BASE = {"roll": (0.0040, 0.0020, 0.00006), "pitch": (0.0040, 0.0020, 0.00006),
        "yaw": (0.0050, 0.0010, 0.0)}
TUNED = {"roll": (0.0027, 0.0013, 0.00004), "pitch": (0.0024, 0.0012, 0.00004),
         "yaw": (0.0054, 0.0011, 0.0)}


def test_round_trip(tmp_path):
    path = str(tmp_path / "gains.json")
    gains_file.save(path, TUNED, (-0.005, 0.043))
    gains, trim, source = gains_file.load(path, BASE)
    assert gains == TUNED and trim == (-0.005, 0.043)
    assert "saved autotune gains" in source
    assert not (tmp_path / "gains.json.tmp").exists()


def test_save_replaces_an_existing_file(tmp_path):
    path = str(tmp_path / "gains.json")
    gains_file.save(path, BASE, (0.0, 0.0))
    gains_file.save(path, TUNED, (0.01, 0.02))
    assert gains_file.load(path, BASE)[0] == TUNED


def test_missing_file_gives_the_defaults(tmp_path):
    gains, trim, source = gains_file.load(str(tmp_path / "nope.json"), BASE, (0.01, 0.0))
    assert gains == BASE and trim == (0.01, 0.0) and "no " in source


def test_bad_files_give_the_defaults_and_say_why(tmp_path):
    good = {"gains": {n: list(TUNED[n]) for n in TUNED}, "trim": [0.0, 0.0]}
    cases = {
        "half written": '{"gains": {"roll": [0.002',
        "missing axis": json.dumps({"gains": {"roll": [0.004, 0.002, 0.00006]}, "trim": [0, 0]}),
        "wild gain": json.dumps(dict(good, gains=dict(good["gains"], roll=[0.4, 0.002, 0.00006]))),
        "negative": json.dumps(dict(good, gains=dict(good["gains"], pitch=[-0.004, 0.002, 0.00006]))),
        "nan": json.dumps(dict(good, trim=[float("nan"), 0.0])),
        "big trim": json.dumps(dict(good, trim=[0.9, 0.0])),
        "not a dict": "[1, 2, 3]",
    }
    for what, text in cases.items():
        path = tmp_path / "gains.json"
        path.write_text(text)
        gains, trim, source = gains_file.load(str(path), BASE)
        assert gains == BASE and trim == (0.0, 0.0), what
        assert source.startswith("defaults"), what
