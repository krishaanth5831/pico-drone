"""
firmware/autotune.py run under CPython, no board: a virtual MicroPython clock,
a fake WiFi and a fake IMU that flies like a small quad (authority K, motor
lag, an off-centre CoM), and a scripted ground station on real UDP. The
ground station's work happens once per board loop, hooked into the
gc.collect() the loop already calls.

What it proves: the loop, the safety pins (SLP low at start and after
disarm; never high in DRY), the link and session wiring, the headroom /
DRY refusals, and that accepted gains reach /gains.json only after disarm.
What it can't: anything about the real radio, I2C noise or loop timing.
"""

import builtins
import gc
import importlib.util
import json
import math
import random
import socket
import sys
import time
import types
from pathlib import Path

import machine
import pytest

import config
from drivers.gcs_link import Link
from flight.mixer import unmix

FIRMWARE = Path(__file__).resolve().parents[1] / "firmware" / "autotune.py"


class FakeIMU:
    """Reads the motor PWMs back, and answers like a quad with authority k and lag tau."""

    def __init__(self, i2c=None):
        self.i2c = i2c
        self.w = [0.0, 0.0, 0.0]
        self.dw = [0.0, 0.0, 0.0]
        self.tilt = [0.0, 0.0]
        self.k = (22000.0, 20000.0, 3000.0)
        self.twist = (0.01, -0.02, 0.0)
        self.rng = random.Random(1)

    def calibrate_gyro(self):
        pass

    def read(self):
        dt, g = 0.01, 9.81
        duty = {p.pin.id: p.duty_u16() for p in machine.PWM.instances[-4:]}
        out = {m: duty[config.MOTOR_PINS[m]] / (config.MAX_DUTY * 65535) for m in (1, 2, 3, 4)}
        u = unmix(out)[1:]
        powered = machine.Pin.instances[config.MOTOR_SLEEP_PIN].value() and out[1] > 0.3
        a = math.exp(-dt / 0.04)
        for i in range(3):
            drive = u[i] + self.twist[i] if powered else 0.0
            self.dw[i] = a * self.dw[i] + (1 - a) * self.k[i] * dt * drive
            self.w[i] += self.dw[i]
        self.tilt[0] += self.w[0] * dt
        self.tilt[1] += self.w[1] * dt
        r, p = math.radians(self.tilt[0]), math.radians(self.tilt[1])
        accel = (-g * math.sin(p), g * math.sin(r) * math.cos(p), g * math.cos(r) * math.cos(p))
        return accel, tuple(w + self.rng.gauss(0, 0.3) for w in self.w), 25.0


class GroundStation:
    """The scripted laptop side: heartbeat, start once hovering, accept, then disarm."""

    def __init__(self, port, clock, mode, when):
        self.link = Link(port=0, peer=("127.0.0.1", port), host="127.0.0.1")
        self.clock, self.mode, self.when = clock, mode, when
        self.last_hb = -1.0
        self.states, self.acks = [], []
        self.started = self.accepted = False
        self.disarms = 0

    def tick(self):
        t = self.clock["us"] / 1e6
        if t - self.last_hb >= 0.2:
            self.link.send({"t": "hb"})
            self.last_hb = t
        for m in self.link.poll(t):
            if m["t"] == "ack":
                self.acks.append((m["c"], m["ok"], m["why"]))
            elif m["t"] == "tel" and (not self.states or self.states[-1] != m["st"]):
                self.states.append(m["st"])
        state = self.states[-1] if self.states else None
        if not self.started and t > (4.0 if self.mode == "DRY" else 9.0):
            self.link.send({"t": "cmd", "c": "at_start", "amp": 0.06, "yaw": True})
            self.started = True
        if state == "result" and not self.accepted:
            self.link.send({"t": "cmd", "c": "accept", "when": self.when})
            self.accepted = True
        finished = state in ("staged", "applied") or t > 60.0
        if self.mode == "DRY" and t > 6.0:
            raise KeyboardInterrupt          # Thonny's Stop
        if self.mode == "ARM" and finished:
            self.link.send({"t": "cmd", "c": "disarm"})
            self.disarms += 1
            if self.disarms > 50:
                raise KeyboardInterrupt


def free_port():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def fly(monkeypatch, tmp_path, mode, when="landing"):
    clock = {"us": 0}

    def sleep_us(n):
        clock["us"] += int(n)

    for name, fn in (("ticks_us", lambda: clock["us"]), ("ticks_ms", lambda: clock["us"] // 1000),
                     ("ticks_diff", lambda a, b: a - b), ("ticks_add", lambda a, b: a + b),
                     ("sleep_us", sleep_us), ("sleep_ms", lambda n: sleep_us(n * 1000))):
        monkeypatch.setattr(time, name, fn, raising=False)

    network = types.ModuleType("network")

    class WLAN:
        def __init__(self, _i):
            self.on = False

        def config(self, **_kw):
            pass

        def active(self, v=None):
            if v is None:
                return self.on
            self.on = v
            return None

        def ifconfig(self):
            return ("192.168.4.1", "255.255.255.0", "", "")

    network.WLAN, network.AP_IF, network.STA_IF = WLAN, 1, 0
    monkeypatch.setitem(sys.modules, "network", network)
    monkeypatch.setattr(machine.Pin, "OPEN_DRAIN", "OPEN_DRAIN", raising=False)
    monkeypatch.setattr(machine.Pin, "instances", {})
    monkeypatch.setattr(machine.PWM, "instances", [])
    monkeypatch.setattr(builtins, "input", lambda _prompt="": mode)

    spec = importlib.util.spec_from_file_location("fw_autotune_under_test", FIRMWARE)
    fw = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(fw)
    port = free_port()
    monkeypatch.setattr(fw, "VEHICLE_PORT", port)
    monkeypatch.setattr(fw, "MPU6050", FakeIMU)
    monkeypatch.setattr(fw, "GAINS_PATH", str(tmp_path / "gains.json"))
    gcs = GroundStation(port, clock, mode, when)
    monkeypatch.setattr(gc, "collect", gcs.tick)
    try:
        fw.run()
    finally:
        gcs.link.close()
    return gcs, machine.Pin.instances[config.MOTOR_SLEEP_PIN], tmp_path / "gains.json"


def test_dry_runs_the_link_but_never_powers_the_motors_or_tunes(monkeypatch, tmp_path):
    gcs, slp, saved = fly(monkeypatch, tmp_path, "DRY")
    assert 1 not in slp.history
    assert gcs.acks and not gcs.acks[0][1] and "DRY" in gcs.acks[0][2]
    assert not saved.exists()


@pytest.mark.parametrize("when", ["landing", "now"])
def test_arm_tunes_and_saves_after_disarm(monkeypatch, tmp_path, when):
    gcs, slp, saved = fly(monkeypatch, tmp_path, "ARM", when)
    assert slp.history[0] == 0 and 1 in slp.history and slp.value() == 0
    assert gcs.states[:6] == ["idle", "settling", "roll", "pitch", "yaw", "result"]
    assert gcs.states[-1] == ("staged" if when == "landing" else "applied")
    data = json.loads(saved.read_text())
    # The fake quad is weaker than the reference (K 22000 vs ~33800): more gain.
    assert data["gains"]["roll"][0] > 0.0050
    # And the trim cancels its off-centre CoM (+0.01 roll, -0.02 pitch).
    assert abs(data["trim"][0] + 0.01) < 0.004 and abs(data["trim"][1] - 0.02) < 0.004
