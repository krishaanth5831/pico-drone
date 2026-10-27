"""
The ground-station link (src/drivers/gcs_link.py) and the vehicle's side of
the conversation (src/flight/ground_station.py), over real UDP on localhost.
"""

import time

from drivers.gcs_link import Link, decode, encode, rounded
from flight.autotune_session import IDLE, SETTLING, AutotuneSession
from flight.ground_station import GroundStation

BASE = {"roll": (0.0040, 0.0020, 0.00006), "pitch": (0.0040, 0.0020, 0.00006),
        "yaw": (0.0050, 0.0010, 0.0)}
STATE = {"armed": True, "airborne": True, "att": [1.234, -0.5, 10.0], "rates": [0.1, 0.2, 0.3],
         "thr": 0.6, "out": [0.6, 0.6, 0.6, 0.6], "hz": 100.0}


def test_encode_decode_round_trip():
    msg = {"t": "cmd", "c": "at_start", "amp": 0.06, "yaw": True}
    assert decode(encode(msg)) == msg


def test_decode_drops_garbage():
    for junk in (b"", b"not json", b"[1, 2]", b'{"no_type": 1}', b"\xff\xfe"):
        assert decode(junk) is None


def test_rounded_reaches_inside_containers():
    assert rounded({"a": [1.23456, (2.98765,)], "b": "x", "c": 3}, 2) == {"a": [1.23, [2.99]], "b": "x", "c": 3}


def pair():
    vehicle = Link(port=0, host="127.0.0.1")
    gcs = Link(port=0, peer=("127.0.0.1", vehicle.port), host="127.0.0.1")
    return vehicle, gcs


def receive(link, now=0.0, want=1, timeout=1.0):
    """Poll until `want` messages arrive (UDP on localhost is quick but not instant)."""
    got = []
    end = time.monotonic() + timeout
    while len(got) < want and time.monotonic() < end:
        got += link.poll(now)
        time.sleep(0.002)
    return got


def test_vehicle_answers_whoever_talked_last():
    vehicle, gcs = pair()
    try:
        assert not vehicle.send({"t": "tel"})      # nobody to send to yet
        gcs.send({"t": "hb"})
        assert receive(vehicle) == [{"t": "hb"}]
        assert vehicle.peer[1] == gcs.port
        assert vehicle.send({"t": "tel", "x": 1})
        assert receive(gcs) == [{"t": "tel", "x": 1}]
    finally:
        vehicle.close()
        gcs.close()


def test_alive_times_out():
    vehicle, gcs = pair()
    try:
        assert not vehicle.alive(0.0)
        gcs.send({"t": "hb"})
        receive(vehicle, now=10.0)
        assert vehicle.alive(11.0)
        assert not vehicle.alive(11.6)
    finally:
        vehicle.close()
        gcs.close()


def station():
    vehicle, gcs = pair()
    gs = GroundStation(vehicle, AutotuneSession(BASE), vehicle="sim")
    gcs.send({"t": "hb"})
    receive(vehicle)
    return gs, gcs


def drain(gs, gcs, now, state=STATE, want=1):
    """Service the vehicle once, then collect what the ground station got."""
    actions = gs.service(now, state)
    return actions, receive(gcs, want=want, timeout=0.3)


def by_type(msgs, t):
    return [m for m in msgs if m["t"] == t]


def test_start_is_acked_and_telemetry_flows():
    gs, gcs = station()
    try:
        gcs.send({"t": "cmd", "c": "at_start", "amp": 0.05, "yaw": False})
        time.sleep(0.02)
        _, msgs = drain(gs, gcs, 0.0, want=4)
        ack = by_type(msgs, "ack")[0]
        assert ack == {"t": "ack", "c": "at_start", "ok": True, "why": None}
        assert gs.session.state == SETTLING
        tel = by_type(msgs, "tel")[0]
        assert tel["veh"] == "sim" and tel["st"] == SETTLING and tel["att"] == [1.23, -0.5, 10.0]
        assert by_type(msgs, "txt")                      # "started ..."
        assert by_type(msgs, "gains")[0]["gains"]["roll"] == list(BASE["roll"])
    finally:
        gs.link.close()
        gcs.close()


def test_refuses_when_the_vehicle_cant_tune():
    gs, gcs = station()
    try:
        gcs.send({"t": "cmd", "c": "at_start"})
        time.sleep(0.02)
        _, msgs = drain(gs, gcs, 0.0, dict(STATE, cant_tune="wrong controller"), want=3)
        ack = by_type(msgs, "ack")[0]
        assert not ack["ok"] and ack["why"] == "wrong controller"
        assert gs.session.state == IDLE
    finally:
        gs.link.close()
        gcs.close()


def test_disarm_is_an_action_and_unknown_commands_are_refused():
    gs, gcs = station()
    try:
        gcs.send({"t": "cmd", "c": "disarm"})
        gcs.send({"t": "cmd", "c": "arm"})              # there is no remote arm
        time.sleep(0.02)
        actions, msgs = drain(gs, gcs, 0.0, want=4)
        assert actions == ["disarm"]
        acks = {m["c"]: m for m in by_type(msgs, "ack")}
        assert acks["disarm"]["ok"] and not acks["arm"]["ok"]
    finally:
        gs.link.close()
        gcs.close()


def test_telemetry_is_rate_limited_and_gains_sent_once():
    gs, gcs = station()
    try:
        tels, gains = 0, 0
        for i in range(100):                           # 1 s of a 100 Hz loop
            gs.service(i * 0.01, STATE)
        msgs = receive(gcs, want=11, timeout=0.5)
        tels = len(by_type(msgs, "tel"))
        gains = len(by_type(msgs, "gains"))
        assert 9 <= tels <= 11
        assert gains == 1
        gcs.send({"t": "cmd", "c": "get_gains"})        # asking again sends them again
        time.sleep(0.02)
        _, msgs = drain(gs, gcs, 1.0, want=2)
        assert len(by_type(msgs, "gains")) == 1
    finally:
        gs.link.close()
        gcs.close()


def test_result_is_sent_once():
    gs, gcs = station()
    try:
        gs.session.result = {"axes": {}, "trim_old": [0, 0], "trim_new": [0, 0]}
        gs.service(0.0, STATE)
        gs.service(0.05, STATE)
        msgs = receive(gcs, want=3, timeout=0.3)
        assert len(by_type(msgs, "res")) == 1
    finally:
        gs.link.close()
        gcs.close()
