"""
Ground-station link: one JSON message per UDP datagram.

The same idea as MAVLink over UDP (QGroundControl listens on 14550, the
vehicle sends to whoever last talked to it), with JSON instead of MAVLink's
binary packing so both ends stay readable and small:

  vehicle (Pico over WiFi, or sim/sitl.py)  <-- UDP -->  gcs/gcs.py
          binds VEHICLE_PORT                           binds GCS_PORT
          replies to the last sender                   sends to the vehicle's IP

Every message is a dict with a "t" (type) field; flight/ground_station.py
defines them. Non-blocking throughout: poll() returns whatever has arrived
and never waits, so it can sit inside the control loop.

Works under MicroPython (rp2, lwIP sockets) and CPython. On the board a
failed send (WiFi busy, no peer yet) is dropped silently - telemetry is sent
again 100 ms later anyway, and nothing in the control loop waits on the link.
"""

import json

try:
    import socket
except ImportError:      # a MicroPython build without networking
    socket = None

VEHICLE_PORT = 14555
GCS_PORT = 14550
LINK_TIMEOUT_S = 1.5     # no message for this long = link lost
RX_BUFFER = 4096
MAX_PER_POLL = 20        # don't let a flood of packets eat a whole control loop


def encode(msg):
    return json.dumps(msg).encode()


def decode(data):
    """A message dict, or None for anything that isn't one."""
    try:
        msg = json.loads(data)
    except (ValueError, UnicodeError):
        return None
    if not isinstance(msg, dict) or "t" not in msg:
        return None
    return msg


def rounded(x, digits=4):
    """x with every float rounded, through dicts / lists / tuples: smaller packets."""
    if isinstance(x, float):
        return round(x, digits)
    if isinstance(x, dict):
        return {k: rounded(v, digits) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [rounded(v, digits) for v in x]
    return x


class Link:
    def __init__(self, port=VEHICLE_PORT, peer=None, host="0.0.0.0"):
        """
        port: local UDP port to listen on (0 = any free one).
        peer: (ip, port) to send to. None = reply to whoever sent last
              (the vehicle side: it learns the ground station's address).
        """
        if socket is None:
            raise OSError("no socket module - this firmware has no networking")
        self.follow_peer = peer is None
        self.peer = peer
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((host, port))
        self.sock.setblocking(False)
        self.last_rx = None
        self.rx_count = 0
        self.tx_count = 0
        self.tx_errors = 0

    @property
    def port(self):
        return self.sock.getsockname()[1]

    def poll(self, now):
        """Messages received since the last poll. `now` in seconds (any clock, same as alive())."""
        msgs = []
        for _ in range(MAX_PER_POLL):
            try:
                data, addr = self.sock.recvfrom(RX_BUFFER)
            except OSError:      # EAGAIN: nothing waiting (CPython: BlockingIOError)
                break
            msg = decode(data)
            if msg is None:
                continue
            if self.follow_peer:
                self.peer = addr
            self.last_rx = now
            self.rx_count += 1
            msgs.append(msg)
        return msgs

    def send(self, msg):
        if self.peer is None:
            return False
        try:
            self.sock.sendto(encode(msg), self.peer)
        except OSError:
            self.tx_errors += 1
            return False
        self.tx_count += 1
        return True

    def alive(self, now, timeout=LINK_TIMEOUT_S):
        return self.last_rx is not None and now - self.last_rx < timeout

    def close(self):
        self.sock.close()
