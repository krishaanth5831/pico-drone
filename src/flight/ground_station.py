"""
The vehicle's side of the ground-station conversation: telemetry out,
commands in, the autotune session in between. firmware/autotune.py (the
Pico) and sim/sitl.py (the simulated drone) both run this, so gcs/gcs.py
can't tell them apart except by the "veh" field.

Messages, vehicle -> ground station:
  tel    10 Hz: state, progress, armed/airborne, attitude, rates, throttle,
         motor outputs, loop rate + worst loop time (and height / battery if the
         vehicle knows them)
  txt    a status text, like PX4's "[autotune] ..." messages
  ack    the answer to a command: ok, or why not
  res    the autotune result: per axis K, lag, R^2, old/new gains, the model
  gains  the gains + trim flying now (on request, and whenever they change)

ground station -> vehicle:
  hb     heartbeat, 5 Hz. No message for LINK_TIMEOUT_S = link lost: a running
         autotune aborts and the old gains keep flying.
  cmd    {"c": ...}:
           at_start  {"amp": 0.06, "yaw": true}  start the autotune
           at_abort                               stop it, old gains
           accept    {"when": "now" | "landing"}  take the result
           reject                                 drop it
           get_gains
           disarm                                 stop the motors NOW

There is deliberately no "arm" command: arming stays at the board's own
prompt (Thonny), where someone is looking at the drone.
"""

from drivers.gcs_link import rounded

TEL_HZ = 10.0


class GroundStation:
    def __init__(self, link, session, vehicle="pico"):
        self.link = link
        self.session = session
        self.vehicle = vehicle
        self._last_tel = None
        self._sent_result = None
        self._sent_version = None

    def link_ok(self, now):
        return self.link.alive(now)

    def service(self, now, state):
        """
        Call once per control loop (it never blocks). `state`: the vehicle's
        current view - armed, airborne, att (roll, pitch, yaw), rates, thr,
        out [m1..m4], hz, and optionally z, bat, and `cant_tune` (a reason
        the autotune can't start right now, e.g. the wrong controller).

        Returns a list of actions for the vehicle: "disarm".
        """
        actions = []
        s = self.session
        for msg in self.link.poll(now):
            if msg.get("t") != "cmd":
                continue                       # heartbeats only keep the link alive
            c = msg.get("c")
            ok, why = True, None
            if c == "at_start":
                if state.get("cant_tune"):
                    ok, why = False, state["cant_tune"]
                else:
                    ok, why = s.start(now, state.get("airborne", False),
                                      amplitude=float(msg.get("amp", 0.06)),
                                      include_yaw=bool(msg.get("yaw", True)))
            elif c == "at_abort":
                if s.running:
                    s.abort("stopped from the ground station")
                else:
                    ok, why = False, "not running"
            elif c == "accept":
                ok, why = s.accept(now, msg.get("when", "landing"))
            elif c == "reject":
                ok, why = s.reject()
            elif c == "get_gains":
                self._sent_version = None
            elif c == "disarm":
                actions.append("disarm")
            else:
                ok, why = False, "unknown command %r" % c
            self.link.send({"t": "ack", "c": c, "ok": ok, "why": why})

        for severity, text in s.pop_texts():
            self.link.send({"t": "txt", "sev": severity, "msg": text})

        # Each sent once (send() is False when there's nobody to send to yet).
        if s.result is not None and s.result is not self._sent_result \
                and self.link.send({"t": "res", "res": rounded(s.result, 5)}):
            self._sent_result = s.result
        if s.version != self._sent_version and self.link.send(
                {"t": "gains", "gains": rounded(s.gains, 7), "trim": rounded(s.trim, 4), "base": rounded(s.base, 7)}):
            self._sent_version = s.version

        if self._last_tel is None or now - self._last_tel >= 1.0 / TEL_HZ:
            self._last_tel = now
            tel = {"t": "tel", "veh": self.vehicle, "st": s.state, "pg": rounded(s.progress(), 3),
                   "why": s.reason}
            for key in ("armed", "airborne", "att", "rates", "thr", "out", "hz", "dtmax", "z", "bat"):
                if key in state:
                    tel[key] = rounded(state[key], 2)
            self.link.send(tel)
        return actions
