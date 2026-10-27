"""
pico GCS - a small ground station in the spirit of QGroundControl, built
around the PX4-style autotune (Vehicle Setup > PID Tuning > Autotune in QGC).

    /usr/bin/python3 gcs/gcs.py                          # the simulated drone (sim/sitl.py)
    /usr/bin/python3 gcs/gcs.py --vehicle 192.168.4.1    # the Pico on its WiFi (firmware/autotune.py)

Standard library only (tkinter: `sudo apt install python3-tk` if it's
missing). Use the system Python: miniforge's and the sim's uv venv's bundled
Tk have no font rendering beyond X11 bitmap fonts, so they work but look
rough. What it shows:

  left    the vehicle: artificial horizon, gyro rates (10 s), motor outputs,
          the rate gains + trim it is flying right now
  right   the autotune: amplitude (PX4's MC_AT_SYSID_AMP), include yaw,
          Start / Abort, per-axis progress, the result (fit quality, how much
          authority this drone has vs the reference, old -> new gains), the
          predicted step response before vs after, and Accept (now / after
          landing) or Reject
  bottom  status messages from the vehicle and the answers to commands
  top     link status and a DISARM button. DISARM stops the motors in
          software over WiFi: it is NOT a kill switch. Keep the hardware
          switch in reach.

There is no arm button: arming stays at the board's own prompt (Thonny),
next to the drone.

The link: JSON over UDP (src/drivers/gcs_link.py). This app listens on
14550 like QGC and talks to the vehicle on 14555; a heartbeat goes out 5x a
second, and the vehicle aborts a running autotune if it stops hearing them.
"""

import argparse
import math
import subprocess
import sys
import time
import tkinter as tk
import tkinter.font as tkfont
from collections import deque
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from drivers.gcs_link import GCS_PORT, VEHICLE_PORT, Link  # noqa: E402
from flight.step_response import STEP_DPS, metrics, step_response  # noqa: E402

AXES = ("roll", "pitch", "yaw")
RUNNING = ("settling", "roll", "pitch", "yaw", "verify")
HEARTBEAT_S = 0.2
HISTORY_S = 10.0
AMP_DEFAULT, AMP_MIN, AMP_MAX = 0.06, 0.02, 0.12

# --- look: dark, QGC-like ------------------------------------------------------------------
BG = "#15171b"
PANEL = "#1e2126"
RAISED = "#282c33"
LINE = "#353a42"
TEXT = "#e8eaed"
MUTED = "#8f969e"
ACCENT = "#4ea1ff"
OK = "#35c27a"
WARN = "#f2b33d"
BAD = "#ef5350"
SKY, GROUND = "#24507e", "#5c4426"
AXIS_COLOUR = {"roll": "#4ea1ff", "pitch": "#f2b33d", "yaw": "#b07cff"}
STATE_TEXT = {
    "idle": ("Ready", MUTED), "settling": ("Waiting for a steady hover", ACCENT),
    "roll": ("Tuning roll", ACCENT), "pitch": ("Tuning pitch", ACCENT), "yaw": ("Tuning yaw", ACCENT),
    "result": ("Finished - review the result", OK), "verify": ("New gains flying - checking for wobble", ACCENT),
    "staged": ("Accepted - applies when it lands", OK), "applied": ("Applied and saved", OK),
    "rejected": ("Rejected - old gains kept", MUTED), "failed": ("Stopped", WARN),
}


def pick_fonts(root):
    have = set(tkfont.families(root))
    sans = next((f for f in ("Inter", "Roboto", "DejaVu Sans") if f in have), "TkDefaultFont")
    mono = next((f for f in ("JetBrains Mono", "DejaVu Sans Mono") if f in have), "TkFixedFont")
    return {
        "body": (sans, 10), "small": (sans, 9), "bold": (sans, 10, "bold"), "title": (sans, 12, "bold"),
        "big": (sans, 15, "bold"), "mono": (mono, 9), "mono_b": (mono, 10, "bold"),
    }


def open_link(port, vehicle):
    """Listen on 14550 like QGC; if something else has it (QGC itself?), any free port."""
    try:
        link = Link(port=port, peer=vehicle)
    except OSError:
        link = Link(port=0, peer=vehicle)
    return link


class Gcs:
    def __init__(self, root, args):
        self.root = root
        self.args = args
        self.f = pick_fonts(root)
        self.link = open_link(args.port, (args.vehicle, args.vehicle_port))
        self.tel = None
        self.gains = None
        self.res = None
        self.responses = {}           # axis -> (times, before, after)
        self.run_axes = AXES
        self.rates = deque()          # (time, roll, pitch, yaw) for the strip chart
        self.last_state = None
        self.show_axis = tk.StringVar(value="roll")
        self.amp = tk.DoubleVar(value=AMP_DEFAULT)
        self.include_yaw = tk.BooleanVar(value=True)
        self.when = tk.StringVar(value="now")
        self.build()
        self.log("listening on UDP %d, talking to %s:%d" % (self.link.port, args.vehicle, args.vehicle_port), "muted")
        if args.vehicle == "127.0.0.1":
            self.log("simulated drone: run  cd sim && uv run python sitl.py", "muted")
        self.root.after(10, self.tick_net)
        self.root.after(10, self.tick_heartbeat)
        self.root.after(50, self.tick_ui)
        if args.screenshot:
            self.root.after(int(args.screenshot_after * 1000), self.screenshot)

    # --- widgets ---------------------------------------------------------------------------
    def label(self, parent, text="", font="body", fg=TEXT, bg=PANEL, **kw):
        return tk.Label(parent, text=text, font=self.f[font], fg=fg, bg=bg, **kw)

    def button(self, parent, text, command, kind="normal"):
        bg, fg, hover = {"normal": (RAISED, TEXT, "#343943"), "primary": (ACCENT, "#08111d", "#74b6ff"),
                         "danger": (BAD, "#ffffff", "#ff7472"), "ok": (OK, "#06140c", "#5ad597")}[kind]
        b = tk.Button(parent, text=text, command=command, font=self.f["bold"], bg=bg, fg=fg,
                      activebackground=hover, activeforeground=fg, disabledforeground="#5d636b",
                      relief="flat", bd=0, highlightthickness=0, padx=16, pady=7, cursor="hand2")
        b.colours = (bg, fg)
        return b

    @staticmethod
    def enable(b, on):
        if on:
            b.configure(state="normal", bg=b.colours[0], cursor="hand2")
        else:
            b.configure(state="disabled", bg=RAISED, cursor="arrow")

    def panel(self, parent, title):
        outer = tk.Frame(parent, bg=PANEL, highlightthickness=1, highlightbackground=LINE)
        self.label(outer, title, "title").pack(anchor="w", padx=16, pady=(12, 6))
        return outer

    def build(self):
        r = self.root
        r.title("pico GCS")
        r.configure(bg=BG)
        r.minsize(1180, 800)
        r.columnconfigure(1, weight=1)
        r.rowconfigure(1, weight=1)

        # Top bar: link, vehicle, DISARM.
        top = tk.Frame(r, bg=PANEL, highlightthickness=1, highlightbackground=LINE)
        top.grid(row=0, column=0, columnspan=2, sticky="ew")
        self.label(top, "pico GCS", "big").pack(side="left", padx=(16, 20), pady=10)
        self.label(top, "Vehicle", fg=MUTED).pack(side="left")
        self.ip = tk.Entry(top, width=14, font=self.f["mono"], bg=RAISED, fg=TEXT, insertbackground=TEXT,
                           relief="flat", highlightthickness=1, highlightbackground=LINE, highlightcolor=ACCENT)
        self.ip.insert(0, self.args.vehicle)
        self.ip.pack(side="left", padx=8, ipady=4)
        self.button(top, "Connect", self.reconnect).pack(side="left")
        self.link_dot = tk.Canvas(top, width=12, height=12, bg=PANEL, highlightthickness=0)
        self.link_dot.pack(side="left", padx=(18, 6))
        self.link_text = self.label(top, "no link")
        self.link_text.pack(side="left")
        self.chips = self.label(top, "", fg=MUTED)
        self.chips.pack(side="left", padx=18)
        self.disarm_btn = self.button(top, "DISARM", self.disarm, "danger")
        self.disarm_btn.pack(side="right", padx=16, pady=8)

        # Left: the vehicle.
        left = self.panel(r, "Vehicle")
        left.grid(row=1, column=0, sticky="nsew", padx=(12, 6), pady=12)
        row = tk.Frame(left, bg=PANEL)
        row.pack(fill="x", padx=16)
        self.horizon = tk.Canvas(row, width=200, height=200, bg=PANEL, highlightthickness=0)
        self.horizon.pack(side="left")
        self.motors = tk.Canvas(row, width=200, height=210, bg=PANEL, highlightthickness=0)
        self.motors.pack(side="left", padx=(16, 0))
        self.label(left, "Gyro rates, last 10 s (deg/s)", "small", MUTED).pack(anchor="w", padx=16, pady=(14, 2))
        self.strip = tk.Canvas(left, width=420, height=150, bg=BG, highlightthickness=0)
        self.strip.pack(padx=16)
        self.label(left, "Flying now", "bold").pack(anchor="w", padx=16, pady=(14, 4))
        self.now_gains = self.label(left, "waiting for the vehicle...", "mono", MUTED, justify="left")
        self.now_gains.pack(anchor="w", padx=16, pady=(0, 14))

        # Right: autotune.
        right = self.panel(r, "PID Tuning  ·  Autotune")
        right.grid(row=1, column=1, sticky="nsew", padx=(6, 12), pady=12)
        self.label(right, "Hover steadily, hands off the sticks. The vehicle wiggles roll, pitch and yaw in "
                          "turn (about 25 s), measures how it responds, and proposes rate gains and trim. "
                          "Nothing changes until you accept.", "small", MUTED, wraplength=640,
                   justify="left").pack(anchor="w", padx=16)

        settings = tk.Frame(right, bg=PANEL)
        settings.pack(fill="x", padx=16, pady=(12, 4))
        self.label(settings, "Amplitude").pack(side="left")
        tk.Scale(settings, variable=self.amp, from_=AMP_MIN, to=AMP_MAX, resolution=0.01, orient="horizontal",
                 length=180, showvalue=False, bg=ACCENT, fg=TEXT, troughcolor=RAISED, activebackground="#74b6ff",
                 width=10, sliderlength=18, highlightthickness=0, bd=0, sliderrelief="flat",
                 command=lambda _v: self.show_amp()
                 ).pack(side="left", padx=8)
        self.amp_text = self.label(settings, "", "mono")
        self.amp_text.pack(side="left")
        self.show_amp()
        tk.Checkbutton(settings, text="Include yaw", variable=self.include_yaw, font=self.f["body"], bg=PANEL,
                       fg=TEXT, selectcolor=RAISED, activebackground=PANEL, activeforeground=TEXT,
                       highlightthickness=0, bd=0).pack(side="left", padx=(24, 0))

        go = tk.Frame(right, bg=PANEL)
        go.pack(fill="x", padx=16, pady=(8, 6))
        self.start_btn = self.button(go, "Start autotune", self.start, "primary")
        self.start_btn.pack(side="left")
        self.abort_btn = self.button(go, "Abort", self.abort)
        self.abort_btn.pack(side="left", padx=8)
        self.state_text = self.label(go, "", "bold", MUTED)
        self.state_text.pack(side="left", padx=12)
        self.reason_text = self.label(right, "", "small", WARN, wraplength=640, justify="left")
        self.reason_text.pack(anchor="w", padx=16)

        prog = tk.Frame(right, bg=PANEL)
        prog.pack(fill="x", padx=16, pady=(4, 8))
        self.bars = {}
        for i, name in enumerate(AXES):
            self.label(prog, name.capitalize(), fg=AXIS_COLOUR[name]).grid(row=i, column=0, sticky="w", pady=2)
            c = tk.Canvas(prog, width=440, height=8, bg=PANEL, highlightthickness=0)
            c.grid(row=i, column=1, padx=10)
            t = self.label(prog, "", "mono", MUTED, width=9, anchor="w")
            t.grid(row=i, column=2, sticky="w")
            self.bars[name] = (c, t)

        tk.Frame(right, bg=LINE, height=1).pack(fill="x", padx=16, pady=6)
        self.label(right, "Result", "bold").pack(anchor="w", padx=16)
        self.table = tk.Frame(right, bg=PANEL)
        self.table.pack(fill="x", padx=16, pady=(4, 2))
        self.trim_text = self.label(right, "", "mono", MUTED)
        self.trim_text.pack(anchor="w", padx=16)

        head = tk.Frame(right, bg=PANEL)
        head.pack(fill="x", padx=16, pady=(10, 2))
        self.label(head, "Predicted response to a %.0f deg/s step" % STEP_DPS, "small", MUTED).pack(side="left")
        for name in AXES:
            tk.Radiobutton(head, text=name, value=name, variable=self.show_axis, command=self.draw_response,
                           indicatoron=False, font=self.f["small"], bg=RAISED, fg=TEXT, selectcolor=ACCENT,
                           activebackground="#343943", activeforeground=TEXT, relief="flat", bd=0,
                           highlightthickness=0, padx=10, pady=2).pack(side="right", padx=1)
        self.response = tk.Canvas(right, width=640, height=200, bg=BG, highlightthickness=0)
        self.response.pack(padx=16, anchor="w")

        apply = tk.Frame(right, bg=PANEL)
        apply.pack(fill="x", padx=16, pady=(10, 14))
        self.label(apply, "Apply").pack(side="left")
        self.when_buttons = {}
        for value, text in (("now", "now (checked in flight for 3 s)"), ("landing", "after landing")):
            b = tk.Radiobutton(apply, text=text, value=value, variable=self.when, font=self.f["body"], bg=PANEL,
                               fg=TEXT, selectcolor=RAISED, activebackground=PANEL, activeforeground=TEXT,
                               highlightthickness=0, bd=0)
            b.pack(side="left", padx=(10, 0))
            self.when_buttons[value] = b
        self.reject_btn = self.button(apply, "Reject", self.reject)
        self.reject_btn.pack(side="right")
        self.accept_btn = self.button(apply, "Accept", self.accept, "ok")
        self.accept_btn.pack(side="right", padx=8)

        # Bottom: messages.
        self.messages = tk.Text(r, height=7, bg=PANEL, fg=TEXT, font=self.f["mono"], relief="flat",
                                highlightthickness=1, highlightbackground=LINE, padx=12, pady=8, wrap="word")
        self.messages.grid(row=2, column=0, columnspan=2, sticky="ew", padx=12, pady=(0, 12))
        for tag, colour in (("info", TEXT), ("warn", WARN), ("error", BAD), ("ok", OK), ("muted", MUTED)):
            self.messages.tag_configure(tag, foreground=colour)
        self.messages.configure(state="disabled")

    def show_amp(self):
        self.amp_text.configure(text="%.2f  (MC_AT_SYSID_AMP)" % self.amp.get())

    def log(self, text, tag="info"):
        m = self.messages
        m.configure(state="normal")
        m.insert("end", "%s  %s\n" % (time.strftime("%H:%M:%S"), text), tag)
        m.see("end")
        m.configure(state="disabled")

    # --- talking to the vehicle -----------------------------------------------------------
    def connected(self):
        return self.link.alive(time.monotonic())

    def send(self, c, **kw):
        self.link.send(dict({"t": "cmd", "c": c}, **kw))

    def reconnect(self):
        ip = self.ip.get().strip()
        self.link.peer = (ip, self.args.vehicle_port)
        self.log("talking to %s:%d" % self.link.peer, "muted")
        self.send("get_gains")

    def start(self):
        self.run_axes = AXES if self.include_yaw.get() else ("roll", "pitch")
        self.send("at_start", amp=round(self.amp.get(), 3), yaw=self.include_yaw.get())

    def abort(self):
        self.send("at_abort")

    def accept(self):
        self.send("accept", when=self.when.get())

    def reject(self):
        self.send("reject")

    def disarm(self):
        for _ in range(3):              # UDP: say it more than once
            self.send("disarm")
        self.log("DISARM sent. This is software over WiFi, not a kill switch.", "warn")

    def tick_heartbeat(self):
        self.link.send({"t": "hb"})
        self.root.after(int(HEARTBEAT_S * 1000), self.tick_heartbeat)

    def tick_net(self):
        was = self.link.last_rx is not None
        for msg in self.link.poll(time.monotonic()):
            self.handle(msg)
        if not was and self.link.last_rx is not None:
            self.send("get_gains")
        self.root.after(20, self.tick_net)

    def handle(self, msg):
        t = msg["t"]
        if t == "tel":
            self.tel = msg
            now = time.monotonic()
            self.rates.append((now, *(msg.get("rates") or (0.0, 0.0, 0.0))))
            while self.rates and now - self.rates[0][0] > HISTORY_S:
                self.rates.popleft()
            st = msg.get("st")
            if st != self.last_state:
                if st == "settling":          # a new run: forget the last one
                    self.res, self.responses = None, {}
                    self.draw_result()
                self.last_state = st
        elif t == "txt":
            sev = msg.get("sev", "info")
            self.log(msg.get("msg", ""), sev if sev in ("warn", "error") else "info")
        elif t == "ack":
            if not msg.get("ok"):
                self.log("%s refused: %s" % (msg.get("c"), msg.get("why")), "warn")
        elif t == "gains":
            self.gains = msg
            self.draw_gains()
        elif t == "res":
            self.res = msg["res"]
            self.run_axes = tuple(n for n in AXES if self.res["axes"].get(n, {}).get("why") != "skipped")
            self.compute_responses()
            self.draw_result()

    # --- result maths ---------------------------------------------------------------------
    def compute_responses(self):
        self.responses = {}
        for name in AXES:
            a = self.res["axes"].get(name, {})
            if a.get("model"):
                t, before = step_response(a["model"], a["old"])
                _, after = step_response(a["model"], a["new"])
                self.responses[name] = (t, before, after)
        first = next((n for n in AXES if n in self.responses and self.res["axes"][n]["ok"]), None)
        if first:
            self.show_axis.set(first)

    # --- drawing --------------------------------------------------------------------------
    def tick_ui(self):
        self.draw_link()
        self.draw_horizon()
        self.draw_motors()
        self.draw_strip()
        self.draw_progress()
        self.root.after(100, self.tick_ui)

    def draw_link(self):
        ok = self.connected()
        tel = self.tel if ok else None
        d = self.link_dot
        d.delete("all")
        d.create_oval(1, 1, 11, 11, fill=OK if ok else BAD, outline="")
        self.link_text.configure(text=("connected  %s:%d" % self.link.peer) if ok else "no link")
        chips = []
        if tel:
            chips.append("SITL (sim)" if tel.get("veh") == "sim" else "Pico")
            chips.append("ARMED" if tel.get("armed") else "disarmed")
            chips.append("flying" if tel.get("airborne") else "on the ground")
            if tel.get("hz"):
                worst = tel.get("dtmax")
                chips.append("%.0f Hz loop%s" % (tel["hz"], " (worst %.0f ms)" % worst if worst else ""))
            if tel.get("z") is not None:
                chips.append("%.2f m" % tel["z"])
            if tel.get("bat") is not None:
                chips.append("%.2f V" % tel["bat"])
        self.chips.configure(text="   ·   ".join(chips))
        # The board can't tell landed from hovering (no altitude sensor), so
        # there it is PX4's "apply on disarm".
        pico = bool(tel) and tel.get("veh") == "pico"
        self.when_buttons["landing"].configure(text="on disarm" if pico else "after landing")
        st = tel.get("st") if tel else None
        running = st in RUNNING
        self.enable(self.start_btn, bool(tel) and st not in RUNNING + ("result", "staged"))
        self.enable(self.abort_btn, bool(tel) and running)
        self.enable(self.accept_btn, bool(tel) and st == "result")
        self.enable(self.reject_btn, bool(tel) and st == "result")
        self.enable(self.disarm_btn, ok)
        text, colour = STATE_TEXT.get(st, ("Waiting for the vehicle", MUTED)) if tel else ("No vehicle", MUTED)
        self.state_text.configure(text=text, fg=colour)
        why = tel.get("why") if tel and st == "failed" else None
        self.reason_text.configure(text=("Why: %s" % why) if why else "")

    def draw_horizon(self):
        c = self.horizon
        c.delete("all")
        w = h = 200
        cx, cy = w / 2, h / 2
        roll, pitch = (self.tel.get("att") or (0.0, 0.0, 0.0))[:2] if self.tel else (0.0, 0.0)
        px_per_deg = 3.0
        # Nose down (pitch > 0) shows more ground: the horizon moves up.
        # Rolled right, the horizon tilts the other way on screen.
        a = math.radians(-roll)
        ca, sa = math.cos(a), math.sin(a)

        def pt(x, y):        # horizon frame -> screen
            y -= pitch * px_per_deg
            return cx + x * ca - y * sa, cy + x * sa + y * ca

        c.create_rectangle(0, 0, w, h, fill=SKY, outline="")
        ground = [pt(-400, 0), pt(400, 0), pt(400, 400), pt(-400, 400)]
        c.create_polygon(*[v for p in ground for v in p], fill=GROUND, outline="")
        c.create_line(*pt(-400, 0), *pt(400, 0), fill="#d9dde3", width=2)
        for deg in (-20, -10, 10, 20):
            half = 22 if deg % 20 == 0 else 14
            y = -deg * px_per_deg
            c.create_line(*pt(-half, y), *pt(half, y), fill="#d9dde3")
        # The aircraft: fixed in the middle.
        c.create_line(cx - 48, cy, cx - 16, cy, cx - 8, cy + 8, fill=WARN, width=3)
        c.create_line(cx + 48, cy, cx + 16, cy, cx + 8, cy + 8, fill=WARN, width=3)
        c.create_oval(cx - 3, cy - 3, cx + 3, cy + 3, fill=WARN, outline="")
        c.create_rectangle(0, h - 24, w, h, fill="#0d0f12", outline="", stipple="gray50")
        c.create_text(cx, h - 12, text="roll %+5.1f°   pitch %+5.1f°" % (roll, pitch), fill=TEXT,
                      font=self.f["mono"])

    def draw_motors(self):
        c = self.motors
        c.delete("all")
        out = (self.tel.get("out") if self.tel else None) or [0.0] * 4
        c.create_text(4, 8, text="Motor outputs", anchor="w", fill=MUTED, font=self.f["small"])
        top, bottom = 26, 170
        for i, v in enumerate(out[:4]):
            x0 = 10 + i * 46
            c.create_rectangle(x0, top, x0 + 30, bottom, fill=RAISED, outline="")
            y = bottom - (bottom - top) * max(0.0, min(1.0, v))
            c.create_rectangle(x0, y, x0 + 30, bottom, fill=ACCENT if self.tel and self.tel.get("armed") else LINE,
                               outline="")
            c.create_text(x0 + 15, bottom + 10, text="M%d" % (i + 1), fill=MUTED, font=self.f["small"])
            c.create_text(x0 + 15, bottom + 24, text="%.2f" % v, fill=TEXT, font=self.f["mono"])

    def draw_strip(self):
        c = self.strip
        c.delete("all")
        w, h = 420, 150
        pts = list(self.rates)
        peak = max([50.0] + [abs(v) for p in pts for v in p[1:]])
        scale = (h / 2 - 8) / peak
        c.create_line(0, h / 2, w, h / 2, fill=LINE)
        c.create_text(4, 4, anchor="nw", text="±%.0f" % peak, fill=MUTED, font=self.f["small"])
        if len(pts) < 2:
            return
        now = time.monotonic()
        for k, name in enumerate(AXES):
            line = []
            for p in pts:
                line += [w - (now - p[0]) / HISTORY_S * w, h / 2 - p[k + 1] * scale]
            c.create_line(*line, fill=AXIS_COLOUR[name], width=1.5)
        for k, name in enumerate(AXES):
            c.create_text(w - 4 - (2 - k) * 46, 4, anchor="ne", text=name, fill=AXIS_COLOUR[name],
                          font=self.f["small"])

    def draw_progress(self):
        tel = self.tel
        st = tel.get("st") if tel else None
        pg = tel.get("pg", 0.0) if tel else 0.0
        n = len(self.run_axes)
        done = (pg - 0.05) / 0.9 * n if st in AXES else None   # axes completed, fractional
        for name in AXES:
            c, t = self.bars[name]
            c.delete("all")
            c.create_rectangle(0, 0, 440, 8, fill=RAISED, outline="")
            if name not in self.run_axes:
                frac, text = 0.0, "skipped"
            elif st in ("result", "verify", "staged", "applied", "rejected"):
                frac, text = 1.0, "done"
            elif done is not None:
                frac = max(0.0, min(1.0, done - self.run_axes.index(name)))
                text = "%3.0f %%" % (100 * frac)
            elif st == "settling":
                frac, text = 0.0, "waiting"
            else:
                frac, text = 0.0, ""
            if self.res and name in self.res["axes"] and not self.res["axes"][name]["ok"] and name in self.run_axes:
                text = "no fit"
            colour = WARN if text == "no fit" else AXIS_COLOUR[name]
            if frac > 0:
                c.create_rectangle(0, 0, 440 * frac, 8, fill=colour, outline="")
            t.configure(text=text)

    def draw_gains(self):
        g = self.gains
        lines = ["        P        I        D        x default"]
        for name in AXES:
            kp, ki, kd = g["gains"][name]
            base = g.get("base", {}).get(name)
            scale = "  x%.2f" % (kp / base[0]) if base and base[0] else ""
            lines.append("%-6s  %.4f   %.4f   %.5f%s" % (name, kp, ki, kd, scale))
        lines.append("trim    roll %+.3f   pitch %+.3f" % tuple(g["trim"]))
        self.now_gains.configure(text="\n".join(lines), fg=TEXT)

    def draw_result(self):
        for w in self.table.winfo_children():
            w.destroy()
        res = self.res
        if not res:
            self.trim_text.configure(text="Start the autotune to see a result here.")
            self.draw_response()
            return
        heads = ("Axis", "Fit R²", "Authority vs reference", "Lag", "Rate P · I · D  before → after", "")
        for j, text in enumerate(heads):
            self.label(self.table, text, "small", MUTED).grid(row=0, column=j, sticky="w", padx=(0, 18))
        for i, name in enumerate(AXES, start=1):
            a = res["axes"].get(name, {})
            cells = [(name.capitalize(), "bold", AXIS_COLOUR[name])]
            if a.get("why") == "skipped":
                cells += [("skipped", "small", MUTED)]
            else:
                r2 = a.get("r2")
                cells.append(("%.2f" % r2 if r2 is not None else "-", "mono", OK if a.get("ok") else WARN))
                if a.get("K"):
                    cells.append(("%.0f / %.0f  (%.0f %%)" % (a["K"], a["K_ref"], 100 * a["K"] / a["K_ref"]),
                                  "mono", TEXT))
                    cells.append(("%.0f ms" % a["lag_ms"], "mono", TEXT))
                else:
                    cells += [("-", "mono", MUTED), ("-", "mono", MUTED)]
                if a.get("ok"):
                    o, n = a["old"], a["new"]
                    cells.append(("%.4f %.4f %.5f  →  %.4f %.4f %.5f" % (*o, *n), "mono", TEXT))
                    cells.append(("x%.2f" % a["scale"], "mono_b", ACCENT))
                else:
                    cells.append(("kept: %s" % a.get("why"), "small", WARN))
            for j, (text, font, colour) in enumerate(cells):
                self.label(self.table, text, font, colour).grid(row=i, column=j, sticky="w", padx=(0, 18), pady=1)
        self.trim_text.configure(text="Trim   roll %+.3f → %+.3f     pitch %+.3f → %+.3f   (mixer units)"
                                 % (res["trim_old"][0], res["trim_new"][0], res["trim_old"][1], res["trim_new"][1]))
        self.draw_response()

    def draw_response(self):
        c = self.response
        c.delete("all")
        w, h = 640, 200
        left, right, top, bottom = 44, 200, 12, h - 26
        pw = w - left - right
        name = self.show_axis.get()
        c.create_rectangle(left, top, left + pw, bottom, outline=LINE)
        if name not in self.responses:
            msg = "no model for %s yet" % name
            if self.res and self.res["axes"].get(name, {}).get("why"):
                msg = "%s: %s" % (name, self.res["axes"][name]["why"])
            c.create_text(left + pw / 2, (top + bottom) / 2, text=msg, fill=MUTED, font=self.f["small"])
            return
        t, before, after = self.responses[name]
        ymax = max(130.0, max(before + after) * 1.05)

        def xy(tt, v):
            return left + tt / t[-1] * pw, bottom - v / ymax * (bottom - top)

        for v in (0, 50, 100):
            y = xy(0, v)[1]
            c.create_line(left, y, left + pw, y, fill=LINE, dash=(2, 4) if v else ())
            c.create_text(left - 6, y, text="%d" % v, anchor="e", fill=MUTED, font=self.f["small"])
        for ms in range(0, int(t[-1] * 1000) + 1, 100):
            x = xy(ms / 1000.0, 0)[0]
            c.create_text(x, bottom + 12, text="%d ms" % ms, fill=MUTED, font=self.f["small"])
        c.create_line(*xy(0, STEP_DPS), *xy(t[-1], STEP_DPS), fill=TEXT, dash=(6, 4))
        for series, colour, width in ((before, MUTED, 1.5), (after, AXIS_COLOUR[name], 2.5)):
            c.create_line(*[v for k in range(len(t)) for v in xy(t[k], series[k])], fill=colour, width=width)
        # Legend with rise time and overshoot.
        x0 = left + pw + 18
        ok = self.res["axes"][name]["ok"]
        for k, (label, series, colour) in enumerate((("before", before, MUTED),
                                                     ("after" if ok else "after (unchanged)", after,
                                                      AXIS_COLOUR[name]))):
            rise, over = metrics(t, series)
            y = top + 8 + k * 62
            c.create_line(x0, y, x0 + 22, y, fill=colour, width=2.5)
            c.create_text(x0 + 30, y, text=label, anchor="w", fill=TEXT, font=self.f["bold"])
            c.create_text(x0 + 30, y + 18, anchor="w", fill=MUTED, font=self.f["mono"],
                          text="rise      %s" % ("%.0f ms" % (rise * 1000) if rise else "-"))
            c.create_text(x0 + 30, y + 34, anchor="w", fill=MUTED, font=self.f["mono"],
                          text="overshoot %.0f %%" % over)
        c.create_text(x0, bottom - 4, anchor="sw", fill=MUTED, font=self.f["small"], width=180,
                      text="From the fitted model: shape, not a promise (no motor limits).")

    # --- docs helper ----------------------------------------------------------------------
    def screenshot(self):
        """--screenshot: save this window with ImageMagick's `import` (for the README)."""
        self.root.update()
        wid = self.root.winfo_id()
        frame = self.root.wm_frame()
        target = frame if frame and frame != "0x0" else hex(wid)
        subprocess.run(["import", "-window", target, self.args.screenshot], check=False)
        print("saved", self.args.screenshot, flush=True)
        if self.args.quit_after_screenshot:
            self.root.destroy()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--vehicle", default="127.0.0.1", help="vehicle IP: 127.0.0.1 = sim/sitl.py, "
                    "192.168.4.1 = the Pico's own WiFi")
    ap.add_argument("--vehicle-port", type=int, default=VEHICLE_PORT)
    ap.add_argument("--port", type=int, default=GCS_PORT, help="UDP port this app listens on")
    ap.add_argument("--screenshot", help=argparse.SUPPRESS)
    ap.add_argument("--screenshot-after", type=float, default=5.0, help=argparse.SUPPRESS)
    ap.add_argument("--quit-after-screenshot", action="store_true", help=argparse.SUPPRESS)
    args = ap.parse_args()
    root = tk.Tk()
    app = Gcs(root, args)
    try:
        root.mainloop()
    finally:
        app.link.close()


if __name__ == "__main__":
    main()
