"""
Fly the simulated pico-drone yourself, with the keyboard, in a room.

    cd sim
    uv run python fly.py                    # the trained policy, balloons on strings
    uv run python fly.py --ctrl pid         # lift.py's PID instead (C switches live)
    uv run python fly.py --wind --random    # gusts on, a randomized drone
    uv run python fly.py --check            # no window: scripted flight, policy vs PID

The levelling is done by the controller (the PPO policy by default), exactly
as the board runs it: noisy IMU -> the board's complementary filter -> policy
-> mixer -> lagging motors. Your keys only move its targets: the tilt it
should hold, the yaw rate, and the throttle. That is what a real stick does.

What is simulated on top of watch.py, because the real room has it:

  balloons    the weight relief is 5 helium party balloons on strings tied
              to the top of the frame, each its own body: they have drag,
              swing, collide with each other and the ceiling, and tug the
              frame through the strings. B swaps back to the relief force
              at the centre of mass the policy was trained with.
  room        4 x 4 m, 2.6 m ceiling, a table, a shelf, a landing pad. The
              frame and the balloons hit the walls.
  wind        G: a draft (a door, a fan) with gusts. It pushes the balloons
              far more than the frame - watch them drag it downwind.
  battery     a 1S LiPo going from 4.2 V down; thrust falls with voltage
              squared, so the hover throttle creeps up. V = fresh battery.
  ground      ground effect: the props push harder within a prop diameter
              of the floor, so it floats on take-off and landing.
  board       sensor noise, vibration, IMU misalignment, loop jitter, motor
              lag and the 35 deg tilt cut-out, as in training.

ALTITUDE HOLD (H) and LOITER (L) are sim-only helpers: the real drone has no
height or position sensor, so on the bench you have throttle and tilt only
(H and L off). They use the true height and position. Loiter leans against
drift (the wind, a tilt error) whenever you let go of the arrows.

Keys (click the window first):
    arrows      tilt: up = nose down (fly forward), right = roll right
    W / S       climb / descend (altitude hold) - throttle up / down (manual)
    A / D       yaw left / right
    H           altitude hold on / off (off = manual throttle, like the real one)
    L           loiter on / off (holds position when the arrows are released)
    X           cut the motors
    C           controller: policy -> lift.py PID -> none
    SPACE       kick it (a sudden knock)
    G           wind gusts on / off
    B           balloons on strings / relief lumped at the CoM (as trained)
    V           fresh battery
    N           new random drone (domain randomization sample); M = nominal again
    R           reset on the pad
    F           camera: chase / tripod in the corner
    P           pause      ESC quit
    mouse       drag = orbit the chase camera, scroll = zoom

The graph (bottom left) is the true roll and pitch against their targets for
the last 10 s, and the top-left panel gives the RMS attitude error - that is
"how stable it flies".
"""

import argparse
import math
import time
from collections import deque

import glfw
import mujoco
import numpy as np

from pico_drone_sim import airframe
from pico_drone_sim import params as P
from pico_drone_sim.controllers import ACT_SCALE, PIDController, PolicyController
from pico_drone_sim.drone import START_Z, Drone
from pico_drone_sim.env import TILT_LIMIT_DEG, HoverEnv

# isort: split
# (pico_drone_sim above puts ../src on sys.path, so `flight` imports after it)
from flight import policy_weights

# --- the room -------------------------------------------------------------------------
ROOM_HALF = 2.0          # m, the room is 4 x 4
ROOM_H = 2.6             # m, ceiling

# --- balloons ---------------------------------------------------------------------------
# A 30 cm latex party balloon lifts ~10 g net, so ~5 of them carry the
# nominal ~50 g relief. Each body's mass is what it takes to accelerate it:
# 3 g of latex + 2 g of helium + ~7 g of air it drags along (a sphere's added
# mass is half the air it displaces). Gravity on that mass is cancelled by
# buoyancy in _forces, which then adds the net lift on top.
N_BALLOONS = 5
BALLOON_R = 0.14                         # m
BALLOON_MASS = 0.012                     # kg, inertial (latex + helium + added air)
BALLOON_DRAG = 0.5 * 1.2 * 0.47 * math.pi * BALLOON_R ** 2   # 1/2 rho Cd A, N/(m/s)^2
BALLOON_SPIN_DRAG = 2e-5                 # N.m per rad/s, so they don't spin forever
STRING_L = (0.55, 0.72, 0.62, 0.80, 0.68)   # m, staggered so they bunch like a real bunch
BALLOON_RGB = ("0.9 0.15 0.2", "0.15 0.45 0.9", "0.95 0.75 0.1", "0.2 0.7 0.3", "0.65 0.25 0.8")

# --- pilot ------------------------------------------------------------------------------
MAX_TILT_DEG = 15.0      # full arrow = this much tilt target
TILT_SLEW_DPS = 90.0     # targets move toward the sticks this fast (no step inputs)
MAX_YAW_DPS = 90.0
YAW_KP = 0.0075          # policy mode: lift.py's yaw P term on top of the policy, which
                         # was only ever trained to hold yaw still
LOITER_KP = 4.0          # loiter: deg of tilt per m off the hold point...
LOITER_KD = 8.0          # ...and per m/s of drift
CLIMB_MPS = 0.5          # altitude hold: W/S move the height target this fast
THROTTLE_RATE = 0.25     # manual: W/S move the throttle this fast (per second)

# --- environment ------------------------------------------------------------------------
PROP_R = airframe.PROP_D / 2000.0
WIND_MEAN_MPS = 0.3
WIND_GUST_MPS = 0.3      # gust standard deviation
WIND_TAU_S = 2.0         # how long a gust lasts


def room_xml():
    """(assets, worldbody, extra) MJCF for the room around the drone."""
    h, c = ROOM_HALF, ROOM_H
    # conaffinity 3 on the room: the frame (contype 1) and balloons (contype 2) both hit it.
    walls = []
    for name, pos, size in (
        ("wall_n", (h, 0, c / 2), (0.02, h, c / 2)),
        ("wall_s", (-h, 0, c / 2), (0.02, h, c / 2)),
        ("wall_e", (0, -h, c / 2), (h, 0.02, c / 2)),
        ("wall_w", (0, h, c / 2), (h, 0.02, c / 2)),
    ):
        walls.append('<geom name="%s" type="box" pos="%g %g %g" size="%g %g %g" material="wall" conaffinity="3"/>'
                     % ((name,) + pos + size))
    table = (1.3, -1.45)
    legs = "\n    ".join(
        '<geom type="box" pos="%g %g 0.36" size="0.02 0.02 0.36" material="wood" conaffinity="3"/>'
        % (table[0] + sx * 0.55, table[1] + sy * 0.3) for sx in (-1, 1) for sy in (-1, 1))
    world = f"""
    <light pos="0 0 {c - 0.05}" dir="0 0 -1" diffuse="0.45 0.45 0.45" castshadow="false"/>
    <light pos="{h - 0.3} {h - 0.3} {c - 0.1}" dir="-1 -1 -1" diffuse="0.25 0.25 0.25"/>
    {"".join(walls)}
    <geom name="ceiling" type="box" pos="0 0 {c + 0.02}" size="{h} {h} 0.02" rgba="0.97 0.97 0.97 0.12"
          conaffinity="3"/>
    <geom name="pad" type="cylinder" pos="0 0 0.0005" size="0.13 0.0005" rgba="0.95 0.5 0.1 1"
          contype="0" conaffinity="0"/>
    <geom name="pad_h" type="box" pos="0 0 0.0012" size="0.06 0.012 0.0003" rgba="1 1 1 1"
          contype="0" conaffinity="0"/>
    <geom name="table" type="box" pos="{table[0]} {table[1]} 0.74" size="0.6 0.35 0.02" material="wood"
          conaffinity="3"/>
    {legs}
    <geom name="laptop" type="box" pos="{table[0] - 0.1} {table[1] + 0.05} 0.77" size="0.16 0.11 0.01"
          rgba="0.25 0.25 0.28 1" conaffinity="3"/>
    <geom name="shelf" type="box" pos="{-h + 0.22} 1.0 0.9" size="0.2 0.5 0.9" material="wood"
          conaffinity="3"/>
    <geom name="box" type="box" pos="-1.2 -1.5 0.15" size="0.2 0.15 0.15" rgba="0.75 0.6 0.4 1"
          conaffinity="3"/>
    """
    assets = """
    <material name="wall" rgba="0.92 0.9 0.86 0.3"/>
    <material name="wood" rgba="0.6 0.45 0.3 1"/>
    """
    return assets, world, ""


def balloon_xml():
    """(worldbody, extra) MJCF: balloons hanging off the frame's "top" site on strings.

    Each string is a spatial tendon with a length limit: slack below its
    length, a hard stop at it - which is all a string does.
    """
    top = np.array([0.0, 0.0, START_Z + 0.02])
    bodies, strings = [], []
    for i in range(N_BALLOONS):
        az = 2.0 * math.pi * i / N_BALLOONS
        tilt = math.radians(25.0)
        length = STRING_L[i % len(STRING_L)]
        knot = top + length * np.array([math.sin(tilt) * math.cos(az), math.sin(tilt) * math.sin(az),
                                        math.cos(tilt)])
        centre = knot + np.array([0.0, 0.0, BALLOON_R])
        # contype/conaffinity 2: balloons hit each other and the room, not the frame.
        bodies.append(f"""
    <body name="balloon{i}" pos="{centre[0]:.4f} {centre[1]:.4f} {centre[2]:.4f}">
      <freejoint/>
      <geom type="sphere" size="{BALLOON_R}" mass="{BALLOON_MASS}" rgba="{BALLOON_RGB[i % 5]} 0.92"
            contype="2" conaffinity="2" friction="0.3 0.005 0.0001"/>
      <site name="knot{i}" pos="0 0 {-BALLOON_R}" size="0.004"/>
    </body>""")
        strings.append(f"""
    <spatial name="string{i}" limited="true" range="0 {length}" width="0.0012" rgba="0.95 0.95 0.95 1">
      <site site="top"/>
      <site site="knot{i}"/>
    </spatial>""")
    return "".join(bodies), "<tendon>" + "".join(strings) + "\n  </tendon>"


class FlyEnv(HoverEnv):
    """HoverEnv in the room: throttle, targets and integrators come from the pilot, not a script."""

    def __init__(self, nominal, seed, balloons):
        super().__init__(nominal=nominal, randomize_scale=0.0, episode_s=1e9, kick_rate_hz=0.0, seed=seed)
        self.wind = np.zeros(3)
        self.ground_effect = 1.0
        self.stick_throttle = 0.0
        self.airborne = False
        self.yaw_rate_sp = 0.0
        self.build(balloons)

    def build(self, balloons):
        """(Re)compile the model with or without the balloons."""
        self.balloons = balloons
        assets, world, extra = room_xml()
        if balloons:
            b_world, b_extra = balloon_xml()
            world += b_world
            extra += b_extra
        self.drone = Drone(self.nominal, assets, world, extra)
        self.drone.relief_on_body = not balloons
        self.drone.force_hook = self._forces
        m = self.drone.model
        self._motor_sites = np.array([m.site("m%d" % n).id for n in airframe.ARMS])
        if balloons:
            self._bal_body = np.array([m.body("balloon%d" % i).id for i in range(N_BALLOONS)])
            adr = np.array([m.jnt_dofadr[m.body_jntadr[b]] for b in self._bal_body])
            self._bal_lin = adr[:, None] + np.arange(3)       # world-frame linear velocity dofs
            self._bal_ang = adr[:, None] + 3 + np.arange(3)   # body-frame angular velocity dofs

    # HoverEnv's open-loop take-off ramp and integrator timing, replaced by the pilot.
    def throttle_at(self, t):
        return self.stick_throttle

    @property
    def integrating(self):
        return self.airborne

    def fly_reset(self, params):
        self.reset(options={"params": params, "ground": True})
        self.wander_amp = 0.0
        self.target = (0.0, 0.0)
        self.yaw_rate_sp = 0.0
        self.base_scale = self.drone._scale.copy()

    def _forces(self, d):
        """Called by Drone every physics step, after its own forces: wind, balloons, ground effect."""
        dr = self.drone
        # Wind on the frame: its linear drag works on air speed, not ground speed.
        d.xfrc_applied[dr.body, 0:3] += dr.p.drag_lin * self.wind
        if self.balloons:
            v = d.qvel[self._bal_lin]
            rel = v - self.wind
            f = -BALLOON_DRAG * np.linalg.norm(rel, axis=1, keepdims=True) * rel
            f[:, 2] += BALLOON_MASS * P.G + dr.p.relief_n / N_BALLOONS
            d.xfrc_applied[self._bal_body, 0:3] = f
            R = d.xmat[self._bal_body].reshape(-1, 3, 3)
            w_world = np.einsum("nij,nj->ni", R, d.qvel[self._bal_ang])
            d.xfrc_applied[self._bal_body, 3:6] = -BALLOON_SPIN_DRAG * w_world
        # Ground effect: thrust x 1/(1 - (R/4h)^2) with h the prop height
        # (Cheeseman-Bennett), capped where the formula blows up.
        h = np.maximum(d.site_xpos[self._motor_sites, 2], 0.6 * PROP_R)
        ge = np.minimum(1.0 / (1.0 - (PROP_R / (4.0 * h)) ** 2), 1.4)
        d.ctrl[:] *= ge
        self.ground_effect = float(np.mean(ge))


class Battery:
    """1S LiPo: open-circuit voltage from state of charge, drained by motor load."""

    CURVE_SOC = (0.0, 0.1, 0.2, 0.5, 0.8, 1.0)
    CURVE_V = (3.3, 3.55, 3.68, 3.80, 3.95, 4.2)

    def __init__(self, hover_throttle, minutes):
        # Hovering takes it from full to 20 % in `minutes`; current ~ throttle^2.
        self.soc = 1.0
        self.rate = 0.8 / (minutes * 60.0) / hover_throttle ** 2

    def drain(self, mean_out, dt):
        self.soc = max(0.0, self.soc - self.rate * mean_out ** 2 * dt)

    @property
    def volts(self):
        return float(np.interp(self.soc, self.CURVE_SOC, self.CURVE_V))

    @property
    def thrust_factor(self):
        """Motor thrust relative to a fresh pack (what drone_params.json describes)."""
        return (self.volts / 4.2) ** 2


class AltHold:
    """Sim-only height hold on the TRUE height (the real drone has no sensor for this)."""

    KP, KD, KI = 0.35, 0.45, 0.15

    def __init__(self):
        self.z_sp = 0.0
        self.i = 0.0

    def throttle(self, hover, idle, z, vz, dt):
        if self.z_sp < 0.05 and z < 0.06:
            self.i = 0.0
            return 0.0                       # landed and asked to stay down: motors off
        err = self.z_sp - z
        if abs(err) < 0.1 and abs(vz) < 0.2:
            # Only near the target: integrating through a climb winds it up and overshoots.
            self.i = max(-0.25, min(0.25, self.i + self.KI * err * dt))
        return max(idle, min(1.0, hover + self.KP * err - self.KD * vz + self.i))


class Zero:
    def reset(self):
        pass

    def __call__(self, env, dt):
        return np.zeros(3)


CONTROLLERS = ("policy", "pid", "none")
CONTROLLER_NAMES = {"policy": "PPO policy (src/flight/policy_weights.py)",
                    "pid": "lift.py PID", "none": "none - nothing levels it"}


class Flight:
    """Everything except the window: the drone, the pilot's inputs, the helpers."""

    def __init__(self, args):
        self.args = args
        self.nominal = P.load()
        self.rng = np.random.default_rng(args.seed)
        self.env = FlyEnv(self.nominal, args.seed, balloons=not args.no_balloons)
        self.controllers = {"policy": PolicyController(policy_weights.LAYERS), "pid": PIDController(),
                            "none": Zero()}
        self.ctrl_name = args.ctrl
        self.hold_on = not args.manual
        self.loiter_on = args.loiter
        self.battery_on = not args.no_battery
        self.gusts_on = args.wind
        self.params, self.drone_label = self.nominal, "nominal"
        if args.random:
            self.new_drone()
        self.paused = False
        self.reset()

    # --- toggles ---------------------------------------------------------------------
    def reset(self):
        env = self.env
        env.fly_reset(self.params)
        self.battery = Battery(self.params.hover_throttle, self.args.flight_min)
        self.hold = AltHold()
        self.manual = 0.0
        self.disarmed = None
        self.gust = np.zeros(3)
        self.wind_mean = np.zeros(3)
        self.hold_xy = np.zeros(2)
        if self.gusts_on:
            self.new_wind()
        env.wind[:] = 0.0
        env.last_action = np.zeros(3)
        self.ctrl.reset()
        self.history = deque(maxlen=1000)

    @property
    def ctrl(self):
        return self.controllers[self.ctrl_name]

    def cycle_controller(self):
        self.ctrl_name = CONTROLLERS[(CONTROLLERS.index(self.ctrl_name) + 1) % len(CONTROLLERS)]
        self.ctrl.reset()
        self.env.last_action = np.zeros(3)

    def toggle_hold(self):
        z, _ = self.height()
        if self.hold_on:
            # Bumpless: manual throttle starts from what the hold was giving.
            self.manual = self.env.stick_throttle
        else:
            self.hold.z_sp = z if self.env.airborne else 0.0
            self.hold.i = self.env.stick_throttle - self.env.p.hover_throttle if self.env.airborne else 0.0
        self.hold_on = not self.hold_on

    def new_wind(self):
        a = self.rng.uniform(0, 2 * math.pi)
        self.wind_mean = WIND_MEAN_MPS * np.array([math.cos(a), math.sin(a), 0.0])

    def toggle_wind(self):
        self.gusts_on = not self.gusts_on
        if self.gusts_on:
            self.new_wind()

    def toggle_balloons(self):
        self.env.build(not self.env.balloons)
        self.reset()

    def new_drone(self):
        self.params = P.randomize(self.nominal, self.rng, 1.0)
        self.drone_label = "random"

    def nominal_drone(self):
        self.params, self.drone_label = self.nominal, "nominal"

    def kick(self):
        self.env.drone.kick(self.rng.uniform(-200, 200, 3) * np.array([1.0, 1.0, 0.5]))

    # --- one pass of the board's loop ------------------------------------------------
    def height(self):
        dr = self.env.drone
        return float(dr.position()[2] - START_Z), float(dr.data.qvel[2])

    def loiter(self, stick, always=False):
        """
        Sim-only position hold: lean against drift toward hold_xy. With the
        arrows held the pilot is flying, so the hold point follows the drone
        (always=True: the scripted check, which adds manoeuvres on top).
        """
        dr = self.env.drone
        pos, vel = dr.position()[0:2], dr.data.qvel[0:2]
        if not always and (stick["roll"] or stick["pitch"]):
            self.hold_xy = pos.copy()
            return stick
        # Desired tilt toward the hold point, world frame, then into the body
        # frame: pitch > 0 = nose down = +x body, roll > 0 = right = -y body.
        want = -LOITER_KP * (pos - self.hold_xy) - LOITER_KD * vel
        yaw = math.radians(dr.attitude_deg()[2])
        fwd = math.cos(yaw) * want[0] + math.sin(yaw) * want[1]
        left = -math.sin(yaw) * want[0] + math.cos(yaw) * want[1]
        out = dict(stick)
        out["pitch"] = max(-1.0, min(1.0, stick["pitch"] + fwd / MAX_TILT_DEG))
        out["roll"] = max(-1.0, min(1.0, stick["roll"] - left / MAX_TILT_DEG))
        return out

    def step(self, stick, always_loiter=False):
        """stick: roll, pitch, yaw, climb, each -1..1 (what the keys hold right now)."""
        env, dr = self.env, self.env.drone
        dt = env.period
        z, vz = self.height()
        if (self.loiter_on or always_loiter) and env.airborne:
            stick = self.loiter(stick, always_loiter)
        elif not env.airborne:
            self.hold_xy = dr.position()[0:2].copy()

        # Tilt targets slew toward the sticks; yaw is a rate.
        step = TILT_SLEW_DPS * dt
        goal = (stick["roll"] * MAX_TILT_DEG, stick["pitch"] * MAX_TILT_DEG)
        env.target = tuple(t + max(-step, min(step, g - t)) for t, g in zip(env.target, goal, strict=True))
        env.yaw_rate_sp = stick["yaw"] * MAX_YAW_DPS

        if self.disarmed:
            thr = 0.0
        elif self.hold_on:
            self.hold.z_sp = max(0.0, min(ROOM_H - 0.5, self.hold.z_sp + stick["climb"] * CLIMB_MPS * dt))
            thr = self.hold.throttle(env.p.hover_throttle, env.p.idle, z, vz, dt)
        else:
            self.manual = max(0.0, min(1.0, self.manual + stick["climb"] * THROTTLE_RATE * dt))
            thr = self.manual
        env.stick_throttle = thr
        env.airborne = thr > 0.0 and (z > 0.03 or thr > env.p.hover_throttle - 0.05)

        if self.battery_on:
            self.battery.drain(float(np.mean(list(env.out.values()))), dt)
            dr._scale = env.base_scale * self.battery.thrust_factor
        else:
            dr._scale = env.base_scale

        if self.gusts_on:
            a = dt / WIND_TAU_S
            self.gust += -a * self.gust + math.sqrt(2 * a) * WIND_GUST_MPS * self.rng.normal(size=3) \
                * np.array([1.0, 1.0, 0.3])
            env.wind = self.wind_mean + self.gust
        else:
            env.wind = np.zeros(3)

        if thr == 0.0:
            cmd = np.zeros(3)
        else:
            cmd = self.ctrl(env, dt)
            if self.ctrl_name == "policy":
                # What the policy itself said, for its next observation (as in rollout()).
                env.last_action = np.clip(cmd / ACT_SCALE, -1.0, 1.0)
                cmd = cmd.copy()
                cmd[2] += YAW_KP * (env.yaw_rate_sp - env.gyro[2])
        _, _, term, _, info = env.step_cmd(cmd)
        if term and not self.disarmed and thr > 0.0:
            # lift.py's cut-out: past 35 deg it stops all four motors.
            self.disarmed = "tilted past %.0f deg - lift.py cuts the motors" % TILT_LIMIT_DEG
        self.history.append((env.t, info["roll"], info["pitch"], env.target[0], env.target[1], env.airborne))

    def rms_error(self):
        e = [(r - tr) ** 2 + (q - tq) ** 2 for _, r, q, tr, tq, air in self.history if air]
        return math.sqrt(sum(e) / len(e)) if len(e) > 50 else None


# --- drawing helpers ---------------------------------------------------------------------
def add_arrow(scn, a, b, width, rgba):
    if scn.ngeom >= scn.maxgeom:
        return
    g = scn.geoms[scn.ngeom]
    mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_ARROW, np.zeros(3), np.zeros(3), np.zeros(9),
                        np.array(rgba, dtype=np.float32))
    mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_ARROW, width, np.asarray(a, float), np.asarray(b, float))
    scn.ngeom += 1


def draw_extras(scn, f):
    env = f.env
    d, m = env.drone.data, env.drone.model
    R = d.xmat[env.drone.body].reshape(3, 3)
    for n in airframe.ARMS:
        p = d.site_xpos[m.site("m%d" % n).id]
        add_arrow(scn, p, p + R[:, 2] * float(d.ctrl[n - 1]), 0.003, (1, 0.55, 0.1, 0.9))
    if np.any(env.wind):
        c = env.drone.position() + np.array([0, 0, 0.12])
        add_arrow(scn, c, c + env.wind * 0.25, 0.003, (0.1, 0.8, 0.9, 0.8))


def attitude_figure(f):
    fig = mujoco.MjvFigure()
    mujoco.mjv_defaultFigure(fig)
    fig.title = "true attitude vs target, last 10 s (deg)"
    fig.figurergba[3] = 0.55
    fig.gridsize[0], fig.gridsize[1] = 5, 5
    fig.range[1][0], fig.range[1][1] = -30, 30
    fig.flg_extend = 0
    names = ("roll", "pitch", "roll target", "pitch target")
    colours = ((1, 0.3, 0.3), (0.3, 0.6, 1), (1, 0.75, 0.75), (0.75, 0.85, 1))
    hist = list(f.history)[::2]
    for k in range(4):
        fig.linename[k] = names[k]
        fig.linergb[k] = colours[k]
        fig.linepnt[k] = len(hist)
        for i, row in enumerate(hist):
            fig.linedata[k][2 * i] = row[0] - (hist[-1][0] if hist else 0.0)
            fig.linedata[k][2 * i + 1] = row[1 + k]
    fig.range[0][0], fig.range[0][1] = -10, 0
    return fig


def status_text(f):
    env = f.env
    p = env.p
    z, vz = f.height()
    roll, pitch, _ = env.drone.attitude_deg()
    rms = f.rms_error()
    if f.hold_on:
        thr = "ALT HOLD %.2f m (sim only) -> %.2f" % (f.hold.z_sp, env.stick_throttle)
    else:
        thr = "MANUAL %.2f (hover %.2f fresh)" % (env.stick_throttle, p.hover_throttle)
    rows = [
        ("controller", CONTROLLER_NAMES[f.ctrl_name]),
        ("throttle", thr),
        ("height", "%.2f m   climb %+.2f m/s" % (z, vz)),
        ("position", "LOITER (sim only)" if f.loiter_on else "free - you correct the drift"),
        ("roll", "%+5.1f deg  target %+5.1f" % (roll, env.target[0])),
        ("pitch", "%+5.1f deg  target %+5.1f" % (pitch, env.target[1])),
        ("yaw rate", "%+4.0f dps  target %+4.0f" % (env.drone.rates_dps()[2], env.yaw_rate_sp)),
        ("attitude error", "RMS %.1f deg over 10 s" % rms if rms is not None else "- (not flying)"),
        ("battery", "%.2f V  %.0f %%  thrust x%.2f%s" % (f.battery.volts, f.battery.soc * 100,
                                                          f.battery.thrust_factor,
                                                          "  LAND NOW" if f.battery.volts < 3.5 else "")
         if f.battery_on else "off (always fresh)"),
        ("weight relief", "%d balloons on strings, %.0f g" % (N_BALLOONS, p.relief_n / P.G * 1000)
         if env.balloons else "%.0f g lumped at the CoM (as trained)" % (p.relief_n / P.G * 1000)),
        ("wind", "%.2f m/s gusting" % float(np.linalg.norm(env.wind)) if f.gusts_on else "off"),
        ("ground effect", "x%.2f thrust" % env.ground_effect),
        ("drone", "%s: %.0f g, hover %.2f, CoM %+.1f %+.1f mm" % (f.drone_label, p.mass_g, p.hover_throttle,
                                                                   *p.com_offset_mm[:2])),
        ("sim time", "%.1f s%s" % (env.t, "   PAUSED" if f.paused else "")),
    ]
    return "\n".join(r[0] for r in rows), "\n".join(r[1] for r in rows)


HELP = (
    ("arrows", "tilt (up = forward)"), ("W / S", "climb / descend"), ("A / D", "yaw"),
    ("H", "altitude hold on/off"), ("L", "loiter on/off"), ("X", "cut motors"), ("C", "controller"), ("SPACE", "kick"),
    ("G", "wind gusts"), ("B", "balloons / lumped"), ("V", "fresh battery"),
    ("N / M", "random / nominal drone"), ("R", "reset"), ("F", "camera"), ("P", "pause"),
    ("mouse", "drag orbit, scroll zoom"), ("ESC", "quit"),
)


# --- the window ------------------------------------------------------------------------
def run_window(f):
    if not glfw.init():
        raise SystemExit("could not start GLFW (no display?)")
    glfw.window_hint(glfw.SAMPLES, 4)
    win = glfw.create_window(1500, 950, "pico-drone: fly it", None, None)
    glfw.make_context_current(win)
    glfw.swap_interval(1)

    opt = mujoco.MjvOption()
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    ui = {"cam": "chase", "az_off": 0.0, "el": -15.0, "dist": 1.0, "drag": None, "model": None}
    ui["lookat"] = f.env.drone.position()

    def make_render():
        ui["model"] = f.env.drone.model
        ui["scene"] = mujoco.MjvScene(ui["model"], maxgeom=4000)
        ui["ctx"] = mujoco.MjrContext(ui["model"], mujoco.mjtFontScale.mjFONTSCALE_150)

    make_render()

    def on_key(_win, key, _sc, action, _mods):
        if action != glfw.PRESS:
            return
        if key == glfw.KEY_ESCAPE:
            glfw.set_window_should_close(win, True)
        elif key == glfw.KEY_SPACE:
            f.kick()
        elif key == glfw.KEY_R:
            f.reset()
        elif key == glfw.KEY_H:
            f.toggle_hold()
        elif key == glfw.KEY_L:
            f.loiter_on = not f.loiter_on
            f.hold_xy = f.env.drone.position()[0:2].copy()
        elif key == glfw.KEY_X:
            f.hold.z_sp, f.manual = 0.0, 0.0
            f.disarmed = "motors cut (X)"
        elif key == glfw.KEY_C:
            f.cycle_controller()
            print("controller:", CONTROLLER_NAMES[f.ctrl_name], flush=True)
        elif key == glfw.KEY_G:
            f.toggle_wind()
        elif key == glfw.KEY_B:
            f.toggle_balloons()
        elif key == glfw.KEY_V:
            f.battery = Battery(f.params.hover_throttle, f.args.flight_min)
        elif key == glfw.KEY_N:
            f.new_drone()
            f.reset()
            print("new drone:", P.describe(f.params), flush=True)
        elif key == glfw.KEY_M:
            f.nominal_drone()
            f.reset()
        elif key == glfw.KEY_F:
            ui["cam"] = "tripod" if ui["cam"] == "chase" else "chase"
        elif key == glfw.KEY_P:
            f.paused = not f.paused

    def on_button(_win, button, action, _mods):
        ui["drag"] = glfw.get_cursor_pos(win) if action == glfw.PRESS else None

    def on_cursor(_win, x, y):
        if ui["drag"] is None:
            return
        x0, y0 = ui["drag"]
        ui["az_off"] -= (x - x0) * 0.3
        ui["el"] = max(-85.0, min(10.0, ui["el"] - (y - y0) * 0.3))
        ui["drag"] = (x, y)

    def on_scroll(_win, _dx, dy):
        ui["dist"] = max(0.3, min(6.0, ui["dist"] * 0.9 ** dy))

    glfw.set_key_callback(win, on_key)
    glfw.set_mouse_button_callback(win, on_button)
    glfw.set_cursor_pos_callback(win, on_cursor)
    glfw.set_scroll_callback(win, on_scroll)

    def held(*keys):
        return any(glfw.get_key(win, k) == glfw.PRESS for k in keys)

    last = time.perf_counter()
    while not glfw.window_should_close(win):
        now = time.perf_counter()
        frame_dt = min(now - last, 0.05)
        last = now
        if f.env.drone.model is not ui["model"]:      # B rebuilt the model
            make_render()
        stick = {
            "roll": held(glfw.KEY_RIGHT) - held(glfw.KEY_LEFT),
            "pitch": held(glfw.KEY_UP) - held(glfw.KEY_DOWN),
            "yaw": held(glfw.KEY_A) - held(glfw.KEY_D),
            "climb": held(glfw.KEY_W) - held(glfw.KEY_S),
        }
        if not f.paused:
            # Run the sim up to wall-clock time (real time, at most 50 ms per frame).
            end = f.env.t + frame_dt
            while f.env.t < end:
                f.step(stick)

        # Camera: chase sits behind the nose; tripod stands in a corner and pans.
        pos = f.env.drone.position()
        ui["lookat"] += (pos - ui["lookat"]) * 0.15
        cam.lookat[:] = ui["lookat"]
        if ui["cam"] == "chase":
            cam.azimuth = f.env.drone.attitude_deg()[2] + ui["az_off"]
            cam.elevation = ui["el"]
            cam.distance = ui["dist"]
        else:
            eye = np.array([ROOM_HALF - 0.15, ROOM_HALF - 0.15, ROOM_H - 0.3])
            v = ui["lookat"] - eye
            cam.azimuth = math.degrees(math.atan2(v[1], v[0]))
            cam.elevation = math.degrees(math.atan2(v[2], math.hypot(v[0], v[1])))
            cam.distance = float(np.linalg.norm(v))

        w, h = glfw.get_framebuffer_size(win)
        vp = mujoco.MjrRect(0, 0, w, h)
        scn, ctx = ui["scene"], ui["ctx"]
        mujoco.mjv_updateScene(ui["model"], f.env.drone.data, opt, None, cam, mujoco.mjtCatBit.mjCAT_ALL, scn)
        draw_extras(scn, f)
        mujoco.mjr_render(vp, scn, ctx)
        left, right = status_text(f)
        mujoco.mjr_overlay(mujoco.mjtFont.mjFONT_NORMAL, mujoco.mjtGridPos.mjGRID_TOPLEFT, vp, left, right, ctx)
        mujoco.mjr_overlay(mujoco.mjtFont.mjFONT_NORMAL, mujoco.mjtGridPos.mjGRID_TOPRIGHT, vp,
                           "\n".join(k for k, _ in HELP), "\n".join(v for _, v in HELP), ctx)
        if f.disarmed:
            mujoco.mjr_overlay(mujoco.mjtFont.mjFONT_BIG, mujoco.mjtGridPos.mjGRID_BOTTOMRIGHT, vp,
                               "DISARMED: %s" % f.disarmed, "R = reset on the pad", ctx)
        if f.history:
            mujoco.mjr_figure(mujoco.MjrRect(0, 0, w // 3, h // 3), attitude_figure(f), ctx)
        glfw.swap_buffers(win)
        glfw.poll_events()
    glfw.terminate()


# --- headless check ----------------------------------------------------------------------
def check(args):
    """
    A scripted 40 s flight per controller - take off to 1 m, then a sequence of
    tilts and yaws with gusts on, a kick at 20 s - and print how well it held
    its targets. Loiter runs underneath throughout, so the wind can't carry it
    into a wall: this measures the levelling, not the piloting.
    """
    script = [  # (from s, roll, pitch, yaw, climb); tilt doublets end where they started
        (0.0, 0, 0, 0, 1), (2.0, 0, 0, 0, 0), (6.0, 0, 0.6, 0, 0), (7.0, 0, -0.6, 0, 0),
        (9.0, 0, 0.6, 0, 0), (10.0, 0.6, 0, 0, 0), (11.0, -0.6, 0, 0, 0), (13.0, 0.6, 0, 0, 0),
        (14.0, 0, 0, 1, 0), (16.0, 0, 0, -1, 0), (18.0, 0, 0, 0, 0), (26.0, 1, 1, 0, 0),
        (27.0, -1, -1, 0, 0), (29.0, 1, 1, 0, 0), (30.0, 0, 0, 0, 0),
    ]
    print("%-8s %-9s %9s %8s %9s %9s" % ("ctrl", "relief", "disarmed", "RMS err", "max tilt", "height"))
    for balloons in (True, False):
        for name in ("policy", "pid"):
            args.ctrl, args.no_balloons, args.wind = name, not balloons, True
            f = Flight(args)
            kicked = False
            worst = 0.0
            errs = []
            while f.env.t < 40.0:
                row = [r for r in script if r[0] <= f.env.t][-1]
                f.step({"roll": row[1], "pitch": row[2], "yaw": row[3], "climb": row[4]}, always_loiter=True)
                if f.env.t > 20.0 and not kicked:
                    f.kick()
                    kicked = True
                h = f.history[-1]
                if f.env.t > 3.0:
                    errs.append((h[1] - h[3]) ** 2 + (h[2] - h[4]) ** 2)
                    worst = max(worst, abs(h[1]), abs(h[2]))
            print("%-8s %-9s %9s %7.1f° %8.1f° %8.2fm"
                  % (name, "balloons" if balloons else "lumped",
                     "no" if not f.disarmed else "YES",
                     math.sqrt(np.mean(errs)), worst, f.height()[0]))
            if f.disarmed:
                print("          ", f.disarmed)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ctrl", choices=CONTROLLERS, default="policy")
    ap.add_argument("--manual", action="store_true", help="start with manual throttle (no altitude hold)")
    ap.add_argument("--wind", action="store_true", help="start with gusts on")
    ap.add_argument("--loiter", action="store_true", help="start with loiter (sim-only position hold) on")
    ap.add_argument("--random", action="store_true", help="start with a randomized drone")
    ap.add_argument("--no-balloons", action="store_true", help="relief at the CoM, as in training")
    ap.add_argument("--no-battery", action="store_true", help="battery never drains")
    ap.add_argument("--flight-min", type=float, default=6.0, help="minutes of hover until the pack is at 20%%")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--check", action="store_true", help="no window: scripted flight, policy vs PID")
    args = ap.parse_args()
    if args.check:
        check(args)
        return
    f = Flight(args)
    print("window open - click it, then W to take off (H toggles altitude hold). ESC quits.", flush=True)
    run_window(f)


if __name__ == "__main__":
    main()
