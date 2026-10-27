"""
Drone parameters: the nominal values in sim/drone_params.json, and per-episode
randomization around them (domain randomization).

Why randomize: nothing on this airframe has been measured on a scale yet, the
720 motors' thrust depends on the supply, and wires shift the CoM. A policy
trained on one exact drone learns that drone's quirks and falls over on the
real one. Trained across the whole plausible range, the real drone is just
one more sample it has already seen.
"""

import copy
import json
import math
from dataclasses import dataclass, field
from pathlib import Path

from . import airframe

PARAMS_PATH = Path(__file__).resolve().parent.parent / "drone_params.json"
G = 9.81


@dataclass
class DroneParams:
    # Mass, CoM and inertia come from the CAD parts (airframe.py). These are
    # what the CAD doesn't know: wires/tape, and how far they pull the CoM.
    gps_fitted: bool = True
    mag_fitted: bool = False
    extra_mass_g: float = 6.0
    com_offset_mm: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    inertia_scale: float = 1.0
    thrust_full_g: float = 15.0
    thrust_curve_exp: float = 1.7
    throttle_deadband: float = 0.15
    motor_scale: list = field(default_factory=lambda: [1.0, 1.0, 1.0, 1.0])
    motor_tau_up_ms: float = 35.0
    motor_tau_down_ms: float = 55.0
    yaw_coef_mm: float = 10.0
    supply_sag: float = 0.08
    hover_throttle: float = 0.62
    idle: float = 0.40
    control_hz: float = 100.0
    latency_ms: float = 4.0
    gyro_noise_dps: float = 0.2
    gyro_bias_dps: float = 0.0
    gyro_vibration_dps: float = 0.6
    accel_noise_ms2: float = 0.1
    accel_vibration_ms2: float = 1.5
    imu_misalign_deg: list = field(default_factory=lambda: [0.0, 0.0])
    drag_lin: float = 0.03
    drag_rot: float = 2e-6
    relief_drag_z: float = 0.15
    # Per-axis gyro bias, filled in by randomize(); nominal is unbiased.
    gyro_bias: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    randomize_ranges: dict = field(default_factory=dict)

    # --- derived quantities ---------------------------------------------------
    @property
    def mass_g(self):
        return airframe.cad_mass_g(self.gps_fitted, self.mag_fitted) + self.extra_mass_g

    @property
    def mass(self):
        return self.mass_g / 1000.0

    def thrust_n(self, throttle):
        """Steady-state thrust of a nominal motor at a mixer throttle (0..1), newtons."""
        u = (throttle - self.throttle_deadband) / (1.0 - self.throttle_deadband)
        if u <= 0.0:
            return 0.0
        return self.thrust_full_g / 1000.0 * G * u ** self.thrust_curve_exp

    @property
    def relief_n(self):
        """
        Upward force that makes the drone hover at `hover_throttle`.

        This stands in for whatever takes weight off the 720 motors (the helium
        balloon on the real drone). The policy never sees it - it only sees that
        some throttle hovers. Zero if the motors can lift it on their own there.
        """
        motors = 4.0 * self.thrust_n(self.hover_throttle) * (1.0 - self.supply_sag * self.hover_throttle)
        return max(0.0, self.mass * G - motors)


def load(path=PARAMS_PATH):
    with open(path) as f:
        raw = json.load(f)
    raw = {k: v for k, v in raw.items() if not k.startswith("_")}
    ranges = raw.pop("randomize", {})
    p = DroneParams(**raw)
    p.randomize_ranges = ranges
    return p


def randomize(nominal, rng, scale=1.0):
    """
    One random drone around `nominal`. scale=0 returns the nominal drone,
    scale=1 the full ranges in drone_params.json. Ranges are [lo, hi] absolute
    bounds, or a single number meaning +- that much around nominal.
    """
    p = copy.deepcopy(nominal)
    r = nominal.randomize_ranges
    if scale <= 0.0 or not r:
        return p

    def draw(name, nominal_value):
        spec = r.get(name)
        if spec is None:
            return nominal_value
        if isinstance(spec, list):
            lo, hi = spec
        else:
            lo, hi = nominal_value - spec, nominal_value + spec
        # Shrink the range toward nominal for scale < 1 (curriculum).
        lo = nominal_value + (lo - nominal_value) * scale
        hi = nominal_value + (hi - nominal_value) * scale
        return float(rng.uniform(min(lo, hi), max(lo, hi)))

    for name in ("extra_mass_g", "inertia_scale", "thrust_full_g", "thrust_curve_exp",
                 "throttle_deadband", "motor_tau_up_ms", "yaw_coef_mm", "supply_sag",
                 "hover_throttle", "control_hz", "latency_ms", "gyro_noise_dps",
                 "gyro_vibration_dps", "accel_noise_ms2", "accel_vibration_ms2",
                 "drag_lin", "drag_rot", "relief_drag_z"):
        setattr(p, name, draw(name, getattr(nominal, name)))

    f = r.get("motor_tau_down_ms_factor")
    if f:
        p.motor_tau_down_ms = p.motor_tau_up_ms * float(rng.uniform(*f)) ** scale
    xy = r.get("com_offset_mm_xy", 0.0) * scale
    p.com_offset_mm = [
        nominal.com_offset_mm[0] + float(rng.uniform(-xy, xy)),
        nominal.com_offset_mm[1] + float(rng.uniform(-xy, xy)),
        draw("com_offset_mm_z", nominal.com_offset_mm[2]),
    ]
    ms = r.get("motor_scale", 0.0) * scale
    p.motor_scale = [s * float(rng.uniform(1.0 - ms, 1.0 + ms)) for s in nominal.motor_scale]
    b = r.get("gyro_bias_dps", 0.0) * scale
    p.gyro_bias = [float(rng.uniform(-b, b)) for _ in range(3)]
    mis = r.get("imu_misalign_deg", 0.0) * scale
    p.imu_misalign_deg = [nominal.imu_misalign_deg[i] + float(rng.uniform(-mis, mis))
                          for i in range(2)]
    return p


def describe(p):
    """One line for printouts."""
    return ("mass %.0fg (%.0f wires)  CoM %+.1f,%+.1f,%+.1f mm  thrust %.1fg/motor  hover %.2f  "
            "relief %.0fg  tau %.0f/%.0fms  %3.0f Hz  lat %.1fms"
            % (p.mass_g, p.extra_mass_g, *p.com_offset_mm, p.thrust_full_g, p.hover_throttle,
               p.relief_n / G * 1000.0, p.motor_tau_up_ms, p.motor_tau_down_ms,
               p.control_hz, p.latency_ms))


def rad(deg):
    return deg * math.pi / 180.0
