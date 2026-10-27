"""
Controllers that drive HoverEnv the way the board would.

PIDController is lift.py's levelling loop line for line (angle P -> rate PID ->
mixer, integrators held empty during the ramp), built on the board's own
flight/pid.py. It is the baseline the RL policy has to beat, and the plant the
autotune is checked against.
"""

import numpy as np

from flight.pid import PID
from flight.policy import ACT_SCALE, Policy

# lift.py's EDIT HERE defaults.
LIFT_GAINS = {
    "roll_rate": (0.0040, 0.0020, 0.00006),
    "pitch_rate": (0.0040, 0.0020, 0.00006),
    "yaw_rate": (0.0050, 0.0010, 0.0),
    "roll_angle_kp": 4.0,
    "pitch_angle_kp": 4.0,
    "max_angle_rate_dps": 150.0,
    "rate_i_limit": 50.0,
}


class PIDController:
    def __init__(self, gains=None):
        g = dict(LIFT_GAINS)
        g.update(gains or {})
        self.g = g
        self.reset()

    def reset(self):
        g = self.g
        self.rate = {
            "roll": PID(*g["roll_rate"], integral_limit=g["rate_i_limit"]),
            "pitch": PID(*g["pitch_rate"], integral_limit=g["rate_i_limit"]),
            "yaw": PID(*g["yaw_rate"], integral_limit=g["rate_i_limit"]),
        }
        lim = g["max_angle_rate_dps"]
        self.angle = {
            "roll": PID(g["roll_angle_kp"], 0.0, 0.0, output_limit=lim),
            "pitch": PID(g["pitch_angle_kp"], 0.0, 0.0, output_limit=lim),
        }

    def __call__(self, env, dt):
        """Corrections (roll, pitch, yaw) from the board-visible state of `env`."""
        if not env.integrating:
            for pid in self.rate.values():
                pid._integral = 0.0
        roll_sp = self.angle["roll"].update(env.level[0] + env.target[0], env.est[0], dt)
        pitch_sp = self.angle["pitch"].update(env.level[1] + env.target[1], env.est[1], dt)
        g = env.gyro
        return np.array([
            self.rate["roll"].update(roll_sp, g[0], dt),
            self.rate["pitch"].update(pitch_sp, g[1], dt),
            self.rate["yaw"].update(getattr(env, "yaw_rate_sp", 0.0), g[2], dt),
        ])


class PolicyController:
    """The exported pure-Python policy, run exactly as the board runs it."""

    def __init__(self, layers):
        self.policy = Policy(layers)

    def reset(self):
        pass

    def __call__(self, env, dt):
        return np.array(self.policy.act(list(env._obs())))


def rollout(env, controller, options=None, seed=None):
    """One episode with a controller producing mixer-unit commands. Returns a log dict."""
    env.reset(seed=seed, options=options)
    controller.reset()
    log = {"t": [], "roll": [], "pitch": [], "yaw_rate": [], "out": [], "z": []}
    total, done = 0.0, False
    while not done:
        cmd = controller(env, env.period)
        _, r, term, trunc, info = env.step_cmd(cmd)
        if isinstance(controller, PolicyController):
            env.last_action = np.clip(cmd / ACT_SCALE, -1.0, 1.0)
        total += r
        done = term or trunc
        log["t"].append(env.t)
        log["roll"].append(info["roll"])
        log["pitch"].append(info["pitch"])
        log["yaw_rate"].append(float(env.drone.rates_dps()[2]))
        log["out"].append([env.out[m] for m in (1, 2, 3, 4)])
        log["z"].append(float(env.drone.position()[2]))
    log["return"] = total
    log["crashed"] = bool(term)
    return log

