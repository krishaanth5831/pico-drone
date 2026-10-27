"""
Watch the simulated pico-drone fly, in a MuJoCo window.

    cd sim
    uv run python watch.py                      # lift.py's PID, nominal drone
    uv run python watch.py --ctrl policy        # the trained policy (src/flight/policy_weights.py)
    uv run python watch.py --ctrl none          # no levelling, just the throttle ramp
    uv run python watch.py --random             # a new randomized drone every reset
    uv run python watch.py --gains autotune     # PID gains from the last sim autotune

Keys (click the window first):
    SPACE       kick it (random sudden rotation)
    arrows      move the centre of mass 1 mm (like a wire moved) - watch it cope
    R           restart (alternates: take off from the floor / start mid-air tilted)

The terminal prints true roll/pitch, yaw rate, height and the four motor
outputs twice a second. Red nose marker = front; orange props spin CCW, blue CW.
"""

import argparse
import json
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

from pico_drone_sim.controllers import ACT_SCALE, PIDController, PolicyController
from pico_drone_sim.env import HoverEnv
from pico_drone_sim.params import describe

KEY_SPACE, KEY_R = 32, 82
KEY_RIGHT, KEY_LEFT, KEY_DOWN, KEY_UP = 262, 263, 264, 265
AUTOTUNE_GAINS = Path(__file__).resolve().parent / "runs" / "autotune_gains.json"


class Zero:
    def reset(self):
        pass

    def __call__(self, env, dt):
        return np.zeros(3)


def make_controller(args):
    if args.ctrl == "policy":
        from flight import policy_weights

        return PolicyController(policy_weights.LAYERS)
    if args.ctrl == "none":
        return Zero()
    gains = None
    if args.gains == "autotune":
        gains = json.loads(AUTOTUNE_GAINS.read_text())["gains"]
        print("PID gains from", AUTOTUNE_GAINS)
    return PIDController(gains)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ctrl", choices=("pid", "policy", "none"), default="pid")
    ap.add_argument("--gains", choices=("lift", "autotune"), default="lift")
    ap.add_argument("--random", action="store_true", help="randomize the drone on every reset")
    ap.add_argument("--seconds", type=float, default=12.0, help="episode length")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    env = HoverEnv(randomize_scale=1.0 if args.random else 0.0, episode_s=args.seconds,
                   kick_rate_hz=0.0, seed=args.seed)
    ctrl = make_controller(args)
    state = {"reset": True, "ground": True, "kick": False, "com": np.zeros(2)}

    def on_key(key):
        if key == KEY_SPACE:
            state["kick"] = True
        elif key == KEY_R:
            state["reset"] = True
        elif key in (KEY_RIGHT, KEY_LEFT, KEY_UP, KEY_DOWN):
            # Viewed from behind: up = toward the nose (+x), left = +y.
            step = {KEY_UP: (1, 0), KEY_DOWN: (-1, 0), KEY_LEFT: (0, 1), KEY_RIGHT: (0, -1)}[key]
            state["com"] += step
            state["com_changed"] = True

    m, d = env.drone.model, env.drone.data
    with mujoco.viewer.launch_passive(m, d, key_callback=on_key) as viewer:
        viewer.cam.distance = 0.55
        viewer.cam.elevation = -18
        viewer.cam.azimuth = 180
        last_print = 0.0
        done_at = None
        while viewer.is_running():
            if state["reset"]:
                state["reset"] = False
                env.reset(options={"ground": state["ground"], "tilt_deg": 15})
                env.p.com_offset_mm[0] += state["com"][0]
                env.p.com_offset_mm[1] += state["com"][1]
                env.drone.set_params(env.p)
                ctrl.reset()
                print("\n%s start | %s" % ("floor" if state["ground"] else "mid-air",
                                           describe(env.p)))
                state["ground"] = not state["ground"]
                done_at = None
                last_print = -1.0
            if state.pop("com_changed", False):
                env.p.com_offset_mm[0] = env.nominal.com_offset_mm[0] + state["com"][0]
                env.p.com_offset_mm[1] = env.nominal.com_offset_mm[1] + state["com"][1]
                env.drone.set_params(env.p)
                print("CoM offset now x %+.0f mm, y %+.0f mm" % tuple(env.p.com_offset_mm[:2]))
            if state["kick"]:
                state["kick"] = False
                env.drone.kick(env.rng.uniform(-200, 200, 3) * np.array([1.0, 1.0, 0.5]))

            t0 = time.perf_counter()
            if done_at is None:
                cmd = ctrl(env, env.period)
                _, _, term, trunc, info = env.step_cmd(cmd)
                if isinstance(ctrl, PolicyController):
                    env.last_action = np.clip(cmd / ACT_SCALE, -1, 1)
                if term or trunc:
                    print("%s at t=%.1fs - R to restart" % ("TILT LIMIT" if term else "episode over", env.t))
                    done_at = env.t
                if env.t - last_print >= 0.5:
                    last_print = env.t
                    r, p_, _ = env.drone.attitude_deg()
                    o = env.out
                    print("t=%4.1f thr %.2f  roll %+5.1f pitch %+5.1f  yaw %+6.0f dps  z %.2fm  "
                          "m1 %.2f m2 %.2f m3 %.2f m4 %.2f"
                          % (env.t, info["throttle"], r, p_, env.drone.rates_dps()[2],
                             env.drone.position()[2], o[1], o[2], o[3], o[4]))
            viewer.cam.lookat[:] = env.drone.position()
            viewer.sync()
            # Real time: one control step of sim per control period of wall clock.
            spare = env.period - (time.perf_counter() - t0)
            if spare > 0:
                time.sleep(spare)


if __name__ == "__main__":
    main()
