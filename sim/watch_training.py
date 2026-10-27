"""
Watch PPO learn to level the drone, live, in a MuJoCo window.

    cd sim
    uv run python watch_training.py                 # 2 minutes from scratch
    uv run python watch_training.py --minutes 5

Training runs in the background (worker processes, like train_ppo.py, with
smaller rollouts so the policy changes every few seconds). The window flies
the CURRENT policy - refreshed after every update - on a fresh randomized
drone each episode, so you see both the learning and the domain
randomization:

  red ball      the drone's real centre of mass (wires moved it)
  blue arrow    the weight relief (the balloon) - longer = more help needed
  orange arrows each motor's thrust right now (motor spread, lag, sag)
  top left      this drone's randomized parameters
  bottom left   the learning curve: mean episode reward per update

Episodes start on the floor (lift-off ramp) or mid-air tilted up to 20 deg,
and random kicks hit the drone, exactly as in training. Early on it falls
over; within a couple of minutes it should start catching itself.

This is a demo: it trains a throwaway policy. The real one comes from
train_ppo.py (which this does not touch).
"""

import argparse
import multiprocessing as mp
import os
import queue
import signal
import sys
import time

import mujoco
import mujoco.viewer
import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.vec_env import SubprocVecEnv, VecMonitor

from pico_drone_sim.env import HoverEnv
from pico_drone_sim.params import G

MOTORS = (1, 2, 3, 4)


def make_env(rank):
    def _init():
        return HoverEnv(randomize_scale=1.0, seed=rank)
    return _init


def policy_kwargs(hidden):
    return {"net_arch": {"pi": [hidden, hidden], "vf": [64, 64]},
            "activation_fn": torch.nn.Tanh, "log_std_init": -1.0}


class Snapshot(BaseCallback):
    """After every rollout: send the policy weights and the mean reward to the window."""

    def __init__(self, out, deadline):
        super().__init__()
        self.out, self.deadline = out, deadline
        self.rewards = []
        self.version = 0

    def _on_step(self):
        for info in self.locals["infos"]:
            if "episode" in info:
                self.rewards.append(info["episode"]["r"])
        return time.time() < self.deadline

    def _on_rollout_end(self):
        self.version += 1
        weights = {k: v.detach().cpu().clone() for k, v in self.model.policy.state_dict().items()}
        reward = float(np.mean(self.rewards)) if self.rewards else None
        self.rewards = []
        self.out.put(("policy", self.version, self.num_timesteps, reward, weights))


def train(out, minutes, envs, hidden):
    """Runs in its own process, so the window's drawing never slows it down."""
    torch.set_num_threads(2)
    venv = VecMonitor(SubprocVecEnv([make_env(i) for i in range(envs)], start_method="fork"))
    model = PPO("MlpPolicy", venv, device="cpu", verbose=0,
                n_steps=256, batch_size=envs * 256, n_epochs=5, learning_rate=3e-4,
                gamma=0.99, gae_lambda=0.95, clip_range=0.2, policy_kwargs=policy_kwargs(hidden))
    cb = Snapshot(out, time.time() + minutes * 60.0)
    # terminate() from the window -> SystemExit here -> the finally closes the
    # env workers instead of orphaning them.
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    try:
        model.learn(total_timesteps=10**9, callback=cb)
        out.put(("done", cb.version, cb.num_timesteps, None, None))
    finally:
        venv.close()


def add_arrow(scn, a, b, width, rgba):
    if scn.ngeom >= scn.maxgeom:
        return
    g = scn.geoms[scn.ngeom]
    mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_ARROW, np.zeros(3), np.zeros(3), np.zeros(9),
                        np.array(rgba, dtype=np.float32))
    mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_ARROW, width, np.asarray(a, float), np.asarray(b, float))
    scn.ngeom += 1


def add_ball(scn, pos, size, rgba):
    if scn.ngeom >= scn.maxgeom:
        return
    g = scn.geoms[scn.ngeom]
    mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_SPHERE, np.array([size, 0, 0]), np.asarray(pos, float),
                        np.eye(3).flatten(), np.array(rgba, dtype=np.float32))
    scn.ngeom += 1


def draw_overlays(viewer, env):
    d, m = env.drone.data, env.drone.model
    body = env.drone.body
    scn = viewer.user_scn
    scn.ngeom = 0
    com = d.xipos[body].copy()
    R = d.xmat[body].reshape(3, 3)
    add_ball(scn, com, 0.006, (1, 0.1, 0.1, 1))
    # Relief force: 1 N drawn as 0.25 m.
    add_arrow(scn, com, com + np.array([0, 0, 0.25 * env.p.relief_n]), 0.004, (0.2, 0.5, 1, 0.9))
    for n in MOTORS:
        p = d.site_xpos[m.site("m%d" % n).id]
        thrust = float(d.ctrl[n - 1])
        add_arrow(scn, p, p + R[:, 2] * 0.25 * thrust * 4, 0.003, (1, 0.55, 0.1, 0.9))


def drone_text(env, ep, how):
    p = env.p
    left = "\n".join([
        "mass", "CoM offset", "motor thrust", "motor spread", "motor lag up/down", "hover throttle",
        "balloon relief", "loop rate / latency", "IMU tilt error", "start", "last episode"])
    right = "\n".join([
        "%.0f g (%.0f g wires)" % (p.mass_g, p.extra_mass_g),
        "%+.1f %+.1f %+.1f mm" % tuple(p.com_offset_mm),
        "%.1f g each at full" % p.thrust_full_g,
        " ".join("%.2f" % s for s in p.motor_scale),
        "%.0f / %.0f ms" % (p.motor_tau_up_ms, p.motor_tau_down_ms),
        "%.2f" % p.hover_throttle,
        "%.0f g" % (p.relief_n / G * 1000.0),
        "%.0f Hz / %.1f ms" % (p.control_hz, p.latency_ms),
        "%+.1f %+.1f deg" % tuple(p.imu_misalign_deg),
        "floor take-off" if env.ground else "mid-air, tilted",
        how,
    ])
    return left, right, ep


def curve_figure(curve):
    fig = mujoco.MjvFigure()
    mujoco.mjv_defaultFigure(fig)
    fig.title = "mean episode reward per update"
    fig.flg_extend = 1
    fig.figurergba[3] = 0.5
    fig.gridsize[0] = 4
    fig.gridsize[1] = 4
    pts = curve[-500:]
    fig.linepnt[0] = len(pts)
    for i, y in enumerate(pts):
        fig.linedata[0][2 * i] = i
        fig.linedata[0][2 * i + 1] = y
    fig.range[0][0], fig.range[0][1] = 0, max(10, len(pts))
    return fig


def display(inbox, hidden, trainer):
    env = HoverEnv(randomize_scale=1.0, seed=12345)
    env.reset()
    # Same network as the trainer's; weights arrive through `inbox`.
    policy = PPO("MlpPolicy", HoverEnv(), device="cpu", policy_kwargs=policy_kwargs(hidden)).policy
    policy.eval()
    have_policy = False
    version, steps, curve, done = 0, 0, [], False
    with mujoco.viewer.launch_passive(env.drone.model, env.drone.data,
                                      show_left_ui=False, show_right_ui=False) as viewer:
        viewer.cam.distance = 0.7
        viewer.cam.elevation = -20
        ep, how = 0, "-"
        last_fig = last_text = 0.0
        while viewer.is_running():
            try:
                while True:
                    kind, version, steps, reward, weights = inbox.get_nowait()
                    if kind == "done":
                        done = True
                        print("training stopped: %s steps, %d updates" % (f"{steps:,}", version), flush=True)
                        continue
                    if reward is not None:
                        curve.append(reward)
                    if version % 5 == 0:
                        print("update %4d  %9s steps  mean episode reward %s"
                              % (version, f"{steps:,}", "%.0f" % reward if reward is not None else "-"),
                              flush=True)
                    policy.load_state_dict(weights)
                    have_policy = True
            except queue.Empty:
                pass
            t0 = time.perf_counter()
            obs = env._obs()
            if not have_policy:
                action = np.zeros(3)
            else:
                with torch.no_grad():
                    action, _ = policy.predict(obs, deterministic=True)
            _, _, term, trunc, _ = env.step(action)
            if term or trunc:
                how = ("CRASHED at %.1f s" % env.t) if term else ("held level %.0f s" % env.t)
                ep += 1
                env.reset()
            with viewer.lock():
                draw_overlays(viewer, env)
                viewer.cam.lookat[:] = env.drone.position()
            # set_texts / set_figures take the viewer lock themselves: calling
            # them inside `with viewer.lock()` deadlocks the window.
            if time.time() - last_text > 0.1:
                left, right, _ = drone_text(env, ep, how)
                status = ("TRAINING  update %d  |  %s env steps  |  episode %d on screen"
                          % (version, f"{steps:,}", ep)) if not done else "TRAINING STOPPED - close the window"
                viewer.set_texts([
                    (mujoco.mjtFont.mjFONT_NORMAL, mujoco.mjtGridPos.mjGRID_TOPLEFT, left, right),
                    (mujoco.mjtFont.mjFONT_BIG, mujoco.mjtGridPos.mjGRID_TOP, status, ""),
                ])
                last_text = time.time()
            if curve and time.time() - last_fig > 0.5:
                vp = viewer.viewport
                w, h = (vp.width, vp.height) if vp is not None else (1200, 900)
                rect = mujoco.MjrRect(0, 0, w // 3, h // 3)
                viewer.set_figures([(rect, curve_figure(curve))])
                last_fig = time.time()
            viewer.sync()
            time.sleep(max(0.0, env.period - (time.perf_counter() - t0)))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--minutes", type=float, default=2.0, help="how long to train")
    ap.add_argument("--envs", type=int, default=8)
    ap.add_argument("--hidden", type=int, default=32)
    args = ap.parse_args()

    torch.set_num_threads(1)
    # spawn, not fork: the trainer must not inherit the window's GL state.
    ctx = mp.get_context("spawn")
    inbox = ctx.Queue()
    trainer = ctx.Process(target=train, args=(inbox, args.minutes, args.envs, args.hidden), daemon=False)
    trainer.start()
    print("window open - training for %.1f min (close the window to quit early)" % args.minutes, flush=True)
    try:
        display(inbox, args.hidden, trainer)
    finally:
        if trainer.is_alive():
            trainer.terminate()
        trainer.join(10)
    # Skip interpreter teardown: MuJoCo's passive viewer segfaults there
    # (exit code 139) after the window is already gone. Nothing left to flush.
    sys.stdout.flush()
    os._exit(0)


if __name__ == "__main__":
    main()
