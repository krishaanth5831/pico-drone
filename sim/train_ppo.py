"""
Train the levelling policy with PPO (Stable-Baselines3) across randomized drones.

    cd sim
    uv run python train_ppo.py                       # 2 x 32 network, 6M steps (~20-30 min)
    uv run python train_ppo.py --hidden 16           # smaller, if the Pico benchmark says so
    uv run python train_ppo.py --finetune runs/ppo_X/best.zip --steps 1000000
                                                     # after drone_params.json changed (autotune)

Progress prints every update (mean episode reward and length: 6 s at ~100 Hz
is ~600 steps, so a length near 600 means it stops falling over). TensorBoard:
    uv run tensorboard --logdir runs

Output in runs/ppo_<time>/: best.zip (best evaluation), final.zip. Then
    uv run python export_policy.py runs/ppo_<time>/best.zip
writes src/flight/policy_weights.py for the board, and
    uv run python watch.py --ctrl policy
shows it flying.

Curriculum: the randomization ranges start at 30 % of drone_params.json's and
widen to 100 % over the first 40 % of training - learning to hover one
drone first, then all of them, is much faster than starting with chaos.
"""

import argparse
import time
from pathlib import Path

import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback, EvalCallback
from stable_baselines3.common.vec_env import SubprocVecEnv, VecMonitor

from pico_drone_sim.env import HoverEnv

RUNS = Path(__file__).resolve().parent / "runs"


def make_env(rank, seed):
    def _init():
        return HoverEnv(randomize_scale=0.3, seed=seed + rank)
    return _init


class Curriculum(BaseCallback):
    """Widen domain randomization from `start` to 1.0 over the first `frac` of training."""

    def __init__(self, total, start=0.3, frac=0.4):
        super().__init__()
        self.total, self.start, self.frac = total, start, frac
        self._last = None

    def _on_step(self):
        p = min(1.0, self.num_timesteps / (self.total * self.frac))
        scale = round(self.start + (1.0 - self.start) * p, 2)
        if scale != self._last:
            self.training_env.set_attr("randomize_scale", scale)
            self._last = scale
            self.logger.record("curriculum/randomize_scale", scale)
        return True


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--hidden", type=int, default=32, help="units in each of the 2 hidden layers")
    ap.add_argument("--steps", type=int, default=6_000_000)
    ap.add_argument("--envs", type=int, default=12)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--finetune", type=Path, help="continue from this .zip (full randomization)")
    args = ap.parse_args()

    torch.set_num_threads(4)
    out = RUNS / time.strftime("ppo_%Y%m%d_%H%M%S")
    out.mkdir(parents=True, exist_ok=True)
    print("writing to", out)

    # fork: workers share the parent's already-imported torch/mujoco pages
    # instead of each loading their own copy (keeps RAM low).
    venv = VecMonitor(SubprocVecEnv([make_env(i, args.seed * 1000) for i in range(args.envs)],
                                    start_method="fork"))
    eval_env = VecMonitor(SubprocVecEnv([make_env(100 + i, 999) for i in range(4)], start_method="fork"))
    eval_env.set_attr("randomize_scale", 1.0)

    if args.finetune:
        model = PPO.load(args.finetune, env=venv, device="cpu")
        venv.set_attr("randomize_scale", 1.0)
        callbacks = []
    else:
        model = PPO(
            "MlpPolicy", venv, device="cpu", seed=args.seed, verbose=1,
            n_steps=1024, batch_size=4096, n_epochs=10, learning_rate=3e-4,
            gamma=0.99, gae_lambda=0.95, clip_range=0.2, ent_coef=0.0,
            policy_kwargs={
                "net_arch": {"pi": [args.hidden, args.hidden], "vf": [64, 64]},
                "activation_fn": torch.nn.Tanh,
                "log_std_init": -1.0,
            },
            tensorboard_log=str(RUNS),
        )
        callbacks = [Curriculum(args.steps)]

    callbacks.append(EvalCallback(eval_env, n_eval_episodes=24, eval_freq=max(1, 100_000 // args.envs),
                                  best_model_save_path=str(out), deterministic=True, verbose=1))
    model.learn(total_timesteps=args.steps, callback=callbacks, tb_log_name=out.name,
                reset_num_timesteps=not args.finetune)
    model.save(out / "final.zip")
    best = out / "best_model.zip"
    if best.exists():
        best.rename(out / "best.zip")
    print("\ndone:", out)
    print("next: uv run python export_policy.py %s" % (out / "best.zip").relative_to(RUNS.parent))


if __name__ == "__main__":
    main()
