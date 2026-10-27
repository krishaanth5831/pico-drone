# sim/ — MuJoCo model, PPO training, PID autotune

A simulated pico-drone for training a reinforcement-learning (PPO) levelling
policy and for checking the PX4-style autotune before it runs on the real
drone. The board's own control code (`src/flight/`: complementary filter, PID,
mixer, policy runner) is imported and run unchanged, so what you train and
tune here is the code that flies.

## Setup (once)

```bash
cd sim
uv sync            # Python 3.12 + MuJoCo + Stable-Baselines3 + CPU torch, in sim/.venv
```

The firmware tests in the repo root don't need any of this.

## Watch it (opens a window)

```bash
cd sim
uv run python watch.py                 # lift.py's PID on the nominal drone
uv run python watch.py --random        # a different randomized drone every R
uv run python watch.py --ctrl none     # no levelling: see why it is needed
```

Keys (click the window first): **SPACE** kick it, **arrows** move the centre
of mass 1 mm (a wire moved), **R** restart (alternates floor take-off and a
tilted mid-air start). The terminal prints true roll/pitch, yaw rate, height
and the four motor outputs.

## Watch it learn (opens a window)

```bash
cd sim
uv run python watch_training.py              # 2 minutes of PPO from scratch, live
uv run python watch_training.py --minutes 5
```

Trains a throwaway policy in a background process and flies the current one,
refreshed after every update, on a fresh randomized drone each episode. On
screen: the drone's randomized parameters (top left), a learning curve
(bottom left), the real centre of mass (red ball), the balloon's lift (blue
arrow) and each motor's thrust (orange arrows). It starts out falling over
and should be catching itself within a couple of minutes. It needs free
cores, so pause a running `train_ppo.py` first, or both crawl.

## Train the real policy

```bash
cd sim
uv run python train_ppo.py                   # 6M steps, 2 x 32 network
uv run tensorboard --logdir runs             # curves
```

The mean episode reward tops out around 600 (6 s at 100 Hz, level the whole
time). Output goes to `runs/ppo_<time>/best.zip`.

## Autotune in the sim (opens a window with --windowed)

```bash
cd sim
uv run python autotune_sim.py --windowed     # watch the roll/pitch/yaw wiggles, then the new gains
uv run python autotune_sim.py --check 12     # 12 random + 6 extreme drones vs lift.py's gains
```

This runs `src/flight/autotune.py`, the same file the board runs. It prints
what it identified next to the sim's ground truth, the gain scale it picked
per axis, and lift.py vs autotuned (with trim) under kicks. Last check:
0 of 18 drones aborted, median K error 6 %, median trim error 0.001,
autotuned better on 9 and worse on 0. `--reference` regenerates
`autotune.REFERENCE` after `drone_params.json` changes.

## What is modelled

| Thing | How | Why it matters |
|---|---|---|
| Airframe | the printed frame and GPS mast STLs from `hardware/frame/`, plus every part as the Onshape fit-check dummy at its real place and mass (`pico_drone_sim/airframe.py`, a port of `pico_drone_frame.fs`). MuJoCo computes mass, CoM and the full inertia from them: 64.4 g without wires (matches Onshape), CoM 2.0 mm forward and 2.7 mm above the deck with the GPS mast | inertia sets how fast it tips; the mast raises the CoM |
| Wires | `extra_mass_g` (randomized 2–20 g) in the wiring bay, plus `com_offset_mm` (±4 mm sideways) for how far they pull the CoM | the CAD has no wires |
| 720 coreless motors | thrust = curve(throttle) with a dead band, first-order spin-up/down lag, ±15 % per-motor spread, supply sag | the lag is what limits gains |
| Prop drag torque | per-motor yaw torque, CCW props push the body CW | yaw control |
| Weight relief | a constant upward force at the CoM sized so the drone hovers at `hover_throttle` | stands in for the helium balloon: the 720s can't lift ~80 g alone |
| MPU6050 | noise, vibration noise growing with throttle, residual gyro bias, 44 Hz DLPF, ±2° mounting error | the policy only sees what the board sees |
| Board loop | 80–130 Hz with jitter, 1–8 ms compute latency before the PWM write | matches the MicroPython loop |

`gps_fitted` / `mag_fitted` in `drone_params.json` add or remove the mast
parts (the compass is off, as on the real drone).

Every number lives in `drone_params.json` with the range it is randomized
over during training. Nothing has been weighed yet, so ranges are wide.

## Frames and signs

FLU body frame, which is exactly the GY-521's frame (chip up, X arrow to the
nose): +roll right side down, +pitch nose down, +yaw counter-clockwise from
above. The motor physics in `pico_drone_sim/drone.py` is written from prop
spin directions alone, independent of `flight/mixer.py`, so the sim can catch
a sign error in the mixer. It did: the mixer's yaw column was reversed (fixed
in 712169c).
