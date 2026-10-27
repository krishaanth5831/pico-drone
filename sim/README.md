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

## What is modelled

| Thing | How | Why it matters |
|---|---|---|
| Frame | 84 mm motor pitch X quad, mass and inertia from `docs/power.md`'s weight budget | inertia sets how fast it tips |
| CoM offset | `com_offset_mm` | wires and the GPS mast pull it off centre |
| 720 coreless motors | thrust = curve(throttle) with a dead band, first-order spin-up/down lag, ±15 % per-motor spread, supply sag | the lag is what limits gains |
| Prop drag torque | per-motor yaw torque, CCW props push the body CW | yaw control |
| Weight relief | a constant upward force at the CoM sized so the drone hovers at `hover_throttle` | stands in for the helium balloon: the 720s can't lift ~80 g alone |
| MPU6050 | noise, vibration noise growing with throttle, residual gyro bias, 44 Hz DLPF, ±2° mounting error | the policy only sees what the board sees |
| Board loop | 80–130 Hz with jitter, 1–8 ms compute latency before the PWM write | matches the MicroPython loop |

Every number lives in `drone_params.json` with the range it is randomized
over during training. Nothing has been weighed yet, so ranges are wide.

## Frames and signs

FLU body frame, which is exactly the GY-521's frame (chip up, X arrow to the
nose): +roll right side down, +pitch nose down, +yaw counter-clockwise from
above. The motor physics in `pico_drone_sim/drone.py` is written from prop
spin directions alone, independent of `flight/mixer.py`, so the sim can catch
a sign error in the mixer. It did: the mixer's yaw column was reversed (fixed
in 712169c).
