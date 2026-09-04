# 03 — Coreless motors

First test that actually spins something. Confirms each motor runs, identifies
which motor is which, and confirms rotation direction against the airframe
layout.

> **Props off.** A loose coreless motor with a prop fitted will fling itself
> across the bench. Props go on only after directions are confirmed.

## What you need

- Passing `02_drv8833`
- 1–4 coreless motors
- The motors physically restrained — taped down or held in a frame

## Wiring

Same as `02_drv8833`, plus the motor leads:

| Motor | Driver | Driver outputs | PWM from | Physical pin |
|---|---|---|---|---|
| **M1** front-right, CCW | DRV #1 ch A | `AOUT1`, `AOUT2` | GP10 | **14** |
| **M2** rear-left, CCW | DRV #1 ch B | `BOUT1`, `BOUT2` | GP11 | **15** |
| **M3** front-left, CW | DRV #2 ch A | `AOUT1`, `AOUT2` | GP12 | **16** |
| **M4** rear-right, CW | DRV #2 ch B | `BOUT1`, `BOUT2` | GP13 | **17** |

Motor lead polarity does not matter electrically — it only decides which way the
motor turns. That is exactly how you set rotation direction.

## Airframe layout

Standard X quad, viewed from above, nose pointing up the page. Diagonal pairs
share a rotation direction so their yaw torques cancel in the hover.

```
      M3 (CW)          M1 (CCW)
        \                 /
         \   +--------+  /
          +--|  PICO  |-+
             |   IMU  |
          +--|        |-+
         /   +--------+  \
        /                 \
      M2 (CCW)          M4 (CW)
```

## Upload the library first

This script imports `config`, `drivers` and `flight`. **Those imports resolve
against the Pico's filesystem, not your computer's** — opening the file in an
editor is not enough. Without this step you get:

```
ImportError: no module named 'config'
```

Upload once, and it stays there until you overwrite it:

- **Thonny** — `View → Files` so both panes show. In the top (computer) pane
  select `config.py`, `drivers` and `flight`, right-click → **Upload to /**.
- **Terminal** — `./tools/upload.sh` (needs `pip install mpremote`).

Verify with `./tools/upload.sh --list`, or just look at the bottom pane in
Thonny — you should see `config.py`, `drivers/` and `flight/` at the root.

Re-upload whenever you change anything under `src/`.

## Running the test

Run `test_coreless_motor.py`. It spins each motor alone, in order, with a ramp
rather than a step.

### Why it ramps instead of stepping

A coreless motor's inrush at zero rpm is several times its running current. On a
1S pack that current spike sags the rail hard enough to reset the Pico. Ramping
gives the battery time to respond. This is also why the flight code slew-limits
motor output.

## What you should see

```
=== coreless motor test ===
MAX_DUTY 0.70, props must be OFF
LED pulses for as long as this runs

motor 1 (front-right, expect CCW)
  ramping up... holding... ramping down
motor 2 (rear-left, expect CCW)
  ...
=== disarmed ===
```

**Watch the motors, one at a time.** For each one confirm:

1. It spins when its own number is printed — and no other motor moves.
2. It spins the direction the layout diagram calls for.

Mark the direction on each motor with a pen as you confirm it.

## Fixing a wrong direction

**Swap that motor's two output wires.** Never in software — the mixer in
`src/flight/mixer.py` assumes the directions in the diagram above, and changing
signs there to compensate will break yaw control in a way that is very hard to
diagnose later.

**The onboard LED pulses throughout.** If it stops and restarts, the board reset — see [07](../07_lipo_power/README.md).

## If it fails

| Symptom | Cause |
|---|---|
| Only `MPY: soft reboot` printed, twice, nothing else | `main.py` is on the board and is hijacking every soft-reboot before your script runs. Run `./tools/upload.sh` to remove it |
| Buzzes but does not turn | Duty below the motor's start threshold. Raise `MIN_START` in `src/config.py` to 0.30 |
| Wrong motor spins | Output leads swapped between channels. Check `AOUT` vs `BOUT` |
| Nothing spins at all | Go back to `02_drv8833` — this is a driver fault, not a motor fault |
| Pico resets when a motor starts | Brownout. See `07_lipo_power` |
| Motor gets hot fast | Coreless motors have no iron to sink heat. Lower `MAX_DUTY` and keep runs short |
