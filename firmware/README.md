# Firmware — attitude-hold flight controller

This is the integrated flight code: sensor fusion, cascaded rate/angle PID
loops, motor mixing, and the safety logic that arms and disarms the motors.
It builds entirely on the drivers and control maths already validated in
`src/` and `testing/` — nothing here is new hardware code, only the loop that
ties it together.

## Read this before you run anything

**This is attitude stabilization, not hover.** It self-levels roll and pitch
and holds a heading, at whatever throttle `tuning.py` sets. It cannot hold
altitude or position, because nothing on this airframe measures either:

- The GPS updates at 1–5 Hz with 1–10 m accuracy — useless for noticing a 10 cm
  drop, let alone correcting one before it matters.
- There is no barometer, no sonar, no optical flow. Nothing on this bench
  measures height at all.

So this will climb, sink, and drift with wind, battery voltage sag, and
airframe asymmetry, even while perfectly level. True hover — holding a fixed
height — needs a barometer at minimum (a BMP280 is a couple of dollars and a
4-wire I2C add-on); this code leaves a clean seam for one but does not fake
having it.

**There is no control link and no physical kill switch.** The only way to stop
a misbehaving motor right now is pulling the battery. Every safety mechanism
below is a software approximation of what a real RC failsafe would do — treat
them as backups, not as a substitute for standing next to the battery
connector with your hand on it.

## Safety model

| Layer | What it does |
|---|---|
| `MotorBank()` construction | Drives `SLP` low before anything else runs — motors are hardware-disarmed the instant the script starts |
| `tuning.LIVE_MOTORS = False` (default) | The entire control loop runs and prints what it *would* do; `arm()` is never called, so nothing can spin regardless of any bug below this line |
| "Type ARM" prompt | Only reached if `LIVE_MOTORS` is `True`. Blocks until answered — no REPL attached means it blocks forever, which is the correct failure mode |
| `TILT_LIMIT_DEG` | Auto-disarms if roll or pitch exceeds this — catches a flip or a bad gain before it grinds into whatever it's mounted on |
| `RUN_SECONDS` | Auto-disarms after a fixed time no matter what — there is no way to command a stop mid-run otherwise |
| `try`/`finally` around the whole loop | Disarms on Ctrl-C, on any exception, and on the abort path — every exit disarms |

That confirmation prompt is a manual `MotorBank()` construction, not
`with MotorBank() as bank:` — the context manager's `__enter__` calls `arm()`
unconditionally, which would have spun the motors *before* the prompt was even
answered. That was a real bug caught by testing the abort path, not by reading
the code — worth remembering if you extend this file.

## Files

| File | Purpose |
|---|---|
| `flight_controller.py` | The control loop. Open it directly in Thonny and run it, same as anything in `testing/` |
| `tuning.py` | Every gain, limit, and the `LIVE_MOTORS` switch — the only file you should need to edit while tuning |

## Setup

1. Complete `testing/01` through `testing/07` first — this assumes every
   component already works individually.
2. Run `testing/05_hmc5883l_compass/` **with the airframe fully assembled**
   and copy the printed `offset`/`scale` into `tuning.py`'s `MAG_OFFSET` /
   `MAG_SCALE`. Skipping this leaves heading-hold steering off whatever
   distortion happens to be nearby.
3. Upload the library, which now includes `tuning.py`:
   ```bash
   ./tools/upload.sh
   ```
4. Open `firmware/flight_controller.py` in Thonny and run it. `LIVE_MOTORS` is
   `False` by default — nothing can spin yet.

## Tuning procedure

Full detail and rationale is in the comments at the top of `tuning.py` — this
is the short version:

1. **Dry run, props off.** Tilt the board by hand and watch the printed mixer
   output. Confirm the *sign* of every axis before anything else: tilt right,
   the roll term should push right-side motors down and left-side up. Get a
   sign wrong here and the first live attempt flips immediately.
2. **Props on, in a restraining rig** that physically cannot leave the ground
   even at full deflection. Set `LIVE_MOTORS = True`, keep `RUN_SECONDS` short.
3. **Rate loop first**, angle gains still at zero. Raise `*_RATE_KP` from zero
   until a hand disturbance makes it oscillate at a fixed frequency, back off
   to roughly half that, then add a little `*_RATE_KD` to damp the wobble.
4. **Angle loop second**, once the rate loop is solid. Small `*_ANGLE_KP`,
   raised until it holds level without overshoot.
5. **Yaw/heading last**, using `HEADING_KP`.

If anything oscillates violently or looks like it's about to flip: set
`LIVE_MOTORS = False` and pull the battery. That's not a tuning problem to
push through — it's the signal to lower gains and re-check the rig.

## Why not autotune, why not RL

**Relay-based autotune** (forcing an oscillation and computing gains from its
period and amplitude — what Betaflight/ArduPilot's "autotune" features do
under the hood) is a legitimate, ML-free technique that could run entirely on
the Pico. It isn't built into this first version because it drives the motors
autonomously to find the oscillation, and this bench has no kill switch faster
than the battery connector. Worth adding once a physical kill switch or a real
control link exists — see `docs/roadmap.md`.

**On-device reinforcement learning** was ruled out outright. Training a
continuous-control policy from scratch typically needs 10⁴–10⁷ real
interactions; MicroPython has no autodiff and the stock firmware has no
`numpy`-equivalent (`ulab` isn't bundled by default); and every failed
training rollout on a real spinning-prop airframe is a potential crash.
Training in simulation and deploying a small pretrained policy for inference
only ("sim-to-real") is possible in principle, but needs an accurate physics
model and system identification of this specific airframe first — a
substantial project on its own, and PID will be flying reliably long before
that pipeline would even be validated.
