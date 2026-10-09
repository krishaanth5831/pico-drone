# Firmware — attitude-hold flight controller

This is the integrated flight code: sensor fusion, cascaded rate/angle PID
loops, motor mixing, and the safety logic that arms and disarms the motors.
It builds entirely on the drivers and control maths already validated in
`src/` and `testing/` — nothing here is new hardware code, only the loop that
ties it together.

## Wiring — every component, in one place

This mirrors [`docs/pinout.md`](../docs/pinout.md) and the per-component
tables in `testing/`, gathered here so the full airframe wiring is on one
page once you're past bench-testing individual parts and wiring the real
thing. `src/config.py` is the authoritative source if this ever drifts from
it — pins are imported from there, never hardcoded.

Hold the Pico 2 W with the **USB port at the top**. Pin 1 is the top-left
pad; numbers run down the left side (1–20), then continue up the right side
(21–40).

### Pico 2 W — onboard LED

| Signal | Pico 2 W | Physical pin | Note |
|---|---|---|---|
| Onboard LED | `Pin("LED")` | — | Behind the CYW43 WiFi chip on "W" boards — on/off only, cannot be PWM'd |

`GP23`, `GP24`, `GP25`, `GP29` are wired to the CYW43 WiFi/Bluetooth chip and
are **reserved** — never assign these. They are internal only; physical pins
29, 31, 32 and 34 are GP22, GP26, GP27 and GP28.

### DRV8833 #1 (motors 1 and 2)

| DRV8833 pin | Pico 2 W | Physical pin | Note |
|---|---|---|---|
| `VM` / `VCC` / `VMOT` | Bench: HW-131 5 V rail. Airframe: battery + direct | — | Never from the Pico |
| `GND` | Bench: HW-131 GND rail. Airframe: battery star point | — | Motor current returns here, not through the Pico |
| `SLP` / `nSLEEP` / `EEP` | GP15 | 20 | Shared with DRV #2 — hardware arm/disarm for all four motors |
| `AIN1` | GP10 | 14 | Motor 1 PWM |
| `AIN2` | GND | — | Tie low — channel A unidirectional |
| `BIN1` | GP11 | 15 | Motor 2 PWM |
| `BIN2` | GND | — | Tie low — channel B unidirectional |
| `nFAULT` *(if present)* | GP14 | 19 | Shared with DRV #2, open-drain, low = fault |
| `AOUT1`/`AOUT2` | Motor 1 leads | — | Polarity picks spin direction, not software |
| `BOUT1`/`BOUT2` | Motor 2 leads | — | Polarity picks spin direction, not software |

### DRV8833 #2 (motors 3 and 4)

Identical to #1, sharing the same `SLP`, `VM`, and `GND`:

| DRV8833 pin | Pico 2 W | Physical pin | Note |
|---|---|---|---|
| `VM` | Same as DRV #1 | — | |
| `GND` | Same as DRV #1 | — | |
| `SLP` | GP15 | 20 | Same net as DRV #1 |
| `AIN1` | GP12 | 16 | Motor 3 PWM |
| `AIN2` | GND | — | Tie low |
| `BIN1` | GP13 | 17 | Motor 4 PWM |
| `BIN2` | GND | — | Tie low |
| `nFAULT` | GP14 | 19 | Same net as DRV #1 |
| `AOUT1`/`AOUT2` | Motor 3 leads | — | |
| `BOUT1`/`BOUT2` | Motor 4 leads | — | |

### Motors — airframe positions

Standard X quad, viewed from above, nose up the page. Diagonal pairs share a
rotation direction so their yaw torques cancel in the hover.

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

| Motor | Position | Rotation | Driver channel | PWM pin |
|---|---|---|---|---|
| M1 | front-right | CCW | DRV #1 ch A | GP10 |
| M2 | rear-left | CCW | DRV #1 ch B | GP11 |
| M3 | front-left | CW | DRV #2 ch A | GP12 |
| M4 | rear-right | CW | DRV #2 ch B | GP13 |

### GY-521 (MPU6050) — IMU, on I2C0

| GY-521 pin | Pico 2 W | Physical pin | Note |
|---|---|---|---|
| `VCC` | 3V3(OUT) | 36 | |
| `GND` | GND | 38 | |
| `SDA` | GP4 | 6 | I2C0 data, the only device on the bus |
| `SCL` | GP5 | 7 | I2C0 clock |
| `XDA`, `XCL`, `ADO`, `INT` | — | — | Leave unconnected (`ADO` floating = address `0x68`) |

Mount as close to the airframe's centre of mass as possible, and decouple
from vibration (foam tape) — bolted rigidly to the frame, prop vibration
feeds straight into the accelerometer and corrupts the attitude estimate.

### HMC5883L — compass: removed from the build

Taken out on 2026-10-09. With it attached the IMU dropped off I2C whenever the
motors ran — most likely the breadboard-rail grounding fault later found with
the GPS (see the grounding rule above), not the compass itself. It stays out:
nothing needs a heading yet, and a few cm from four motors its readings are
swamped anyway. `flight_controller.py` detects that it is missing and holds
yaw by the gyro rate loop alone, so the heading drifts slowly. See
[`testing/05_hmc5883l_compass/`](../testing/05_hmc5883l_compass/README.md) if
it comes back.

### GY-GPS6MV2 (NEO-6M) — GPS, on UART0

| GY-GPS6MV2 pin | Pico 2 W | Physical pin | Note |
|---|---|---|---|
| `VCC` | Bench: VBUS. Battery: 3V3(OUT) | 40 / 36 | VBUS is 5 V from USB into the module's own regulator — it gets a lock there and stays off the IMU's 3.3 V line |
| `GND` | GND | 3 | Directly below GP0/GP1, keeps the wire short |
| `TX` | GP1 = UART0 RX | 2 | **Crossover** — GPS transmit to Pico receive |
| `RX` | GP0 = UART0 TX | 1 | **Crossover**, only needed to reconfigure the module |

Check the board's silkscreen before wiring — these ship in `VCC RX TX GND`
and `GND TX RX VCC` header orders depending on the batch; the label is
authoritative, not the pin position. Mount with the ceramic antenna facing
up, clear view of sky, away from motors and power wiring.

### Power — bench (current setup, props off)

No battery yet. The Pico runs from the laptop's USB; the motors from an HW-131
breadboard power supply. Details and limits in
[`docs/power.md`](../docs/power.md#bench-power-hw-131-breadboard-supply).

```
USB charger (>=2 A) -> HW-131 --> rail A, jumper 5 V --> DRV #1 VM, DRV #2 VM
                              --> rail B, jumper OFF
                              --> GND rail <--+-- DRV #1 GND, DRV #2 GND
                                              +-- ONE wire to Pico GND

Laptop USB -> Pico 2 W --> 3V3 OUT (pin 36) --> GY-521 VCC
                       --> VBUS    (pin 40) --> GPS VCC
                       --> GND (38) -> GY-521 GND,  GND (3) -> GPS GND
```

**Grounding rule:** once the HW-131 is plugged in, the breadboard's power rails
are motor ground. Sensor grounds go straight to Pico GND pins (GY-521 → 38,
GPS → 3), never to a rail; exactly one wire joins Pico GND to the HW-131 GND
rail; the HW-131 runs off a wall charger, not the laptop. Break this and the IMU
drops off I2C the moment the motors spin. Full explanation and a multimeter
check in [`docs/power.md`](../docs/power.md#grounding-rule--breadboard-rails-are-motor-ground).

### Power — airframe (battery)

```
1S LiPo (+) --+-------------------> DRV #1 VM --+-- 470uF -- GND
              |                                  |
              +-------------------> DRV #2 VM --+-- 470uF -- GND
              |
              +--[ SS14 Schottky ]-> Pico VSYS (physical pin 39)

1S LiPo (-) ---- star point ----+--> DRV #1 GND
                                +--> DRV #2 GND
                                +--> Pico GND (physical pin 38)
```

| Connection | Pico 2 W | Physical pin | Note |
|---|---|---|---|
| Battery + via Schottky | VSYS | 39 | VSYS accepts 1.8–5.5 V |
| Battery − (star ground) | GND | 38 | |
| Battery + direct | — | — | To both DRV8833 `VM` pins, not to the Pico |
| *(bench only)* USB 5 V | VBUS | 40 | Alternative driver supply for bench work |

**Never connect battery + to 3V3 (pin 36)** — that regulator only supplies
~300 mA, nowhere near enough for a motor.

The 470 µF capacitors sit right at each driver's `VM`/`GND` pins as a local
energy reservoir — the battery chemistry can't respond to a millisecond
current spike, the capacitor can. The Schottky diode on VSYS stops a motor's
current surge from browning out the Pico through the shared battery rail.

### I2C addresses (I2C0, GP4/GP5)

| Device | Address |
|---|---|
| MPU6050 / GY-521 | `0x68` (`0x69` with `ADO` high) — the only device on the bus |

### Full wiring diagrams and mounting detail

Each `testing/0X_*/README.md` covers its component in more depth — soldering
gotchas, why AIN2/BIN2 are tied low, why the GPS crossover trips people up,
what a `nFAULT` low actually means. Work through those in order
(`testing/README.md` is the index) before wiring the full airframe from this
page.

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
| `lift.py` | Standalone lift test: staggered spin-up, ramp, hold until Stop, with self-levelling (`LEVELLING`, level taken from how it sits at start) or bench mode (all motors equal). Gains and the `WIRING` mode live inline at the top, so Thonny edits take effect without re-uploading |
| `imu_motor_check.py` | Runs each motor on its own while hammering the IMU and counts failed reads — finds which motor (or which power/ground path) knocks the IMU off the bus |

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
