# pico-drone

A flight controller for a small coreless quadcopter, built on a **Raspberry Pi
Pico 2 W** and written in MicroPython. The repo also has a MuJoCo simulator of
the same drone. The simulator runs the board's own control code, so you can
fly the drone with a keyboard, train a reinforcement-learning (PPO) levelling
policy, and try a PX4/QGroundControl-style autotune before anything spins on
the bench.

> **Status: not flight-tested.** The drone has never flown.
>
> - **Bench-run, props off:** the drivers and the component tests in `testing/`,
>   and `firmware/lift.py` on the bench power supply.
> - **Never run on a board:** the autotune flight script (`firmware/autotune.py`),
>   the trained policy, and the ground station over WiFi. They are tested only
>   under CPython with a mocked hardware layer, and in the simulator.
>
> There is no RC link and no physical kill switch. The WiFi ground station can
> DISARM, but only in software. The roadmap is in [`docs/roadmap.md`](docs/roadmap.md).

## What's in here

- **Drivers** for the DRV8833 motor drivers, the GY-521 (MPU6050) IMU, the
  HMC5883L/QMC5883L compass and the NEO-6M GPS. Pins come from one file, `src/config.py`.
- **A hardware disarm.** One GPIO holds both motor drivers asleep, so a software
  bug cannot spin a motor (see [Safety model](#safety-model)).
- **Control maths:** a complementary filter, a rate/angle PID and an X-quad
  mixer with desaturation, all in `src/flight/`.
- **Bench tests**, one folder per component, run in order from Thonny
  ([`testing/`](testing/README.md)).
- **Firmware scripts** that run on the board: a lift test, an attitude-hold
  controller and an autotune flight ([`firmware/`](firmware/README.md)).
- **A MuJoCo simulator** built from the CAD parts. It has keyboard flight,
  PPO training, an autotune check and a SITL vehicle ([`sim/`](sim/README.md)).
- **A small ground station** in the spirit of QGroundControl, with an
  autotune page ([`gcs/`](gcs/README.md)).
- **A 3D-printed frame**: STLs plus the Onshape FeatureScript that generates
  them ([`hardware/frame/`](hardware/frame/README.md)).

## Bill of materials

The parts below come from the docs in this repo. Weights and thrust figures
are in [`docs/power.md`](docs/power.md).

| Part | Qty | Role / note |
|---|---|---|
| Raspberry Pi Pico 2 W | 1 | Flight controller (RP2350) with WiFi for the ground station |
| DRV8833 breakout | 2 | Dual H-bridge, two motors each. `MAX_DUTY` is capped at 0.70 because two chips overheat at full throttle |
| Coreless motors + 55 mm props | 4 | The bench build uses **720** motors, but they cannot lift the drone alone (about 61 g usable thrust). [`docs/power.md`](docs/power.md#thrust-to-weight) recommends **8520** motors. The sim and the autotune assume 720s with helium balloons carrying part of the weight |
| GY-521 (MPU6050) | 1 | Gyro and accelerometer on I2C0, the only device on the bus |
| GY-GPS6MV2 (NEO-6M) | 1 | GPS on UART0. Optional: the docs advise building v1 without it to save weight |
| HMC5883L compass | 0 | **Removed from the build.** The driver and test are kept in case it comes back |
| 1S LiPo, ~600 mAh | 1 | Airframe power (planned; the bench uses a supply) |
| SS14 Schottky diode | 1 | Battery to Pico VSYS, so USB cannot backfeed the LiPo |
| 470 µF capacitor | 2 | One at each driver's VM/GND pins |
| HW-131 breadboard supply | 1 | Bench motor power, fed from a USB charger of at least 2 A |
| 3D-printed frame | 1 | [`hardware/frame/`](hardware/frame/README.md): 8.7 g PLA, no supports |

## Wiring

**Use [`docs/pinout.md`](docs/pinout.md).** It has every pin, by GPIO and
physical pin number, plus the bench and battery power rails. `src/config.py` is
the source of truth, and `docs/pinout.md` mirrors it.
[`firmware/README.md`](firmware/README.md#wiring--every-component-in-one-place)
puts the whole airframe on one page, and each `testing/0X_*/README.md` covers
one part in detail.

The KiCad schematic and wiring PDFs in `hardware/` are older and still show
the compass, so trust `docs/pinout.md` where they differ.

**Grounding rule:** once the bench supply is plugged in, the breadboard rails
are motor ground. Sensor grounds go straight to Pico GND pins (GY-521 to pin 38,
GPS to pin 3). Exactly one wire joins Pico GND to the supply's GND rail. See
[Troubleshooting](#troubleshooting).

## Getting started on the board

You need [Thonny](https://thonny.org) (the firmware and test scripts are run
from it) and, for uploading from a terminal, `mpremote`:

```bash
pip install mpremote        # or: pipx install mpremote
```

### 1. Flash MicroPython

Download the Pico 2 W UF2 from
[micropython.org](https://micropython.org/download/RPI_PICO2_W/). Hold
**BOOTSEL** while plugging the board in, then copy the file to the `RP2350`
drive that appears. The full walkthrough, with a self-contained LED and serial
check, is in [`testing/01_pico_2w/`](testing/01_pico_2w/README.md).

### 2. Put the library on the board

Every script imports `config`, `drivers` and `flight`. Those imports resolve
against the **Pico's** filesystem, not your computer's, so upload them first:

```bash
./tools/upload.sh           # copies config.py, drivers/, flight/ and firmware/tuning.py, then lists the board
./tools/upload.sh --list    # only show what is on the board (warns if main.py is there)
./tools/upload.sh --clean   # wipe the board's root first, then upload
```

Or in Thonny: **View → Files**, select `config.py`, `drivers` and `flight` in
the computer pane, right-click → **Upload to /**.

Re-upload after any change under `src/`, because the board keeps its own copy.
Close Thonny first, or use Thonny's upload instead, since `mpremote` cannot open
the port while Thonny has it.

`upload.sh` never uploads `main.py`, and it removes one it finds. See
[Troubleshooting](#troubleshooting) for why.

### 3. Work through the bench tests, in order

Each test assumes the previous ones pass. Open the script in Thonny (interpreter:
**MicroPython (Raspberry Pi Pico)**) and press Run. **Props off for all of them.**

| # | Component | Motors spin? |
|---|---|---|
| [01](testing/01_pico_2w/README.md) | Pico 2 W bring-up | no |
| [02](testing/02_drv8833/README.md) | DRV8833 drivers, no motor attached | no |
| [03](testing/03_coreless_motor/README.md) | Motor mapping and direction | **yes** |
| [04](testing/04_gy521_imu/README.md) | GY-521 IMU | no |
| [05](testing/05_hmc5883l_compass/README.md) | HMC5883L compass. **Skip it**: the compass is removed from the build | no |
| [06](testing/06_gy_gps6mv2/README.md) | GPS | no |
| [07](testing/07_lipo_power/README.md) | Power system, brownout hunt | **yes** |
| [08](testing/08_full_stack_dry_run/README.md) | Everything together, motors detached | no |

The index and safety notes are in [`testing/README.md`](testing/README.md).

### 4. Firmware scripts

These live in [`firmware/`](firmware/README.md). Open them in Thonny and press
Run; they are not uploaded. Read `firmware/README.md` before running any of
them.

| Script | What it does |
|---|---|
| `lift.py` | Lift test: staggered spin-up, ramp, hold until Stop, optional self-levelling. Settings are at the top (EDIT HERE), so you don't need to re-upload |
| `flight_controller.py` | Attitude hold: self-levels roll and pitch. Without the compass it holds the yaw rate only, so the heading drifts. `tuning.LIVE_MOTORS = False` by default, so nothing spins |
| `autotune.py` | PX4-style autotune flight over WiFi with `gcs/gcs.py`. Starts a WiFi access point and asks for `DRY` or `ARM`. **Never run on hardware** |
| `bench_policy.py` | Times neural-network sizes on the Pico. It never arms the motors |
| `imu_motor_check.py` | Spins each motor on its own as soon as you press Run (props off), and counts failed IMU reads, to find a bad power or ground path |

### 5. Standalone sensor stream (optional)

`src/main.py` brings up the sensors and streams attitude and GPS status to the
REPL, with the motors held hardware-disarmed. MicroPython runs `main.py` on every
boot, so copy it only when you want this:

```bash
mpremote fs cp src/main.py :     # then reset the board
mpremote fs rm :main.py          # remove it before going back to the bench tests (upload.sh also does this)
```

## Simulator quick start

The simulator lives in `sim/`, a separate [uv](https://docs.astral.sh/uv/)
project (Python 3.11–3.12, MuJoCo, Stable-Baselines3, CPU-only torch). The
firmware tests don't need it.

```bash
cd sim
uv sync                                  # once: creates sim/.venv
uv run python watch.py                   # opens a MuJoCo window: lift.py's PID on the nominal drone
uv run python fly.py                     # opens a window: fly it with the keyboard in a room
```

In `watch.py`, click the window first. Then **SPACE** kicks the drone, the
**arrows** shift its centre of mass, and **R** restarts. In `fly.py`, the
**arrows** tilt, **W/S** climb and descend, **A/D** yaw, and **C** switches
between the policy, the PID and no controller. All keys are listed in
[`sim/README.md`](sim/README.md).

### Training (PPO)

```bash
cd sim
uv run python watch_training.py                      # watch PPO learn live in a window (2 min throwaway run)
uv run python train_ppo.py                           # the real run: 6M steps, 2 x 32 network, output in sim/runs/ppo_<time>/
uv run tensorboard --logdir runs                     # learning curves
uv run python export_policy.py runs/ppo_<time>/best.zip   # overwrites src/flight/policy_weights.py (tracked)
uv run python watch.py --ctrl policy --random        # watch the exported policy fly random drones
```

Training is CPU-heavy, so stop other heavy jobs first. A script that flies the
policy on the board (`firmware/rl_lift.py`) is not written yet.

### Autotune

```bash
cd sim
uv run python autotune_sim.py --windowed             # watch the board's autotune run on the simulated drone
uv run python autotune_sim.py --check 12             # 12 random + the extreme drones, no window

# The PX4 + QGC flow, in two terminals:
uv run python sitl.py                                # terminal 1: the simulated vehicle (MuJoCo window)
/usr/bin/python3 ../gcs/gcs.py                       # terminal 2: the ground station (Tk window)
uv run python sitl.py --check                        # no window: the whole flow over real UDP
```

The ground station needs tkinter (`sudo apt install python3-tk`). Use the
system Python: other Pythons' bundled Tk renders fonts badly. The step-by-step
flow, the results so far, and what is modelled are in
[`sim/README.md`](sim/README.md) and [`gcs/README.md`](gcs/README.md). The
procedure for the real drone is in
[`firmware/README.md`](firmware/README.md#autotune-flight-px4-style-over-wifi).

## Project layout

```
src/                     code that runs on the board (also imports under CPython for tests)
  config.py              every pin and constant, in one place
  main.py                optional standalone sensor stream, motors stay disarmed
  drivers/               motors.py (hardware arm/disarm), heartbeat.py, mpu6050.py,
                         hmc5883l.py, gps.py, gcs_link.py (JSON over UDP)
  flight/                fusion.py, pid.py, mixer.py, angles.py,
                         policy.py + policy_weights.py (the trained network, pure Python),
                         autotune.py, autotune_session.py, ground_station.py,
                         gains_file.py (/gains.json), step_response.py
firmware/                Thonny scripts: lift.py, flight_controller.py + tuning.py,
                         autotune.py, bench_policy.py, imu_motor_check.py
testing/01..08_*/        one folder per component: README + bench script
sim/                     MuJoCo model, keyboard flight, PPO, autotune check, SITL (uv project)
gcs/                     gcs.py: QGC-style ground station (tkinter, standard library only)
hardware/                KiCad schematic + wiring PDFs (older, see Wiring)
  frame/                 3D-printed airframe STLs + Onshape FeatureScript source
docs/                    pinout, power/thrust/weight, roadmap, images
tests/                   CPython tests, with tests/mocks/machine.py faking MicroPython's hardware
tools/upload.sh          copies the library to the board with mpremote
```

## Documentation map

| Doc | What it covers |
|---|---|
| [`docs/pinout.md`](docs/pinout.md) | Every pin, power rails, reserved GPIOs, I2C addresses |
| [`docs/power.md`](docs/power.md) | Bench supply, grounding rule, battery topology, driver losses, weight and thrust |
| [`docs/roadmap.md`](docs/roadmap.md) | What is still needed before it flies |
| [`testing/README.md`](testing/README.md) | Bench-test index, upload step, LED heartbeat |
| [`firmware/README.md`](firmware/README.md) | Full wiring, safety model, manual tuning, autotune flight procedure |
| [`sim/README.md`](sim/README.md) | Simulator setup, every script and key, results, what is modelled |
| [`gcs/README.md`](gcs/README.md) | Ground station screens, how the autotune works, the UDP link |
| [`hardware/frame/README.md`](hardware/frame/README.md) | Printing, fitting the boards, tolerances, re-exporting |

## Safety model

**`GP15` (physical pin 20) drives `SLP` on both DRV8833s.** While it is low, the
driver outputs are off and the motors cannot spin, whatever the PWM registers
say. This is a hardware disarm, not a software convention.

- `MotorBank` pulls `SLP` low **before** it creates any PWM channel.
- Using `MotorBank` as a context manager disarms it on every exit, including
  exceptions. New motor code must do the same, with `with MotorBank()` or a
  `finally` block.
- `MAX_DUTY` is capped at 0.70, and `tests/test_config.py` fails if it is raised.
- Nothing in this repo arms the motors at power-on. The flight scripts
  (`lift.py`, `flight_controller.py`, `autotune.py`) only spin up after you type
  `ARM` at the board's prompt. The bench tests that spin motors (03, 07) and
  `imu_motor_check.py` spin them as soon as you press Run, with no prompt. The ground station has no arm button.
- **Every bench procedure assumes props are off.**

The onboard LED pulses (long on, short off) while any script runs. If it stops
and restarts, the board reset, which on this airframe usually means a brownout.

## Development and tests

The tests run on a normal computer with no board attached.
`tests/mocks/machine.py` fakes MicroPython's `machine` module. `Pin` records
levels, `PWM` records duty, and `I2C` serves bytes from a register dict.

```bash
python3 -m venv .venv
./.venv/bin/pip install pytest ruff

./.venv/bin/python -m pytest        # firmware, autotune, policy and link tests
./.venv/bin/ruff check .
```

On a machine with ROS installed, prefix pytest with
`env -u PYTHONPATH PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`. Otherwise ROS's
`launch_testing` plugin auto-loads and crashes.

`sim/sitl.py --check` and `sim/autotune_sim.py --check 12` are the simulator's
end-to-end checks. No CI is attached.

## Troubleshooting

**Thonny prints `MPY: soft reboot` over and over, and none of your script's output.**
There is a `main.py` on the board. MicroPython runs it on every boot *and every
soft reboot*, and Thonny's Run always soft-reboots first, so `main.py` takes
over every run. Run `./tools/upload.sh` (it removes it) or delete it in
Thonny's Files view.

**`ImportError: no module named 'config'`.** The library is not on the board.
Pressing Run in Thonny does not upload anything. See
[step 2](#2-put-the-library-on-the-board).

**A script exits with no message.** MicroPython in Thonny drops the text of an
uncaught `SystemExit("...")`. The scripts here print the message first; do the
same in new code.

**`mpremote: failed to access ... (it may be in use)`.** Thonny has the serial
port open. Close Thonny (or disconnect it), or upload from Thonny's
**View → Files → Upload to /** instead.

**Your edit didn't take effect.** Thonny can save its open, older copy of a file
over a change made on disk. After editing a file outside Thonny, close its tab
*without saving* and reopen it. Then check the file on disk before trusting a
run.

**The IMU drops off I2C the moment the motors spin** (`IMU stopped answering`,
or `i2c err` climbing). A sensor ground is on a breadboard rail, which is motor
ground. Move GY-521 GND to Pico pin 38 and GPS GND to pin 3, and keep exactly one
wire from Pico GND to the supply's GND rail. There is a multimeter check in
[`docs/power.md`](docs/power.md#grounding-rule--breadboard-rails-are-motor-ground).

**Motors stall or slow down on the bench supply.** First check the printed
`m1..m4` values. If the code commands equal throttle, the cause is power, not
the PID: supply sag, a split rail, or extra load on a motor (oversized tape
flags caused this once). Run `lift.py` with `LEVELLING = False` to take the PID
out of the picture. Feed the HW-131 from a USB charger of at least 2 A, not its
barrel jack.

**A motor spins the wrong way.** Swap its two leads at the driver's outputs
(`AOUT1`/`AOUT2` or `BOUT1`/`BOUT2`). `AIN2`/`BIN2` are tied to ground, so this
cannot be fixed in code.

**The ground station window looks rough or won't start.** Run it with
`/usr/bin/python3` and install `python3-tk`.

## Contributing

Issues and pull requests are welcome. Day-to-day work goes on `dev`, and `main`
is protected, so open pull requests against `dev`. Keep the hardware rules in
[`CLAUDE.md`](CLAUDE.md): every script that can drive motors pulls GP15 low at
start and in a `finally` block, `MAX_DUTY` stays at or below 0.70, and code in
`src/` must import under CPython with `tests/mocks/` on the path. Run the tests
and ruff before sending a change.

## License

[MIT](LICENSE)
