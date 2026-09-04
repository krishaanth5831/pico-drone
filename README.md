# pico-drone

A coreless quadcopter flight controller built on a **Raspberry Pi Pico 2 W**,
written in MicroPython.

Sensor drivers, control maths, and a component-by-component bench test suite —
with a hardware disarm line that a software bug cannot override.

> **Status: bench development.** This does not fly yet. There is no control link
> on the airframe, so `main.py` deliberately stops at sensor bring-up and never
> arms the motors. See [`docs/roadmap.md`](docs/roadmap.md).

## Hardware

| Part | Role |
|---|---|
| Raspberry Pi Pico 2 W | Flight controller (RP2350, dual-core 150 MHz) |
| 2× DRV8833 | Dual H-bridge motor drivers, two motors each |
| 4× coreless motors | Propulsion (8520 recommended — see [`docs/power.md`](docs/power.md)) |
| GY-521 (MPU6050) | 3-axis gyro + accelerometer |
| HMC5883L | 3-axis magnetometer, yaw reference |
| GY-GPS6MV2 (NEO-6M) | GPS position |
| 1S LiPo | Power |

Full wiring: [`docs/pinout.md`](docs/pinout.md).

## Layout

```
src/
  config.py            single source of truth for every pin and constant
  main.py              entry point - sensor bring-up, motors stay disarmed
  drivers/
    motors.py          DRV8833 bank with hardware arm/disarm
    heartbeat.py       onboard-LED liveness pulse, soft-timer driven
    mpu6050.py         GY-521 gyro + accelerometer
    hmc5883l.py        magnetometer, auto-detects the QMC5883L clone
    gps.py             NEO-6M NMEA parser
  flight/
    fusion.py          complementary filter -> roll and pitch
    pid.py             rate controller, anti-windup, no derivative kick
    mixer.py           X-quad motor mixing with desaturation

testing/               per-component bench procedures + scripts
docs/                  pinout, power/thrust/weight, roadmap
tests/                 CPython tests against a mocked hardware layer
tools/                 repo convention checks run in CI
```

## Getting started

### 1. Flash MicroPython

Grab the Pico 2 W UF2 from
[micropython.org](https://micropython.org/download/RPI_PICO2_W/), hold
**BOOTSEL** while plugging in, and copy the file to the `RP2350` drive that
appears. Full walkthrough in [`testing/01_pico_2w/`](testing/01_pico_2w/README.md).

### 2. Put the library on the board

The bench scripts import `config`, `drivers` and `flight`, and those imports
resolve against the **Pico's** filesystem — not your computer's:

```bash
pip install mpremote
./tools/upload.sh          # --list to check, --clean to wipe first
```

Or in Thonny: `View → Files`, select `config.py`, `drivers` and `flight`,
right-click → **Upload to /**. Re-upload after any change under `src/`.

`main.py` is deliberately excluded. MicroPython auto-runs it on every boot *and
every soft-reboot* — which is what Thonny's Run button always does first — so a
`main.py` on the board hijacks every test run before your script executes. If
you see the shell print `MPY: soft reboot` repeatedly with none of your test's
own output, this is why: run `./tools/upload.sh` again, which removes it.

### 3. Work through the bench tests

Do these **in order** — each assumes the previous one passes.

| # | Component | Motors spin? |
|---|---|---|
| [01](testing/01_pico_2w/README.md) | Pico 2 W bring-up | no |
| [02](testing/02_drv8833/README.md) | DRV8833 driver, no motor attached | no |
| [03](testing/03_coreless_motor/README.md) | Motor mapping and direction | **yes** |
| [04](testing/04_gy521_imu/README.md) | IMU | no |
| [05](testing/05_hmc5883l_compass/README.md) | Compass | no |
| [06](testing/06_gy_gps6mv2/README.md) | GPS | no |
| [07](testing/07_lipo_power/README.md) | Power system, brownout hunt | **yes** |

Index and safety notes: [`testing/README.md`](testing/README.md).

### 4. Run it

**Do this separately from the bench tests above, and remove it again afterwards.**
`main.py` auto-runs on every boot and soft-reboot, so leaving it on the board
hijacks every test run in step 3 - if you go back to bench testing after this,
delete it first (`./tools/upload.sh` does this for you automatically).

```bash
mpremote fs cp src/main.py :
```

Then reset the board. You get live attitude, heading and GPS status streamed to
the REPL, with the motors held hardware-disarmed throughout.

## Safety model

**`GP15` (physical pin 20) drives `SLP` on both DRV8833s.** Held low, the driver
outputs go high-impedance and the motors are dead no matter what the PWM
registers contain. This is a hardware disarm, not a software convention.

Enforced throughout:

- `MotorBank` drives `SLP` low **before** any PWM channel is constructed, so no
  stale duty can reach a motor during startup.
- Using it as a context manager disarms on every exit path, exceptions included.
- Keep that pattern in any new motor-driving code: a `finally` block or
  `with MotorBank()`, no exceptions.
- `MAX_DUTY` is capped at 0.70 — two DRV8833s thermally shut down at sustained
  full throttle — and `tests/test_config.py` fails if it is raised.
- Nothing in this repo arms motors at power-on.

**Every bench procedure assumes props are off.**

### LED heartbeat

Every script pulses the onboard LED for as long as it runs — a long on, short off
that reads as a glow. Pulsing means the board is powered and its scheduler is
running; **stopping and restarting means it reset**, which on this airframe means
a brownout. It runs on a soft timer, so it survives blocking calls, and
Keep that pattern in anything new: start it, stop it, no orphaned timers.

It does not prove the main loop is progressing — a soft timer keeps firing even
if the code above it hangs. It catches reset and power loss, which is the failure
this hardware actually has.

## Development

```bash
python3 -m venv .venv
./.venv/bin/pip install pytest ruff

./.venv/bin/python -m pytest        # 80 tests, no hardware needed
./.venv/bin/ruff check .
```

`tests/mocks/machine.py` fakes MicroPython's hardware layer — `Pin` records its
levels, `PWM` its duty, `I2C` serves bytes from a register dict — so the same
code the board runs can be exercised on a computer with no board attached.

> On a machine with ROS installed, prefix pytest with
> `env -u PYTHONPATH PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` — ROS's `launch_testing`
> plugin auto-loads and crashes.

## License

[MIT](LICENSE)
