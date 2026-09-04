# Component bench tests

Each folder isolates **one component**, so a fault is found in the part that
causes it rather than in the assembled aircraft. Each contains a `README.md` with
a wiring table in physical pin numbers and a runnable script.

**Work through them in order.** Each assumes the previous ones pass.

| # | Component | What it proves | Motors spin? |
|---|---|---|---|
| [01](01_pico_2w/README.md) | Raspberry Pi Pico 2 W | MicroPython runs, serial works, LED blinks | no |
| [02](02_drv8833/README.md) | DRV8833 driver | Chip wakes, outputs swing — measured, no motor | no |
| [03](03_coreless_motor/README.md) | Coreless motors | Each motor spins, mapping and direction correct | **yes** |
| [04](04_gy521_imu/README.md) | GY-521 / MPU6050 | Gyro + accel read, attitude tracks tilt | no |
| [05](05_hmc5883l_compass/README.md) | HMC5883L compass | Heading sweeps 0–360 correctly | no |
| [06](06_gy_gps6mv2/README.md) | GY-GPS6MV2 GPS | NMEA arrives, fix acquired outdoors | no |
| [07](07_lipo_power/README.md) | 1S LiPo power | No brownout under full four-motor load | **yes** |

## Before running anything that spins

- **Props off.** Every one of these procedures assumes it.
- Motors physically restrained — taped down or in a frame.
- Nothing loose on the bench near a motor.

## Running a script

### Step 0 — put the library on the board

Every script except [01](01_pico_2w/README.md) imports `config`, `drivers` and
`flight`. **Those imports resolve against the Pico's filesystem, not your
computer's.** Opening a file in an editor and pressing Run does not upload
anything, so without this step you get:

```
ImportError: no module named 'config'
```

- **Thonny** — `View → Files`. In the top (computer) pane select `config.py`,
  `drivers` and `flight`, right-click → **Upload to /**.
- **Terminal** — `./tools/upload.sh`, which needs `pip install mpremote`.
  `--list` shows what is currently on the board, `--clean` wipes it first.

Re-upload after any change under `src/`. The board keeps its own copy; editing
the file on your computer does not update it.

**`main.py` is deliberately never uploaded by this script.** MicroPython
auto-runs a file named `main.py` on every boot *and every soft-reboot* — and
Thonny's Run button always soft-reboots the board first. If `main.py` were on
the board, every test run would re-enter it instead of your script before your
script ever got a chance to execute. Its sensor bring-up blocks forever in two
places, so the symptom is total silence: the shell just prints

```
MPY: soft reboot
```

repeatedly, with none of your test's output ever appearing. `upload.sh` actively
removes `main.py` from the board if it finds one there, for exactly this reason.

### Step 1 — run it

Open the script in Thonny with the interpreter set to **MicroPython (Raspberry Pi
Pico)** and press Run.

[01](01_pico_2w/README.md) needs no upload at all — it is deliberately
self-contained so you can check the board before anything else exists on it.

## The LED heartbeat

**Every script here pulses the onboard LED for as long as it runs** — a long on,
short off that reads as a steady glow but visibly beats. It is a failsafe you can
watch without looking at the shell.

| LED | Meaning |
|---|---|
| Pulsing | The board is powered and its scheduler is running |
| Went dark | The script ended normally, or the board lost power |
| **Stopped and restarted** | The board **reset** — on this airframe that means a brownout ([07](07_lipo_power/README.md)) |
| Fast even blink | `main.py` only: a sensor failed to initialise |

It runs on a soft timer, so it keeps beating through blocking calls — the
six-second measurement pauses in [02](02_drv8833/README.md), the minutes of
waiting for a GPS fix in [06](06_gy_gps6mv2/README.md).

**What it does not prove:** that your main loop is progressing. A soft timer
fires from the scheduler and carries on even if the code above it is stuck. This
detects reset and power loss — which is what a coreless quad actually suffers
from — not a hang.

[01](01_pico_2w/README.md) is the one exception to the shared implementation: it
runs before anything has been uploaded to the board, so it carries its own copy
rather than importing `drivers/heartbeat.py`. Every other script imports the real
one, and `tools/check_structure.py` fails CI if any script starts a heartbeat
without stopping it.

## Safety model

`GP15` (physical pin 20) drives `SLP` on both DRV8833s. Held low, the driver
outputs go high-impedance and the motors are dead regardless of what the PWM
registers contain — a hardware disarm that a software bug cannot override.

Every script here pulls it low at start and in a `finally` block.
`tools/check_structure.py` fails CI if one does not.
