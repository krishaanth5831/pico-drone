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

Open it in Thonny with the interpreter set to **MicroPython (Raspberry Pi Pico)**
and press Run. Scripts import from `src/`, so copy `config.py`, `drivers/` and
`flight/` to the board's filesystem root first — or upload the release bundle,
which has the right layout.

## Safety model

`GP15` (physical pin 20) drives `SLP` on both DRV8833s. Held low, the driver
outputs go high-impedance and the motors are dead regardless of what the PWM
registers contain — a hardware disarm that a software bug cannot override.

Every script here pulls it low at start and in a `finally` block.
`tools/check_structure.py` fails CI if one does not.
