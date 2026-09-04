# Consolidated pin map

Every GPIO on the airframe, in one place. The authoritative source is
[`src/config.py`](../src/config.py) — this document mirrors it, and
Keep it that way when editing `testing/` scripts - import pins from `config.py`
rather than hardcoding a GPIO number.

## Board orientation

Hold the Pico 2 W with the **USB port at the top**. Pin 1 is the top-left pad;
numbers run down the left side (1–20), then continue up the right side (21–40).

## Signals

| Function | Device pin | Pico GPIO | Physical pin |
|---|---|---|---|
| Motor 1 PWM | DRV #1 `AIN1` | GP10 | 14 |
| Motor 2 PWM | DRV #1 `BIN1` | GP11 | 15 |
| Motor 3 PWM | DRV #2 `AIN1` | GP12 | 16 |
| Motor 4 PWM | DRV #2 `BIN1` | GP13 | 17 |
| Motor arm / kill | both DRV `SLP` | GP15 | 20 |
| Driver fault *(optional)* | both DRV `nFAULT` | GP14 | 19 |
| IMU data | GY-521 `SDA` | GP4 | 6 |
| IMU clock | GY-521 `SCL` | GP5 | 7 |
| Compass data | HMC5883L `SDA` | GP4 | 6 |
| Compass clock | HMC5883L `SCL` | GP5 | 7 |
| GPS → Pico | GY-GPS6MV2 `TX` | GP1 | 2 |
| Pico → GPS | GY-GPS6MV2 `RX` | GP0 | 1 |
| — | DRV `AIN2`/`BIN2` ×4 | **tie to GND** | 3, 8, 13, 18, 23, 28, 33, 38 |

## Power rails

| Rail | Feeds | Physical pin |
|---|---|---|
| 3V3(OUT) | GY-521 `VCC`, HMC5883L `VCC`, GPS `VCC` | 36 |
| VSYS ← battery via Schottky | the Pico itself | 39 |
| VBUS (5 V, USB only) | bench-only driver supply | 40 |
| Battery + direct | DRV #1 `VM`, DRV #2 `VM` | — |
| GND | everything, star-grounded at the battery | 38 (also 3, 8, 13, 18, 23, 28, 33) |

## Reserved — never assign these

| GPIO | Physical pin | Used by |
|---|---|---|
| GP23 | 29 | CYW43 WiFi power-save |
| GP24 | 31 | CYW43 data |
| GP25 | 32 | CYW43 chip select |
| GP29 | 34 | CYW43 clock / VSYS sense |

On Pico W and Pico 2 W these are wired to the WiFi/Bluetooth chip and are not
free GPIO. That is also why the onboard LED is `Pin("LED")` rather than `Pin(25)`
and cannot be PWM'd. `tests/test_config.py` fails if any of them is assigned.

## I2C addresses

| Device | Address |
|---|---|
| MPU6050 / GY-521 | `0x68` (`0x69` with `ADO` high) |
| HMC5883L | `0x1E` |
| QMC5883L clone | `0x0D` |

The IMU and compass share bus I2C0 on GP4/GP5 — wire them in parallel.
