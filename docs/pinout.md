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
| GPS → Pico | GY-GPS6MV2 `TX` | GP1 | 2 |
| Pico → GPS | GY-GPS6MV2 `RX` | GP0 | 1 |
| — | DRV `AIN2`/`BIN2` ×4 | **tie to GND** | 3, 8, 13, 18, 23, 28, 33, 38 |

The HMC5883L compass was **removed from the build on 2026-10-09**. With it
attached the IMU dropped off I2C whenever the motors ran — most likely the same
breadboard-rail grounding fault later found with the GPS, not the compass itself.
It stays out anyway: the build doesn't need a heading yet, and a few cm from the
motors its readings are swamped. The GY-521 alone gives roll, pitch and
yaw rate; `firmware/flight_controller.py` runs without a compass (yaw rate-hold,
heading drifts slowly). Its component test in `testing/05_hmc5883l_compass/` is
kept in case it comes back on a mast.

## Power rails — bench (current setup, props off)

No battery. The Pico runs from the laptop's USB and the motors from an HW-131
(MB102-style) breadboard power supply. Full picture in
[`power.md`](power.md#bench-power-hw-131-breadboard-supply).

| Rail | Feeds | Physical pin |
|---|---|---|
| Laptop USB → Pico | the Pico itself | — |
| 3V3(OUT) | GY-521 `VCC` only | 36 |
| VBUS (5 V from USB) | GPS `VCC` (the module has its own 3.3 V regulator) | 40 |
| HW-131 rail, jumper on **5 V** | DRV #1 `VM`, DRV #2 `VM` — nothing else | — |
| HW-131 GND rail | DRV #1 `GND`, DRV #2 `GND`, one wire to Pico GND | — |
| Pico GND | GY-521 `GND` (38), GPS `GND` (3), the one wire to the HW-131 GND rail | 3, 38 |

**The breadboard's power rails are motor ground once the HW-131 is plugged in.**
Sensor grounds go straight to Pico GND pins (GY-521 → 38, GPS → 3), never to a
rail, and exactly one wire joins Pico GND to the HW-131 GND rail. Getting this
wrong knocks the IMU off I2C the moment the motors spin — see the
[grounding rule](power.md#grounding-rule--breadboard-rails-are-motor-ground).
HW-131 5 V never goes to VSYS or VBUS.

## Power rails — airframe (battery, planned)

| Rail | Feeds | Physical pin |
|---|---|---|
| 3V3(OUT) | GY-521 `VCC`, GPS `VCC` | 36 |
| VSYS ← battery via Schottky | the Pico itself | 39 |
| Battery + direct | DRV #1 `VM`, DRV #2 `VM` | — |
| GND | everything, star-grounded at the battery | 38 (also 3, 8, 13, 18, 23, 28, 33) |

## Reserved — never assign these

| GPIO | Physical pin | Used by |
|---|---|---|
| GP23 | none (internal) | CYW43 WiFi power-save |
| GP24 | none (internal) | CYW43 data |
| GP25 | none (internal) | CYW43 chip select |
| GP29 | none (internal) | CYW43 clock / VSYS sense |

These four are not broken out to the header at all. Physical pins 29, 31, 32 and
34 are GP22, GP26, GP27 and GP28, which are ordinary free GPIO.

On Pico W and Pico 2 W these are wired to the WiFi/Bluetooth chip and are not
free GPIO. That is also why the onboard LED is `Pin("LED")` rather than `Pin(25)`
and cannot be PWM'd. `tests/test_config.py` fails if any of them is assigned.

## I2C addresses

| Device | Address |
|---|---|
| MPU6050 / GY-521 | `0x68` (`0x69` with `ADO` high) |

The IMU is alone on I2C0 (GP4/GP5). A scan should return `[104]`.
