# 02 — DRV8833 dual H-bridge

Verifies the driver board **with no motor attached**. You are checking that the
chip wakes up and that its outputs actually swing, using a multimeter. Doing this
before connecting a motor means a wiring fault costs you nothing.

> **No motor connected for this test.** Connect motors in `03_coreless_motor`.

## What you need

- 1× DRV8833 breakout
- Multimeter
- Pico 2 W with MicroPython flashed (see `01_pico_2w`)

## Wiring — driver #1

Orient the Pico with the **USB port at the top**. Pin 1 is the top-left pad;
numbers run down the left side (1–20) then up the right side (21–40).

| DRV8833 pin | Pico 2 W | Physical pin | Note |
|---|---|---|---|
| `VM` / `VCC` / `VMOT` | VBUS (5 V) | **40** | Bench testing only. In flight this comes from the battery |
| `GND` | GND | **38** | |
| `SLP` / `nSLEEP` / `EEP` | GP15 | **20** | Software arm/disarm. Chip is asleep when low |
| `AIN1` | GP10 | **14** | Motor 1 PWM |
| `AIN2` | GND | **33** | Tie low — makes channel A unidirectional |
| `BIN1` | GP11 | **15** | Motor 2 PWM |
| `BIN2` | GND | **33** | Tie low — makes channel B unidirectional |
| `nFAULT` *(if present)* | GP14 | **19** | Open-drain, low = over-current or thermal shutdown |
| `AOUT1`, `AOUT2` | — | — | **Leave unconnected for this test** |
| `BOUT1`, `BOUT2` | — | — | **Leave unconnected for this test** |

Driver #2 is identical except `AIN1`→GP12 (pin 16) and `BIN1`→GP13 (pin 17).
Both drivers share the same `SLP`, `VM` and `GND`.

### Why AIN2/BIN2 go to ground

The DRV8833 is an H-bridge, which can drive a motor both ways. Drone props never
reverse, so tying one input of each channel low turns it into a simple one-way
speed controller: PWM the other input and duty cycle is throttle. Rotation
direction is then set by **which way round the motor leads are soldered**, never
in software.

### The DRV8833 has only one supply pin

Whatever your board calls it — `VM`, `VCC`, `VMOT`, `VIN` — it powers both the
motor and the chip's internal logic. There is no separate logic rail.

**Never connect it to 3V3 (pin 36).** That regulator supplies ~300 mA; motors
pull several times that. Pin 36 is only for `SLP`.

## Running the test

1. Wire as above, **no motors**.
2. Multimeter in DC volts, black probe on GND.
3. Run `test_drv8833.py`.
4. When prompted, put the red probe on `AOUT1`, then `BOUT1`.

## What you should see

```
=== DRV8833 driver check ===
SLP low  -> drivers asleep
  measure AOUT1 now: expect ~0 V (high impedance)
SLP high -> drivers awake
  AIN1 at 50% duty
  measure AOUT1 now: expect roughly 2.0-2.7 V
  AIN1 at 0%
  measure AOUT1 now: expect ~0 V
nFAULT  : OK (high)
=== disarmed ===
```

A PWM output measured with a cheap multimeter reads the **average**, so 50% duty
on a 5 V rail shows somewhere around 2.0–2.7 V, not a clean 2.5 V. Anything in
that band is a pass. What matters is that it moves when the duty changes.

## If it fails

| Symptom | Cause |
|---|---|
| `AOUT1` stays at 0 V always | `SLP` not reaching 3.3 V. Measure it directly — this is the most common failure and produces no error message |
| `AOUT1` sits at full 5 V | `AIN2` not tied to GND |
| `nFAULT: TRIPPED` | Over-current or thermal. With no motor attached this means an output is shorted to GND |
| Nothing on either channel | `VM` unpowered, or GND not shared with the Pico |

## Thermal reality check

This build runs **two motors per DRV8833**. Each dissipates roughly 0.8 W per
motor at 1.5 A, and the cheap breakouts have no heatsinking, so at sustained full
throttle the chip reaches its 150 °C thermal shutdown and cuts both its motors
without warning.

`MAX_DUTY` in `src/config.py` is capped at **0.70** for this reason. Do not raise
it while running two drivers.
