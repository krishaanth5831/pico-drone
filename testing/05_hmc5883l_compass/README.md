# 05 — HMC5883L magnetometer

Supplies the absolute yaw reference the MPU6050 cannot. Without it, heading
drifts slowly over a flight. You do not need it to fly manually — you do need it
for GPS position hold or return-to-home, because otherwise the controller knows
*where* it is but not *which way it is facing*.

## HMC5883L or QMC5883L?

Two different chips ship under the same silkscreen. The driver detects which one
you have, but it helps to know:

| | HMC5883L (Honeywell) | QMC5883L (QST clone) |
|---|---|---|
| I2C address | `0x1E` | `0x0D` |
| ID registers | spell `H43` | none |
| Data order | X, **Z**, Y | X, Y, Z |

The register data order is a genuine trap: reading an HMC5883L as X, Y, Z gives a
compass that looks entirely plausible and is wrong by 90°.

## Wiring — shares I2C0 with the IMU

| HMC5883L pin | Pico 2 W | Physical pin | Note |
|---|---|---|---|
| `VCC` | 3V3(OUT) | **36** | |
| `GND` | GND | **38** | |
| `SDA` | GP4 | **6** | Same bus as the GY-521 |
| `SCL` | GP5 | **7** | Same bus as the GY-521 |
| `DRDY` | — | — | Leave unconnected |

No address conflict: the IMU answers at `0x68`, this at `0x1E` or `0x0D`. Both
modules connect to the same two Pico pins — wire them in parallel.

## Mounting — this one matters more than the others

A magnetometer measures fields in the microtesla range. Earth's field is about
50 µT; the tens of amps flowing through your power leads produce fields far
stronger than that at close range.

- Mount it **as far from motors and battery wiring as physically possible**. On
  small builds it usually shares a mast with the GPS.
- Keep it away from anything ferrous, and from the magnets in the motors.
- **Calibrate with the aircraft fully assembled and the battery connected.** The
  distortion you are cancelling comes from the airframe itself, so a calibration
  done on a bare bench is worthless once the module is bolted down.

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

Run `test_hmc5883l_compass.py`. It detects the chip, runs a 30-second
calibration, then streams live heading.

During calibration, **rotate the board slowly through every orientation** — a
lazy figure-of-eight in all three axes — for the whole 30 seconds.

## What you should see

```
=== HMC5883L / QMC5883L test ===
LED pulses for as long as this runs

I2C devices : ['0x1e', '0x68']
detected    : HMC5883L

calibration: rotate slowly through all orientations for 30s
  ...
offset: (12.4, -8.1, 3.7)
scale:  (1.02, 0.97, 1.01)

streaming - rotate the board, ctrl-C to stop
  heading  47.2  |  x  18.3  y  -19.8  z  -41.2  |  field  49.8 uT
  heading 138.9  |  x -20.1  y  -17.9  z  -41.0  |  field  49.6 uT
```

**Rotate the board on a level surface and watch the heading.** Checks that
matter:

- Heading sweeps smoothly through **0 → 360** as you turn it a full circle, with
  no jumps.
- Turning **clockwise** makes the heading **increase**.
- Point it north and compare against a phone compass — within ~10° is fine.
- `field` magnitude stays roughly **25–65 µT** and barely changes as you rotate.
  A magnitude that swings wildly means calibration failed or something magnetic
  is too close.

**The onboard LED pulses throughout.** If it stops and restarts, the board reset — see [07](../07_lipo_power/README.md).

## If it fails

| Symptom | Cause |
|---|---|
| Not in the I2C scan | Wrong pins, or no power. Both this and the IMU must appear |
| `0x1E responded but ID was...` | Clone at the HMC address. Harmless, driven identically |
| Heading jumps rather than sweeping | Data register order — you likely have a QMC misdetected as HMC |
| Heading 90° off consistently | Same cause as above |
| `field` swings hugely while rotating | Uncalibrated, or hard iron too close. Re-calibrate assembled |
| Heading changes when throttle changes | Module too close to motor or battery wiring. Move it further away |
