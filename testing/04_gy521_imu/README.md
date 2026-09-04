# 04 — GY-521 / MPU6050 IMU

The sensor the aircraft actually flies on. The gyro feeds the rate loop; the
accelerometer corrects its drift. Nothing else on the airframe can replace it —
GPS at 1 Hz and a magnetometer at 50 Hz are both orders of magnitude too slow to
keep a quad upright.

## What you need

- GY-521 breakout (MPU6050)
- 4 jumper wires

No motors involved. Safe to run on USB power alone.

## Wiring — I2C0

| GY-521 pin | Pico 2 W | Physical pin | Note |
|---|---|---|---|
| `VCC` | 3V3(OUT) | **36** | Onboard regulator accepts 3–5 V, but 3V3 is the cleaner supply |
| `GND` | GND | **38** | |
| `SDA` | GP4 | **6** | I2C0 data |
| `SCL` | GP5 | **7** | I2C0 clock |
| `XDA`, `XCL`, `ADO`, `INT` | — | — | Leave unconnected |

`ADO` floating gives address **0x68**. Pull it high for 0x69 if you ever need two
on one bus.

Most GY-521 breakouts include the 4.7 kΩ I2C pull-ups. If yours does not, add
them from SDA and SCL to 3V3.

## Mounting

Two mechanical points that matter as much as the wiring:

- **Mount it as close to the centre of mass as you can.** Off-centre, rotation
  shows up as linear acceleration and corrupts the attitude estimate.
- **Decouple it from frame vibration** — a square of double-sided foam tape is
  the standard trick. Bolted rigidly to a frame with four spinning motors, prop
  vibration feeds straight into the accelerometer and the estimate drifts badly.

Keep the wires away from motor leads. Coreless motors are brush-commutated and
throw a lot of electrical noise; corrupted I2C reads mid-flight drop your
attitude estimate.

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

Put the board flat and still, then run `test_gy521_imu.py`. It scans the bus,
identifies the chip, calibrates the gyro, then streams live attitude.

**Do not move the board during calibration.** Every MPU6050 has a few deg/s of
zero offset, and integrating that uncorrected walks your heading away within
seconds.

## What you should see

```
=== GY-521 / MPU6050 test ===
LED pulses for as long as this runs

I2C devices : ['0x68']
WHO_AM_I    : 0x68 (MPU6050)
temperature : 24.8 C

calibrating gyro - hold completely still...
  bias  -1.53  0.88  0.24 deg/s

streaming - tilt the board, ctrl-C to stop
  roll   +0.3  pitch   -0.1  |  gyro   0.1  -0.2   0.0  |  accel  9.79
  roll  -28.7  pitch   +1.2  |  gyro  -2.1   0.4   0.1  |  accel  9.81
```

**Now pick the board up and tilt it.** This is the part to actually watch:

- Tilt **right** → `roll` goes positive
- Tilt **nose down** → `pitch` goes negative
- Hold any angle steady → the reading holds steady too, and does not drift away
- Lying flat → both read within a degree or two of zero
- `accel` magnitude stays near **9.81** in any orientation — that is gravity, and
  it is the check that your scaling is right

**The onboard LED pulses throughout.** If it stops and restarts, the board reset — see [07](../07_lipo_power/README.md).

## If it fails

| Symptom | Cause |
|---|---|
| Only `MPY: soft reboot` printed, twice, nothing else | `main.py` is on the board and is hijacking every soft-reboot before your script runs. Run `./tools/upload.sh` to remove it |
| `I2C devices: []` | SDA/SCL swapped, or no power. Check 3V3 at the module |
| `0x68` present, WHO_AM_I error | Clone chip. MPU6500/9250 report 0x70/0x71/0x73 and are accepted |
| Readings all zero | Chip still asleep — a failed `PWR_MGMT_1` write, usually a flaky SDA connection |
| `accel` magnitude far from 9.81 | Wrong full-scale range, or a fake chip |
| Attitude drifts steadily while still | Gyro calibration ran while the board was moving. Re-run it |
| Values jump erratically | I2C wires routed near motor leads, or missing pull-ups |
