# 08 — Full-stack dry run

Every component working together, closing the real control loop, with
**no motors attached**. Tests 01–07 each prove one part in isolation; this one
proves they coexist — sharing I2C0, UART0 and the CPU — and that the
IMU → fusion → PID → mixer → `MotorBank` chain produces sane commands fast
enough to fly on.

> **Motors must not be attached.** There is no control link on this airframe,
> so there would be no way to command a stop. The script never calls `arm()`,
> so `SLP` stays low and both DRV8833s stay asleep throughout — but wire it
> with bare driver outputs anyway.

## What you need

- The complete airframe wired as in 01–07, minus the motors
- 1S LiPo, or USB for bench power
- Nothing else — no multimeter for this one

## Wiring

This is the union of the component tests. Nothing here is new; if a section
looks unfamiliar, go back to that component's README.

### Sensors — I2C0 shared by IMU and magnetometer

| Device pin | Pico 2 W | Physical pin | Note |
|---|---|---|---|
| GY-521 `VCC`, HMC5883L `VCC` | 3V3(OUT) | **36** | |
| GY-521 `GND`, HMC5883L `GND` | GND | **38** | |
| Both `SDA` | GP4 | **6** | One bus, two addresses (0x68 and 0x1E) |
| Both `SCL` | GP5 | **7** | |

### GPS — UART0

| GY-GPS6MV2 pin | Pico 2 W | Physical pin | Note |
|---|---|---|---|
| `VCC` | 3V3(OUT) | **36** | |
| `GND` | GND | **3** | Short wire, sits below GP0/GP1 |
| `TX` | GP1 = UART0 **RX** | **2** | **Crossover** |
| `RX` | GP0 = UART0 **TX** | **1** | **Crossover** |

### Drivers — both DRV8833s, outputs bare

| DRV8833 pin | Pico 2 W | Physical pin | Note |
|---|---|---|---|
| `SLP` (both boards) | GP15 | **20** | Held **low** for this entire test |
| #1 `AIN1` | GP10 | **14** | Motor 1 — front-right |
| #1 `BIN1` | GP11 | **15** | Motor 2 — rear-left |
| #2 `AIN1` | GP12 | **16** | Motor 3 — front-left |
| #2 `BIN1` | GP13 | **17** | Motor 4 — rear-right |
| `AIN2`, `BIN2` (both) | GND | **33** | Tie low — unidirectional |
| `nFAULT` (both) | GP14 | **19** | Open-drain, shared line |
| All `OUT` pads | — | — | **Leave unconnected** |

### Power

Motor current and Pico current take **separate paths from the battery** — see
[07](../07_lipo_power/README.md) for the full treatment and the reasoning.

```
1S LiPo (+) --+-------------------> DRV #1 VM --+-- 470uF -- GND
              |                                  |
              +-------------------> DRV #2 VM --+-- 470uF -- GND
              |
              +--[ Schottky ]-----> Pico VSYS (physical pin 39)

1S LiPo (-) ---- star point ----+--> DRV #1 GND
                                +--> DRV #2 GND
                                +--> Pico GND (physical pin 38)
```

**Never connect battery + to 3V3 (pin 36).** That regulator supplies ~300 mA
and is already feeding the RP2350, the CYW43 and all three sensors.

## Running the test

1. Wire as above, **no motors on the driver outputs**.
2. Upload the library if you have not since your last `src/` change —
   `./tools/upload.sh`, or Thonny's `View → Files` → select `config.py`,
   `drivers`, `flight` → right-click → **Upload to /**.
3. Run `test_full_stack_dry_run.py`.
4. Hold the board still for the gyro calibration.
5. **Tilt the board by hand** and watch the `M1`–`M4` column.

## What you should see

```
=== full-stack dry run ===
>> MOTORS MUST NOT BE ATTACHED <<
drivers stay asleep (SLP low) for the whole run

MOTORS  disarmed (SLP low on GP15), arm() is never called
IMU     ok  (WHO_AM_I 0x68)
calibrating gyro - hold still...
  bias -0.42 0.18 0.06 deg/s
MAG     ok  (HMC5883L)
GPS     ok  (listening, fix takes 30s+ outdoors)

gains (GUESSES - not flight-tuned)
  angle  kp 0.0200  ki 0.0100  kd 0.0040
  yaw    kp 0.0030  ki 0.0000  kd 0.0000
  collective throttle 0.45, MAX_DUTY 0.70

tilt the board by hand and watch M1-M4 respond.
these are corrections: roll RIGHT -> M1,M4 up, M2,M3 down.
ctrl-C to stop.

r  +0.3 p  -0.6 | pid r+0.01 p+0.01 y+0.00 | M1 0.46 M2 0.44 M3 0.44 M4 0.46 |  480Hz hdg 214 gps 0
r +10.2 p  -1.2 | pid r-0.20 p+0.02 y-0.01 | M1 0.65 M2 0.25 M3 0.27 M4 0.67 |  478Hz hdg 231 gps 0
r +24.6 p  -1.5 | pid r-0.49 p+0.03 y-0.01 | M1 0.94 M2 0.00 M3 0.02 M4 0.95 |  477Hz hdg 238 gps 0
```

**The test passing is the asymmetry.** Level, all four motors sit at the
collective throttle. Rolled right, `M1` and `M4` (the right-hand pair) rise and
`M2`/`M3` fall — the loop pushing back toward level. Pitch forward and the rear
pair rises instead.

**The onboard LED pulses throughout.** If it stops and restarts, the board
reset — brownout, see [07](../07_lipo_power/README.md).

**Watch the Hz figure.** Anything above ~250 Hz is comfortable. If it collapses
when the GPS gets a fix, the NMEA parser is eating the loop.

## About those PID gains

**They are guesses.** They were chosen to be visibly responsive on a bench and
too soft to oscillate — they have never been near an airframe in flight.

`ANGLE_KP = 0.020` with `OUTPUT_LIMIT = 0.50` saturates at **25° of tilt** —
past that the PID output stops growing and the motor spread stops widening.
Tilting through that point and watching the numbers go static is worth doing
once. The gain is almost certainly too hot for flight, where corrections live
in the 0–5° band.

### Why this test passes `idle=0.0` and the flight code does not

`MIN_START` (0.20) exists so coreless motors never spool from a dead stop. The
mixer applies it as a hard floor **after** desaturating, so with the floor in
place the low pair pins at 0.20 from about **13°** of tilt and the differential
stops tracking your hand. Nothing spins here, so the dry run passes `idle=0.0`
and every printed number is the true PID + mixer output. Expect the low motors
to reach 0.00 near 25°; in flight they would sit at 0.20.

This test **cannot tell you a gain is wrong.** It only proves the arithmetic
runs and the signs are right. Tuning needs a tethered airframe. When you have
real numbers, move them into `src/config.py` rather than leaving them here.

## If it fails

| Symptom | Cause |
|---|---|
| `IMU FAILED` | Nothing else can proceed. See [04](../04_gy521_imu/README.md) |
| `MAG FAILED` but IMU fine | Address clash or a QMC5883L clone at 0x0D. See [05](../05_hmc5883l_compass/README.md) |
| Motors respond the **wrong way** to tilt | Accelerometer axis signs in `src/flight/fusion.py`, not the gains |
| `r`/`p` drift steadily with the board still | Gyro bias — recalibrate, holding genuinely still |
| `r`/`p` jitter violently | Loose IMU. It must be rigidly mounted, not dangling on wires |
| Hz below ~100 | Something blocking in the loop, usually GPS |
| `[DRV FAULT]` | With no motors attached, an output pad is shorted to GND |
| Board resets mid-run | Brownout. See [07](../07_lipo_power/README.md) |

## Next

This is the last test before motors go on. After this:
`03_coreless_motor` with one motor, then `07_lipo_power` for the brownout hunt,
then a tethered hover.
