# Roadmap

What exists, and what is still needed before this flies.

## Done

- [x] Pin map single-sourced in [`src/config.py`](../src/config.py)
- [x] Motor driver with hardware arm/disarm ([`src/drivers/motors.py`](../src/drivers/motors.py))
- [x] MPU6050, HMC5883L/QMC5883L and NEO-6M drivers
- [x] Complementary filter, PID, and X-quad mixer
- [x] Per-component bench procedures under [`testing/`](../testing/README.md)
- [x] CI: lint, tests against a mocked hardware layer, structure checks

## Next — in order

### 1. Sensor loop timing
Measure the actual achievable loop rate on the RP2350 under MicroPython. The rate
loop wants 500 Hz minimum. If MicroPython cannot hold that, the hot path moves to
`@micropython.viper` or the whole thing moves to the C/C++ SDK.

### 2. Control link
**Nothing currently commands the aircraft.** Options:

| Option | Latency | Range | Extra parts |
|---|---|---|---|
| Pico 2 W WiFi + phone app over UDP | 50–100 ms | ~30 m | none |
| nRF24L01+ on SPI, second Pico as transmitter | ~10 ms | good | ~£2 |
| ExpressLRS receiver on UART | ~5 ms | excellent | ~£15 + transmitter |

The nRF24L01+ is the usual answer for a build like this.

### 3. Rate loop
Gyro → PID → mixer → motors, with a slew limit on motor output. Tune on a test
rig with a single axis free before anything flies.

### 4. Angle loop
Outer loop over the rate loop, using the complementary filter's attitude.

### 5. Failsafe
Link loss must cut throttle, not hold it. Arming must require an explicit
sequence, never happen at power-on.

### 6. GPS modes
Position hold and return-to-home. Needs the magnetometer calibrated on the
assembled airframe — without a heading reference the controller knows where it is
but not which way it faces.

## Known constraints

- Two DRV8833s thermally shut down at sustained full throttle — see
  [`docs/power.md`](power.md)
- Thrust-to-weight is marginal with the GPS fitted; build v1 without it
- MicroPython is likely too slow for the final rate loop
