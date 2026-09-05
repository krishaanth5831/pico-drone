# Roadmap

What exists, and what is still needed before this flies.

## Done

- [x] Pin map single-sourced in [`src/config.py`](../src/config.py)
- [x] Motor driver with hardware arm/disarm ([`src/drivers/motors.py`](../src/drivers/motors.py))
- [x] MPU6050, HMC5883L/QMC5883L and NEO-6M drivers
- [x] Complementary filter, PID, and X-quad mixer
- [x] Per-component bench procedures under [`testing/`](../testing/README.md)
- [x] LED heartbeat failsafe on every bench script
- [x] Attitude-hold flight controller (`firmware/`) — self-levels and holds
      heading, manually tuned; no altitude/position hold (no sensor for it)

## Next — in order

### 1. Sensor loop timing
Measure the actual achievable loop rate on the RP2350 under MicroPython. The rate
loop wants 500 Hz minimum. If MicroPython cannot hold that, the hot path moves to
`@micropython.viper` or the whole thing moves to the C/C++ SDK.

### 2. A physical kill switch

`firmware/` runs with no way to stop a misbehaving motor faster than pulling
the battery. A GPIO-wired push-button cutting `SLP` directly, independent of
whatever the main loop is doing, is cheap and should happen before any
autotune routine or untethered test.

### 3. Control link
**Nothing currently commands the aircraft.** Options:

| Option | Latency | Range | Extra parts |
|---|---|---|---|
| Pico 2 W WiFi + phone app over UDP | 50–100 ms | ~30 m | none |
| nRF24L01+ on SPI, second Pico as transmitter | ~10 ms | good | ~£2 |
| ExpressLRS receiver on UART | ~5 ms | excellent | ~£15 + transmitter |

The nRF24L01+ is the usual answer for a build like this.

### 4. Slew-limit motor output
`firmware/flight_controller.py` has the rate and angle loops implemented and
tuning-capable, but motor commands aren't slew-limited between iterations yet —
only the throttle ramp at startup is. A cascaded PID can still command a step
change in individual motor output on a bad transient; rate-limiting the mixer's
output the same way `drivers/motors.ramp()` does for throttle would catch that.

### 5. Failsafe
Link loss must cut throttle, not hold it. This is unreachable until the control
link (#3) exists — right now, arming still requires an explicit "type ARM"
confirmation at the REPL, and nothing keeps running unattended past
`RUN_SECONDS`, but there is no *link* to lose yet.

### 6. GPS modes
Position hold and return-to-home. Needs the magnetometer calibrated on the
assembled airframe — without a heading reference the controller knows where it is
but not which way it faces.

## Known constraints

- Two DRV8833s thermally shut down at sustained full throttle — see
  [`docs/power.md`](power.md)
- Thrust-to-weight is marginal with the GPS fitted; build v1 without it
- MicroPython is likely too slow for the final rate loop
