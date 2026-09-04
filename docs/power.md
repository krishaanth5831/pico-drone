# Power, thrust and weight

The three numbers that decide whether this build flies.

## Power topology

Motor current and Pico current take separate paths from the battery, meeting only
at the terminals:

```
1S LiPo (+) --+-------------------> DRV #1 VM --+-- 470uF -- GND
              |                                  |
              +-------------------> DRV #2 VM --+-- 470uF -- GND
              |
              +--[ SS14 Schottky ]-> Pico VSYS (pin 39)

1S LiPo (-) ---- star point ----+--> DRV #1 GND
                                +--> DRV #2 GND
                                +--> Pico GND (pin 38)
```

**Capacitors** absorb the millisecond current spikes the battery's chemistry
cannot. Mount them physically close to each driver's VM/GND pins.

**The Schottky** prevents USB 5 V backfeeding the LiPo when both are connected.
Without it, connect only one at a time.

**Star grounding** keeps motor return current out of the Pico's ground wire,
where it would create offset voltages that read as IMU noise.

Full brownout explanation and the test procedure:
[`testing/07_lipo_power/`](../testing/07_lipo_power/README.md).

## Driver losses

Each DRV8833 half-bridge pair has ~360 mΩ of on-resistance. At 1.5 A that drops
~0.54 V and dissipates ~0.8 W **per motor**.

| Setup | R path | Drop @1.5 A | Heat per chip | Motor sees | Thrust (8520) |
|---|---|---|---|---|---|
| **2 chips, 2 motors each** *(current build)* | 360 mΩ | 0.54 V | **1.6 W** | 3.06 V | ~101 g |
| 4 chips, 1 motor, not paralleled | 360 mΩ | 0.54 V | 0.81 W | 3.06 V | ~101 g |
| 4 chips, 1 motor, channels paralleled | 180 mΩ | 0.27 V | 0.41 W | 3.33 V | ~120 g |
| 4× SI2302 MOSFET | 50 mΩ | 0.08 V | negligible | 3.52 V | ~134 g |

*(1S pack sagging to ~3.6 V under load; coreless thrust scales roughly with V².)*

### The current build's limitation

Two chips means two motors per package at ~1.6 W. The cheap breakouts have no
heatsinking, so **sustained full throttle reaches the 150 °C thermal shutdown**
and cuts both that chip's motors without warning.

`MAX_DUTY` is capped at **0.70** in [`src/config.py`](../src/config.py) for this
reason, and `tests/test_config.py` fails if it is raised.

### Upgrade paths

**Four DRV8833s with channels paralleled** — tie `AIN1`↔`BIN1`, `AIN2`↔`BIN2`,
`AOUT1`↔`BOUT1`, `AOUT2`↔`BOUT2` on each chip, one chip per motor. Halves both
the voltage drop and the heat. Costs ~5 g, returns ~19 g of thrust. Only parallel
*within* one chip — two chips will not switch in sync.

**Four SI2302 MOSFETs** — drone props never reverse, so an H-bridge is not
needed. Recovers nearly all the loss, weighs ~1 g total, costs pennies. This is
what commercial coreless drones use.

## Weight budget

| Item | Weight |
|---|---|
| Pico 2 W (with headers) | ~10 g |
| 2× DRV8833 breakouts | ~5 g |
| GY-521 + HMC5883L | ~3 g |
| GY-GPS6MV2 | ~14 g |
| 1S 600 mAh LiPo | ~15 g |
| 4 motors + props | ~22 g |
| Frame, wire, connectors | ~25 g |
| **Total with GPS** | **~94 g** |
| **Total without GPS** | **~80 g** |

## Thrust-to-weight

You want **2:1 minimum** for controllable flight. Below ~1.5:1 the aircraft
cannot accelerate upward fast enough to correct a disturbance — it wallows and
eventually falls out of the sky regardless of PID tuning.

| Motor | Thrust each | Total | After driver loss | T:W with GPS | T:W without |
|---|---|---|---|---|---|
| 720 (7 mm) | ~18 g | 72 g | ~61 g | 0.65 — will not lift | 0.76 — will not lift |
| 8520 (8.5 mm) | ~35 g | 140 g | ~119 g | 1.27 — marginal | **1.49 — flyable** |

**Build v1 without the GPS.** It is 15% of all-up weight, and it is useless until
the stabilisation loop works anyway. It bolts on later in minutes.

With 8520 motors and no GPS you are at roughly 1.5:1 — enough to develop and tune
the control loop. Swapping to MOSFETs later takes it to ~1.76:1.
