# 07 — 1S LiPo power system

Verifies the power path and hunts for brownouts. Run this **last**, once
everything else works individually, because it is the test that explains
mysterious resets in all the others.

> **Props off.** This test runs all four motors together at high throttle.

## What you need

- 1S LiPo (3.7 V nominal, 3.0–4.2 V range)
- 2× 470 µF low-ESR electrolytic capacitors
- 1× Schottky diode (SS14, 1N5819, SR240 — anything ≥1 A)
- Multimeter

## Wiring

Motor current and Pico current take **separate paths from the battery**, meeting
only at the battery terminals:

```
1S LiPo (+) --+-------------------> DRV #1 VM --+-- 470uF -- GND
              |                                  |
              +-------------------> DRV #2 VM --+-- 470uF -- GND
              |
              +--[ SS14 Schottky ]-> Pico VSYS (physical pin 39)

1S LiPo (-) ---- star point ----+--> DRV #1 GND
                                +--> DRV #2 GND
                                +--> Pico GND (physical pin 38)
```

| Connection | Pico 2 W | Physical pin | Note |
|---|---|---|---|
| Battery + via Schottky | VSYS | **39** | VSYS accepts 1.8–5.5 V |
| Battery − (star) | GND | **38** | |
| Battery + direct | — | — | To both DRV8833 `VM` pins, not to the Pico |
| *(bench only)* USB 5 V | VBUS | **40** | Alternative to the battery for driver power on the bench |

**Never connect battery + to 3V3 (pin 36).** That regulator supplies ~300 mA.

### What each part is actually for

**The capacitors are the important ones.** A capacitor next to the driver acts as
a local energy reservoir. The battery's chemistry cannot respond to a millisecond
current spike; a capacitor discharges essentially instantly, supplying the
transient locally so the rail never dips far enough to reset the Pico. Mount them
**physically close** to each driver's VM/GND pins — wire inductance between cap
and chip defeats the point.

**The Schottky is about USB coexistence**, not sag. It stops USB's 5 V
backfeeding into the LiPo when both are connected. Without it, connect only one
at a time — forgetting once means overcharging a LiPo.

**Star-ground the negatives.** Do not daisy-chain DRV#1 → DRV#2 → Pico; motor
return current flowing through the Pico's ground wire creates offset voltages
that show up as IMU noise.

## What a brownout is

The battery is not an ideal voltage source. It has internal resistance, maybe
80–150 mΩ on a small 1S pack:

```
V_actual = V_battery - (I x R_internal)
```

At idle, drawing 50 mA, the drop is negligible. Punch the throttle and four
coreless motors pull ~6 A between them:

```
3.8 V - (6 A x 0.1 ohm) = 3.2 V
```

The Pico's regulator handles 3.2 V steady-state fine — VSYS is rated to 1.8 V.
The problem is the **fast transient**. During inrush the rail momentarily
collapses much lower, the 3V3 rail follows, and the RP2350's brownout detector
asserts reset, because a CPU below its minimum voltage produces garbage rather
than stopping cleanly.

**In flight that means the MCU resets mid-air**, all four motors stop, and the
aircraft falls. A second later it finishes booting and — without an arming check
— could spin motors up again while tumbling. That is why `MOTOR_SLEEP_PIN` exists
and why nothing in this repo arms motors at power-on.

## Running the test

Restrain the airframe, props off, battery connected. Run
`test_lipo_power.py`. It boots, records a boot counter, then ramps all four
motors together while watching for resets.

## What you should see

```
=== 1S LiPo power test ===
boot #1  (reset cause 1)

LED pulses for as long as this runs.
If the pulse stops and restarts, that IS the brownout - watch the board.

battery: measure across the pack now
  idle, motors off   -> expect 3.7-4.2 V
ramping all four motors to 0.70 over 3s
  measure again under load -> expect >3.3 V

  motors at 0.70 for 5s...
  still alive at 1s
  still alive at 2s
  ...
=== PASS: no reset during load ===
boot count stayed at 1
```

**The pass condition is that `boot #1` never becomes `boot #2`.** A reset shows
up as the script restarting from the top with an incremented counter.

### Watch the LED, not the shell

This is where the heartbeat earns its keep. During the load test your eyes are on
the multimeter, not the terminal — and **a brownout is visible as the LED pulse
stopping and restarting**. That is the reset happening, in real time, without
having to catch it scroll past.

The boot counter is the durable record; the LED is the live one. If they
disagree, trust the counter — it is written to flash and survives the reset that
a glance might miss.

Watch the multimeter across the battery as the ramp runs. A drop from 3.9 V to
3.5 V under load is normal. A drop below 3.0 V means the pack is too small, too
discharged, or its internal resistance is too high.

## If it fails

| Symptom | Cause |
|---|---|
| `boot #2` appears mid-test | Brownout confirmed. Add or move the capacitors closer to the drivers |
| LED pulse stops and restarts | Same thing, seen live — the board reset under load |
| Thonny disconnects when motors spool | Same thing, seen from the host side |
| Voltage under load drops below 3.0 V | Pack too small or too discharged. Charge it, or use a pack with a higher C rating |
| Resets only at high throttle | Reduce `MAX_DUTY`, then fix the capacitors |
| Drivers hot, motors cut out together | Thermal shutdown, not brownout. Check `nFAULT` and see `02_drv8833` |

## Reducing brownout risk

- **Keep battery leads short and thick.** Every centimetre of thin wire adds
  series resistance in exactly the wrong place.
- **Ramp throttle, never step it.** A gradual current increase gives the battery
  time to respond. This is why `drivers/motors.ramp()` exists and why the flight
  loop slew-limits its output.
