# 06 — GY-GPS6MV2 (u-blox NEO-6M) GPS

Position and ground speed. Optional for a first flight — the stabilisation loop
does not need it, and at 14 g it is a significant fraction of a coreless quad's
lift budget. See `docs/power.md` for the weight discussion.

## Wiring — UART0

| GY-GPS6MV2 pin | Pico 2 W | Physical pin | Note |
|---|---|---|---|
| `VCC` | 3V3(OUT) | **36** | Onboard MIC5205 LDO drops ~110 mV at 50 mA, giving the NEO-6M ~3.19 V — inside its 2.7–3.6 V range |
| `GND` | GND | **3** | Pin 3 sits directly below GP0/GP1, keeping the wire short |
| `TX` | GP1 = UART0 **RX** | **2** | **Crossover** |
| `RX` | GP0 = UART0 **TX** | **1** | **Crossover**, only needed to reconfigure the module |

### Two things that catch people out

**Check your silkscreen before soldering.** These boards ship with the header in
two different orders — some `VCC RX TX GND`, others `GND TX RX VCC`. The label is
authoritative, not the position.

**TX goes to RX.** GPS transmit connects to Pico receive. Wiring TX→TX gives
total silence with no error message, and is the single most common mistake with
this module.

You can omit the GPS `RX` wire entirely if you only read the default 1 Hz stream.
Wire it anyway — you will want it to push the module to 5 Hz.

## Mounting

- **Antenna (the ceramic patch) faces up**, with a clear view of sky.
- Mount it **away from motors and power wiring**. GPS signals arrive at roughly
  −130 dBm, far below the switching noise your drivers and brushed motors
  radiate. On a small quad this means a nylon standoff lifting it a couple of
  centimetres above the frame.

## Running the test

**Take it outside.** The NEO-6M will not get a fix indoors — it needs actual
sky, and there is no way around that.

Run `test_gy_gps6mv2.py`. It dumps raw NMEA first (which proves the wiring), then
switches to parsed output and waits for a fix.

## What you should see

**Stage 1 — raw NMEA, immediately, indoors or out:**

```
--- raw NMEA for 10s (proves wiring) ---
$GPRMC,,V,,,,,,,,,,N*53
$GPGGA,,,,,,0,00,99.99,,,,,,*48
$GPGSV,1,1,00*79
```

Mostly empty comma fields is **correct** with no fix yet. What matters is that
sentences arrive at all.

**Stage 2 — waiting, then a fix:**

```
--- waiting for fix (up to 300s, outdoors only) ---
waiting for fix... 0 sats visible
waiting for fix... 4 sats visible
FIX ACQUIRED after 68s
  latitude   : 51.507351
  longitude  : -0.127758
  altitude   : 21.3 m
  satellites : 7
```

**Watch the red PPS LED on the module.** It starts blinking once per second the
moment a fix is acquired — that is hardware confirmation, independent of any
code. Cold start outdoors takes 30 s to several minutes; the onboard backup
battery then makes later warm starts a few seconds.

**The onboard LED pulses throughout.** If it stops and restarts, the board reset — see [07](../07_lipo_power/README.md).

## If it fails

| Symptom | Cause |
|---|---|
| No output at all | TX/RX swapped. Swap GP0 and GP1 — this fixes it most of the time |
| Garbage bytes | Baud mismatch. Try 38400; some clones ship reconfigured |
| Sentences arrive, never a fix, indoors | Expected. Go outside |
| Sentences arrive, no fix outdoors after 10 min | Antenna face-down, or too close to the power wiring |
| `checksum errors` climbing | Noisy wiring, or baud slightly off |
