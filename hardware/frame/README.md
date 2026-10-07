# Frame — 3D-printed airframe

A one-piece printed quad frame sized around this build's parts, a bolt-on
GPS + compass mast, and a small tolerance coupon to print first.

| File | Part | Mass (PLA) |
|---|---|---|
| `pico_drone_frame.stl` | Frame: deck, arms, motor pods, battery bay | 8.7 g |
| `pico_drone_gps_mast.stl` | GPS + compass mast (bolts on, optional) | 2.7 g |
| `pico_drone_tolerance_coupon.stl` | Fit test: motor bores + header slots | — |
| `pico_drone_frame.fs` | Onshape FeatureScript source that generates all three | — |

The model is fully parametric in Onshape (document **Pico Drone Frame**,
private: <https://cad.onshape.com/documents/835a5cba0ddda1a520f81383>).
Double-click the *Pico Drone Frame* feature to change any dimension or
clearance, and everything regenerates. To reproduce it in another document,
paste `pico_drone_frame.fs` into a new Feature Studio and add the feature to a
Part Studio.

> **Thrust check before you print.** With the 720 motors, `docs/power.md`
> puts usable thrust at about 61 g. The fit-check model weighs the full build
> at **65.7 g without wires** (with GPS), so it will not lift. 8520 motors
> (~119 g usable) give about 1.5:1 with the GPS fitted. To refit the pods,
> set *Motor diameter* to 8.5 and re-export — the 55 mm props and every other
> dimension stay the same.

## Print

The STLs are already in print orientation: flat deck face (or platform face)
on the bed. **No supports.** Every overhang is a bridge or a 45° slope.

- PLA, 0.4 mm nozzle, 0.2 mm layers, 3 walls, any infill. Most walls are
  1.0–1.6 mm thick, so they print solid.
- Every corner is rounded and every edge has a 0.4 mm fillet (the 8 bottom
  edges of the arm webs take a smaller one), so there are no sharp edges.
  The only unrounded edges are inside the shallow 0.4 mm engraved labels.
- **Print the coupon first** (about 10 minutes):
  - **Motor rings:** 5 rings with bore allowances −0.1 / 0.0 / +0.1 /
    +0.2 / +0.3 mm, marked by 1–5 dots. Pick the ring that grips a motor
    firmly without forcing it. If that isn't the +0.1 ring (3 dots), set
    *Motor bore allowance* to its value and re-export.
  - **Header slots:** 3 slots of 3.0 / 3.4 / 3.8 mm. A 2.54 mm header strip
    should drop into the 3.4 slot freely. If it doesn't, change *Header slot
    width*.

## Layout

Nose is +X. Motor labels are engraved on the arms, and arrows on the front
two arms point forward. Rotation directions match `firmware/README.md`:

```
                  nose
      M3 (CW)              M1 (CCW)
         \                   /
          \ +--------+------+
            |        | DRV1 |
            |  Pico  |------|
            |  2 W   | IMU  |
            |        |------|
            |  USB   | DRV2 |
          / +--------+------+ \
         /     battery under    \
      M2 (CCW)              M4 (CW)
```

- The **Pico** sits in the left column with its USB port at the rear, where a
  notch in the deck leaves room for the cable. The antenna end faces the
  front edge, partly over a window in the deck.
- The **GY-521** sits in the middle of the right column, as close to the centre
  of mass as the layout allows (computed CG offset 1.5 mm, -0.2 mm). Turn it
  so the silkscreen **X arrow points to the nose**. That edge has slots on
  both sides, so either 180° orientation fits. If your board's X arrow runs
  along the short edge, tick *GY-521 silkscreen X runs along the short edge*.
- The **battery** (EEMB 852040, 20 × 40 × 8.5 mm) slides into the bay under
  the deck from the rear. It stops against the front wall and is held down by
  two 45° rails. Put a zip tie or rubber band through the rear truss windows
  so it can't slide out.

## Fitting the boards (headers stay on, pins down)

Each board's header plastic drops through a slot, so the PCB lies flat on the
deck. Its pins hang into a 10 mm wiring bay above the battery. Solder wires to
the pin tips. Dupont housings need about 18 mm: set *Wiring gap under deck* to
18 if you use them.

| Board | Held by |
|---|---|
| Pico 2 W, DRV8833 ×2 | header strips in the slots + thin double-sided tape |
| GY-521 | 1 mm foam tape (vibration isolation, per `testing/04_gy521_imu`) |
| GY-GPS6MV2 + antenna, GY-271 | foam tape on the mast platform; antenna taped on top of the NEO-6M module. Turn the GY-271 so its silkscreen X arrow points to the nose (either 180° orientation fits), and recalibrate it once it's on the airframe |
| Mast | leg tabs drop through the deck slots, 2× M2 × 6 self-tapping screws from the front and rear |
| Motors | split clamp with a 45° bottom lip; leads exit through the bottom hole |

Motor leads run along the arm webs; the two holes in each web take a zip tie.

## Tolerances

| Fit | Clearance |
|---|---|
| Board pockets | 0.3 mm per side |
| Header strip in slot (Pico, IMU, GY-271, GPS) | 3.4 mm slot for a 2.54 mm strip, 0.43 mm per side |
| DRV8833 header slots | 3.8 mm (row spacing on clone boards varies) |
| Motor bore | Ø motor + 0.1 mm with a 0.8 mm split, so the clamp grips |
| Battery bay | 0.4 mm per side |
| Mast tab in deck slot | 0.15 mm per side |

These were checked in Onshape against fit-check dummies of every part.
Minimum clearances: 0.05 mm board-to-deck (the dummies float by design),
0.26 mm at the header strip corners, 0.2 mm battery, 0.43 mm GPS header, and
more than 1 mm between the props and anything else.

## Dimensions used

- **Pico 2 W**: official datasheet. 51 × 21 × 1 mm, header rows 17.78 mm
  apart, 1.3 mm USB overhang, 14 × 9 mm antenna keep-out.
- **Clone boards**: typical published sizes. **Caliper yours** and edit the
  dialog before printing; sellers differ by 1–2 mm.

| Board | Size used |
|---|---|
| GY-521 | 21.2 × 16.4 mm |
| DRV8833 (red EEP/ULT module) | 18.5 × 16 mm |
| GY-271 | 14.8 × 13.5 mm |
| GY-GPS6MV2 | 36 × 25 mm; antenna 25 × 25 × 6.2 mm |
| 852040 battery | 42 × 20 × 8.5 mm (2 mm allowance for the protection PCB) |
| 720 motor, 55 mm props | Ø7 × 20 mm |

## Re-exporting after a change

Onshape exports in flight orientation. Either flip the part 180° about X in
the slicer, or use *lay flat* on the large deck face.
