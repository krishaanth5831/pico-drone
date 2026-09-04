## What this changes

<!-- One or two sentences. -->

## Why

<!-- What problem does this solve? -->

## Hardware verification

- [ ] Not applicable — software only, covered by CI
- [ ] Tested on the bench with **props off**
- [ ] Tested in flight

Which `testing/` procedures were re-run?

<!-- e.g. 03_coreless_motor, 07_lipo_power -->

## Safety checklist

- [ ] Any code that can drive motors disarms in a `finally` block or via
      `with MotorBank()`
- [ ] `MAX_DUTY` unchanged, or the thermal note in `testing/02_drv8833/README.md`
      updated to match
- [ ] Pin changes made in `src/config.py`, not hardcoded at the call site
- [ ] Wiring docs updated if any pin assignment moved

## Notes for the reviewer

<!-- Anything worth a closer look. -->

---

<sub>Merging is done by the repo owner only.</sub>
