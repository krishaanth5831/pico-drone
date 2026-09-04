# Working agreements for this repo

## Branching

Plain version control. One branch, `main`. No branch protection, no required
PRs, no CI. Commit and push directly.

Claude does not create or merge pull requests here unless explicitly asked to -
not because of any repo-level gate, just as a matter of not taking actions the
user has not asked for.

## Hardware safety rules that affect code

- Every script that can drive motors must pull the arm pin (GP15) **low** at
  start and in a `finally` block. No exceptions.
- Never write a script that spins motors at power-on without an explicit arm step.
- `MAX_DUTY` stays at or below 0.70 while running two DRV8833s. They thermally
  shut down at sustained full throttle.
- Test scripts assume **props removed** unless the doc says otherwise in bold.

## Docs

- Every component under `testing/` has a `README.md` with a wiring table using
  **physical pin numbers**, and a runnable script.
- Every procedure must have at least one step that produces something you can
  *see* — REPL output, an LED, a spinning motor, live NMEA. Never document a
  headless-only flow.

## Style

- MicroPython on the RP2350. Target the `rp2` port.
- Comment generously, especially anything touching hardware registers or timing.
- Hardware code goes in `src/drivers/`, control maths in `src/flight/`.
- Anything in `src/` must import cleanly under CPython with `tests/mocks/` on the
  path, so it can be tested without a board attached.
