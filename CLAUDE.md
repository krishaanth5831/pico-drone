# Working agreements for this repo

## Branching and merging — hard rules

- `main` is protected. All work happens on `dev` or on feature branches cut from `dev`.
- Nothing is ever pushed directly to `main`. It is rejected by branch protection.
- **Claude never merges a pull request.** Not to `main`, not to `dev`, not ever.
  Claude may *create* a PR, and only when explicitly asked to. A human (the repo
  owner) performs every merge.
- Do not use `gh pr merge`, `gh api --method PUT .../merge`, or any equivalent.
- When a PR is ready, say so and hand over the URL. Stop there.

## Flow

```
feature branch ──PR──> dev ──PR──> main
                                    ^
                            human merges only
```

- Day-to-day work in progress goes to `dev`.
- `dev` -> `main` PRs are opened only when a milestone is genuinely ready.

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
  path, so CI can exercise it without a board attached.
