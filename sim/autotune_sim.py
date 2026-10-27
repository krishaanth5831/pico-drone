"""
Run the board's autotune (src/flight/autotune.py) on simulated drones.

    cd sim
    uv run python autotune_sim.py --windowed       # watch one autotune flight, then the result
    uv run python autotune_sim.py --windowed --com 3 -2      # wires pulled the CoM 3 mm fwd, 2 mm right
    uv run python autotune_sim.py --windowed --extreme "slow motors"
    uv run python autotune_sim.py --windowed --random        # a randomized drone (--seed picks which)
    uv run python autotune_sim.py                  # nominal drone: identify, scale, compare
    uv run python autotune_sim.py --check 12       # 12 randomized + 6 extreme drones
    uv run python autotune_sim.py --reference      # regenerate autotune.REFERENCE

It flies exactly the board's sequence - hover on lift.py's gains, then roll,
pitch, yaw square-wave excitation - and prints, per axis, what was identified
next to the sim's ground truth (the sim knows the real torque per command and
the CoM offset), the gain scale it chose, and a before/after comparison on the
same drone with kicks: lift.py's gains vs autotuned gains + trim.

Writes runs/autotune_gains.json, which `watch.py --gains autotune` and
`fly.py --gains autotune` fly (give fly.py the same --com to fly the same drone).
"""

import argparse
import copy
import json
import time
from pathlib import Path

import mujoco
import numpy as np

from pico_drone_sim.controllers import LIFT_GAINS, PIDController, rollout
from pico_drone_sim.env import HoverEnv
from pico_drone_sim.params import describe, randomize

# isort: split
# (pico_drone_sim above puts ../src on sys.path, so `flight` imports after it)
from flight.autotune import AXES, Autotune
from flight.mixer import mix

OUT = Path(__file__).resolve().parent / "runs" / "autotune_gains.json"
BASE = {name: LIFT_GAINS[name + "_rate"] for name in AXES}
ABORT_TILT_DEG = 25.0   # firmware/autotune.py stops exciting past this

# Drones at the edges of drone_params.json's ranges, where one fixed set of
# gains should start to struggle and the autotune has to earn its keep.
EXTREMES = {
    "weak motors, heavy": {"thrust_full_g": 9.0, "inertia_scale": 1.25, "extra_mass_g": 20.0},
    "strong motors, light": {"thrust_full_g": 18.0, "inertia_scale": 0.85, "extra_mass_g": 2.0},
    "slow motors": {"motor_tau_up_ms": 60.0, "motor_tau_down_ms": 100.0},
    "near full throttle": {"hover_throttle": 0.75},
    "near idle": {"hover_throttle": 0.52},
    "CoM way off, slow loop": {"com_offset_mm": [4.0, -4.0, -6.0], "control_hz": 80.0,
                               "latency_ms": 8.0},
}


def truth(env):
    """
    The real model the fit should find, straight from the physics: angular
    acceleration (deg/s^2) per unit roll/pitch/yaw command at hover throttle,
    and the command that cancels the CoM-offset twist (true trim).
    """
    d = env.drone
    saved = (d.data.qpos.copy(), d.data.qvel.copy())
    thr = env.p.hover_throttle

    def accel(cmd):
        d.data.qpos[3:7] = [1, 0, 0, 0]
        d.data.qvel[:] = 0
        out = mix(thr, *cmd, idle=0.0)
        d.data.ctrl[:] = d._targets(np.array([out[m] for m in (1, 2, 3, 4)]))
        d._external_forces()
        mujoco.mj_forward(d.model, d.data)
        return np.degrees(d.data.qacc[3:6].copy())

    base = accel((0, 0, 0))
    res = {}
    for i, name in enumerate(AXES):
        cmd = [0.0, 0.0, 0.0]
        cmd[i] = 0.02
        k = (accel(cmd)[i] - base[i]) / 0.02
        res[name] = {"K": k, "trim": -base[i] / k}
    d.data.qpos[:], d.data.qvel[:] = saved
    mujoco.mj_forward(d.model, d.data)
    return res


def fly_autotune(env, seed, params=None, viewer=None):
    """One autotune flight, as the board flies it. Returns (Autotune, aborted)."""
    at = Autotune()
    env.episode_s = at.total_s + 3.0
    env.kick_rate_hz = 0.0
    env.trim = (0.0, 0.0)
    # Calm start: the board starts its autotune from a steady hover.
    opts = {"ground": False, "tilt_deg": 3.0, "rate_scale": 0.0}
    if params is not None:
        opts["params"] = params
    env.reset(seed=seed, options=opts)
    pid = PIDController()
    aborted = False
    while not at.done:
        t0 = time.perf_counter()
        exc = at.excitation(env.t)
        _, _, term, _, _ = env.step_cmd(pid(env, env.period) + np.array(exc))
        at.record(env.out)
        at.observe(env.gyro, env.period)
        # The board only knows its estimate, so it aborts on that.
        if (abs(env.est[0] - env.level[0]) > ABORT_TILT_DEG
                or abs(env.est[1] - env.level[1]) > ABORT_TILT_DEG or term):
            aborted = True
            break
        if viewer is not None:
            if not viewer.is_running():
                break
            viewer.cam.lookat[:] = env.drone.position()
            viewer.sync()
            time.sleep(max(0.0, env.period - (time.perf_counter() - t0)))
    return at, aborted


def tuned(res):
    """(gains dict for PIDController, (roll trim, pitch trim)) from Autotune.results()."""
    g = dict(LIFT_GAINS)
    for name in AXES:
        if res[name]["gains"] is not None:
            g[name + "_rate"] = res[name]["gains"]
    trim = tuple(res[n]["model"]["trim"] if res[n]["model"] else 0.0 for n in ("roll", "pitch"))
    return g, trim


def score(env, params, gains, trim=(0.0, 0.0), seeds=range(10)):
    """Crash count and mean RMS tilt (deg) over kicked episodes on one drone."""
    crashes, rms = 0, []
    for s in seeds:
        env.kick_rate_hz = 0.5
        env.episode_s = 8.0
        env.trim = trim
        log = rollout(env, PIDController(gains), options={"params": params, "ground": False,
                                                           "tilt_deg": 15}, seed=s)
        crashes += log["crashed"]
        r, q = np.array(log["roll"]), np.array(log["pitch"])
        rms.append(float(np.sqrt(np.mean(r * r + q * q))))
    env.trim = (0.0, 0.0)
    return crashes, float(np.mean(rms))


def report(env, res, verbose=True):
    """Print found vs true per axis. Returns {axis: (K rel. error, trim error)}."""
    tr = truth(env)
    if verbose:
        print("  axis   K found / true (deg/s^2 per unit)   tau ms  dead ms   trim found / true   gain x")
    errs = {}
    for name in AXES:
        r = res[name]
        m = r["model"]
        if m is None:
            print("  %-5s  no fit: %s - keeps lift.py's gains" % (name, r["why"]))
            continue
        errs[name] = (m["K"] / tr[name]["K"] - 1.0, m["trim"] - tr[name]["trim"])
        if verbose:
            print("  %-5s  %7.0f / %7.0f  (%+4.0f%%)   %5.0f    %5.1f     %+.3f / %+.3f    %s"
                  % (name, m["K"], tr[name]["K"], 100 * errs[name][0], 1000 * m["tau_s"],
                     1000 * m["dead_s"], m["trim"], tr[name]["trim"],
                     "%.2f" % r["scale"] if r["scale"] else "-"))
    return errs


def make_reference(seeds=5):
    """Average identified K and lag on the nominal drone -> autotune.REFERENCE."""
    env = HoverEnv(randomize_scale=0.0)
    acc = {n: [] for n in AXES}
    for s in range(seeds):
        at, aborted = fly_autotune(env, s)
        assert not aborted, "nominal drone aborted its autotune"
        for n in AXES:
            m = at.models[n].result()
            acc[n].append((m["K"], m["lag_s"]))
    print("REFERENCE = {")
    for n in AXES:
        k = np.mean([a[0] for a in acc[n]])
        lag = np.mean([a[1] for a in acc[n]])
        spread = 100 * np.std([a[0] for a in acc[n]]) / k
        print('    "%s": (%.0f, %.4f),   # K spread over %d runs: %.0f %%' % (n, k, lag, seeds, spread))
    print("}")


def check(n_random, seed):
    env = HoverEnv(randomize_scale=1.0, seed=seed)
    drones = []
    for i in range(n_random):
        drones.append(("random %d" % i, None, 1000 + i))
    for label, mods in EXTREMES.items():
        p = copy.deepcopy(env.nominal)
        for k, v in mods.items():
            setattr(p, k, v)
        drones.append((label, p, 0))

    k_err, t_err, worse, better, aborts = [], [], [], 0, 0
    for label, p, s in drones:
        at, aborted = fly_autotune(env, s, params=p)
        p = env.p
        print("%-22s %s" % (label, describe(p)))
        if aborted:
            aborts += 1
            print("   ABORTED (tilt > %.0f deg) - keeps lift.py's gains" % ABORT_TILT_DEG)
            continue
        res = at.results(BASE)
        errs = report(env, res, verbose=False)
        for n, (ke, te) in errs.items():
            k_err.append(abs(ke))
            if n != "yaw":
                t_err.append(abs(te))
        g, trim = tuned(res)
        c0, r0 = score(env, p, LIFT_GAINS)
        c1, r1 = score(env, p, g, trim)
        scales = " ".join("%s x%.2f" % (n, res[n]["scale"]) if res[n]["scale"] else "%s -" % n
                          for n in AXES)
        verdict = "WORSE" if (c1 > c0 or r1 > 1.1 * r0) else ("better" if (c1, r1) < (c0, r0) else "same")
        better += verdict == "better"
        if verdict == "WORSE":
            worse.append(label)
        print("   %s | K err %s | lift.py: %d/10 crashed, %.1f deg | autotuned: %d/10, %.1f deg  %s"
              % (scales, " ".join("%+.0f%%/R2 %.2f" % (100 * errs[n][0], res[n]["model"]["r2"]) for n in errs), c0, r0, c1, r1, verdict))
    print("\n%d drones, %d aborted. |K error| median %.0f%%, 90th pct %.0f%%; |trim error| median %.3f"
          % (len(drones), aborts, 100 * np.median(k_err), 100 * np.percentile(k_err, 90),
             np.median(t_err)))
    print("autotuned better on %d, worse on %d%s" % (better, len(worse),
                                                     (": " + ", ".join(worse)) if worse else ""))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--windowed", action="store_true", help="watch the autotune flight")
    ap.add_argument("--check", type=int, default=0, help="run on N randomized + the extreme drones")
    ap.add_argument("--reference", action="store_true", help="print a new autotune.REFERENCE")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--com", type=float, nargs=2, metavar=("FWD_MM", "LEFT_MM"),
                    help="centre of mass moved this far by wires/tape (+x nose, +y left)")
    ap.add_argument("--extreme", choices=sorted(EXTREMES), help="one of the --check edge-case drones")
    ap.add_argument("--random", action="store_true", help="a randomized drone (--seed picks which)")
    args = ap.parse_args()

    if args.reference:
        make_reference()
        return
    if args.check:
        check(args.check, args.seed)
        return

    env = HoverEnv(randomize_scale=0.0, seed=args.seed)
    p = copy.deepcopy(env.nominal)
    if args.random:
        p = randomize(env.nominal, np.random.default_rng(args.seed), 1.0)
    for k, v in EXTREMES.get(args.extreme, {}).items():
        setattr(p, k, v)
    if args.com is not None:
        p.com_offset_mm = [args.com[0], args.com[1], p.com_offset_mm[2]]
    print("drone:", describe(p))
    if args.windowed:
        import mujoco.viewer

        with mujoco.viewer.launch_passive(env.drone.model, env.drone.data) as viewer:
            viewer.cam.distance = 0.6
            viewer.cam.elevation = -15
            print("flying the autotune: hover, then roll, pitch, yaw wiggles (~%.0f s)"
                  % Autotune().total_s)
            at, aborted = fly_autotune(env, args.seed, params=p, viewer=viewer)
            if aborted:
                print("aborted: tilted past %.0f deg" % ABORT_TILT_DEG)
                return
            res = at.results(BASE)
            report(env, res)
            gains, trim = tuned(res)
            p = env.p
            c0, r0 = score(env, p, LIFT_GAINS)
            c1, r1 = score(env, p, gains, trim)
            print("\nwith kicks, 10 episodes: lift.py gains %d crashed, rms tilt %.1f deg | "
                  "autotuned %d crashed, %.1f deg" % (c0, r0, c1, r1))
            print("\nnow flying the NEW gains + trim with kicks - close the window to finish")
            env.kick_rate_hz, env.episode_s, env.trim = 0.5, 1e9, trim
            env.reset(options={"params": p, "ground": False, "tilt_deg": 15})
            ctl = PIDController(gains)
            while viewer.is_running():
                t0 = time.perf_counter()
                _, _, term, _, _ = env.step_cmd(ctl(env, env.period))
                if term:
                    env.reset(options={"params": p, "ground": False, "tilt_deg": 15})
                    ctl.reset()
                viewer.cam.lookat[:] = env.drone.position()
                viewer.sync()
                time.sleep(max(0.0, env.period - (time.perf_counter() - t0)))
    else:
        at, aborted = fly_autotune(env, args.seed, params=p)
        if aborted:
            print("aborted: tilted past %.0f deg" % ABORT_TILT_DEG)
            return
        res = at.results(BASE)
        report(env, res)
        gains, trim = tuned(res)
        c0, r0 = score(env, env.p, LIFT_GAINS)
        c1, r1 = score(env, env.p, gains, trim)
        print("\nwith kicks, 10 episodes: lift.py gains %d crashed, rms tilt %.1f deg | "
              "autotuned %d crashed, %.1f deg" % (c0, r0, c1, r1))

    OUT.parent.mkdir(exist_ok=True)
    OUT.write_text(json.dumps({"gains": gains, "trim": trim, "drone": describe(env.p)}, indent=2))
    print("wrote", OUT, "- fly it: uv run python watch.py --gains autotune")


if __name__ == "__main__":
    main()
