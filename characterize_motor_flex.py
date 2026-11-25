#!/usr/bin/env python3
"""Characterize and model the exo finger (motor <-> flex sensor), alone and with the hand.

The motor sweeps the finger between its two ends in steps, holds at each step and logs the flex
resistance + real motor position, in both directions, for several cycles.
Analysis: calibration curve, polynomial model (both directions), hysteresis, repeatability.
Models are saved as JSON; several CSVs can be analysed together to compare the settings.

Place at the repo root, next to flex_triggered_motor.py.

    # 1) finger alone
    python characterize_motor_flex.py --start open --setting alone
    # 2) finger on the hand
    python characterize_motor_flex.py --start open --setting hand
    # 3) analyse / compare (no hardware needed)
    python characterize_motor_flex.py --analyze characterization/alone_*.csv characterization/hand_*.csv
"""
import argparse
import csv
import json
import time
from datetime import datetime
from pathlib import Path

import numpy as np

COLUMNS = ["t", "cycle", "direction", "step", "goal", "pos", "R"]


# ============================== ACQUISITION ==============================
def acquire(args):
    import flex_triggered_motor as ftm

    if args.velocity is not None:
        ftm.PROFILE_VELOCITY = args.velocity
    dist = args.distance if args.distance is not None else ftm.MOVE_DISTANCE

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    csv_path = out / f"{args.setting}_{datetime.now():%Y%m%d_%H%M%S}.csv"

    ser = ftm.open_port(args.port)
    A = ftm.setup_motor()                                   # start position = end A
    B = A - dist if args.start == "open" else A + dist      # other end
    N = args.steps

    # sequence: initial point, then per cycle A->B ("fwd") and B->A ("back")
    seq = [(0, "init", 0)]
    for c in range(1, args.cycles + 1):
        seq += [(c, "fwd", i) for i in range(1, N + 1)]
        seq += [(c, "back", i) for i in range(N - 1, -1, -1)]
    print(f"[{args.setting}] {len(seq)} points, about {len(seq) * (args.dwell + 1) / 60:.0f} min. "
          f"Keep still. Ctrl+C stops.")

    f = open(csv_path, "w", newline="")
    f.write(f"# setting={args.setting}\n# pos_A={A}\n# pos_B={B}\n# steps={N}\n")
    w = csv.writer(f)
    w.writerow(COLUMNS)
    T0 = time.time()

    def sample(cycle, direction, i, goal, log):
        ser.reset_input_buffer()                            # fresh sensor sample
        r = ftm.read_resistance(ser)
        pos = ftm.read_position()
        if log and r is not None:
            w.writerow([f"{time.time() - T0:.3f}", cycle, direction, i, goal,
                        "" if pos is None else pos, f"{r:.1f}"])
        return pos

    try:
        for cycle, direction, i in seq:
            goal = round(A + (B - A) * i / N)
            if not ftm.set_goal(goal):
                raise RuntimeError("could not command the motor")
            t0 = time.time()                                # move (not logged)
            while time.time() - t0 < args.move_timeout:
                pos = sample(cycle, direction, i, goal, False)
                if pos is not None and abs(pos - goal) <= ftm.TOLERANCE:
                    break
            t_end = time.time() + args.dwell                # hold + log
            while time.time() < t_end:
                pos = sample(cycle, direction, i, goal, True)
            f.flush()
            print(f"cycle {cycle}/{args.cycles} {direction:<5} step {i:>2}/{N} goal {goal} pos {pos}")
    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        ftm.release_motor()                                 # torque OFF, always
        ser.close()
        f.close()
    print(f"Saved {csv_path}")
    return csv_path


# ============================== ANALYSIS ==============================
def load_visits(path, settle):
    """One visit per (cycle, direction, step): mean R and mean pos after the settling time."""
    meta, lines = {}, []
    for line in open(path):
        if line.startswith("#"):
            k, _, v = line[1:].strip().partition("=")
            meta[k] = v
        else:
            lines.append(line)
    groups = {}
    for d in csv.DictReader(lines):
        groups.setdefault((int(d["cycle"]), d["direction"], int(d["step"])), []).append(d)
    visits = []
    for (c, dr, s), g in groups.items():
        t0 = float(g[0]["t"])
        g = [d for d in g if float(d["t"]) >= t0 + settle]
        pos = [int(d["pos"]) for d in g if d["pos"] != ""]
        if len(g) < 3 or not pos:
            continue
        visits.append(dict(cycle=c, dir=dr, step=s, pos=np.mean(pos),
                           R=np.mean([float(d["R"]) for d in g])))
    return meta, visits


def analyze(paths, skip, settle, deg, make_plot=True):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(7, 5))
    colors = ["tab:blue", "tab:red", "tab:green", "tab:orange"]

    for k, path in enumerate(paths):
        meta, visits = load_visits(path, settle)
        name = meta.get("setting", Path(path).stem)
        inc = [v for v in visits if v["cycle"] >= skip + 1] or [v for v in visits if v["cycle"] >= 1]
        if len(inc) < 6:
            print(f"{path}: not enough data")
            continue
        N = max(v["step"] for v in inc)

        R_ref = np.mean([v["R"] for v in inc if v["step"] == 0])   # open end -> 0 %
        for v in inc:
            v["x"] = (v["R"] - R_ref) / R_ref * 100                 # dR/R [%]
        P = np.array([v["pos"] for v in inc])
        X = np.array([v["x"] for v in inc])

        # model: pos -> dR/R (sensor prediction) and dR/R -> pos (what a controller needs)
        c_fwd = np.polyfit(P, X, deg)
        c_inv = np.polyfit(X, P, deg)
        rms = float(np.sqrt(np.mean((X - np.polyval(c_fwd, P)) ** 2)))
        rms_pos = float(np.sqrt(np.mean((P - np.polyval(c_inv, X)) ** 2)))

        # hysteresis and repeatability
        mean = {}
        for v in inc:
            mean.setdefault((v["dir"], v["step"]), []).append(v["x"])
        hyst = [abs(np.mean(mean[("fwd", s)]) - np.mean(mean[("back", s)]))
                for s in range(1, N) if ("fwd", s) in mean and ("back", s) in mean]
        stds = [np.std(x, ddof=1) for x in mean.values() if len(x) > 1]
        span = abs(np.polyval(c_fwd, P.max()) - np.polyval(c_fwd, P.min()))

        print(f"\n=== {name}  ({path}) ===")
        print(f"R_ref (open) : {R_ref / 1000:.2f} kOhm")
        print(f"span         : {span:.1f} % over {P.min():.0f}..{P.max():.0f} ticks")
        print(f"model fit    : deg {deg}, residual RMS {rms:.2f} % (= {rms_pos:.0f} ticks)")
        if hyst:
            print(f"hysteresis   : max {max(hyst):.2f} %  mean {np.mean(hyst):.2f} %")
        if stds:
            print(f"repeatability: cycle-to-cycle std mean {np.mean(stds):.2f} %  max {np.max(stds):.2f} %")

        out = Path(path).with_suffix("")
        json.dump(dict(setting=name, R_ref=R_ref, degree=deg,
                       pos_to_dRR_percent=c_fwd.tolist(), dRR_percent_to_pos=c_inv.tolist(),
                       pos_min=float(P.min()), pos_max=float(P.max()),
                       rms_percent=rms, rms_ticks=rms_pos,
                       hysteresis_max_percent=max(hyst) if hyst else None),
                  open(f"{out}_model.json", "w"), indent=2)
        print(f"model saved  : {out}_model.json")

        c = colors[k % len(colors)]
        for dr, mk in (("fwd", "o"), ("back", "x")):
            vd = [v for v in inc if v["dir"] == dr]
            ax.scatter([v["pos"] for v in vd], [v["x"] for v in vd], s=14, marker=mk, color=c, alpha=0.5)
        pp = np.linspace(P.min(), P.max(), 100)
        ax.plot(pp, np.polyval(c_fwd, pp), color=c, lw=2, label=f"{name} (RMS {rms:.1f} %)")

    ax.set_xlabel("motor position [ticks]")
    ax.set_ylabel("dR/R [%]")
    ax.set_title("Flex sensor vs motor position (o = fwd, x = back)")
    ax.grid(alpha=0.3)
    ax.legend()
    if make_plot:
        png = Path(paths[0]).parent / ("comparison.png" if len(paths) > 1 else Path(paths[0]).stem + ".png")
        fig.tight_layout()
        fig.savefig(png, dpi=130)
        print(f"\nFigure saved : {png}")


# ============================== MAIN ==============================
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--analyze", nargs="+", metavar="CSV", help="analyse one or more CSVs (no hardware)")
    ap.add_argument("--start", choices=["open", "closed"], help="finger OPEN or CLOSED at the start")
    ap.add_argument("--setting", choices=["alone", "hand"], help="test condition label")
    ap.add_argument("-p", "--port", help="flex sensor serial port (auto-detected if omitted)")
    ap.add_argument("--steps", type=int, default=10)
    ap.add_argument("--cycles", type=int, default=4)
    ap.add_argument("--dwell", type=float, default=6.0, help="seconds held at each step")
    ap.add_argument("--settle", type=float, default=2.0, help="seconds of each dwell discarded before averaging")
    ap.add_argument("--distance", type=int, help="ticks between the ends (default: MOVE_DISTANCE)")
    ap.add_argument("--velocity", type=int, help="profile velocity override")
    ap.add_argument("--move-timeout", type=float, default=20.0)
    ap.add_argument("--degree", type=int, default=2, help="polynomial degree of the model")
    ap.add_argument("--skip-cycles", type=int, default=1, help="initial cycles excluded from the fit")
    ap.add_argument("--out", default="characterization")
    args = ap.parse_args()

    if args.analyze:
        analyze(args.analyze, args.skip_cycles, args.settle, args.degree)
        return
    if not args.start or not args.setting:
        ap.error("--start and --setting are required (or use --analyze CSV...)")
    if args.dwell <= args.settle + 1:
        ap.error("--dwell must be larger than --settle + 1")
    path = acquire(args)
    analyze([path], args.skip_cycles, args.settle, args.degree)


if __name__ == "__main__":
    main()