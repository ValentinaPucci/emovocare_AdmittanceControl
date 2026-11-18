#!/usr/bin/env python3
"""Live plot of flex sensor bending (needs scripts/flex_shape.py + read_serial.py alongside).

Left   : the sensor drawn as a constant-curvature arc (base clamped at the origin, 95 mm)
Right  : dR/R0 history (top) and tip offset history (bottom)

Keys: z = re-zero (hold the sensor flat ~2 s first), q = quit.

  python scripts/flex_plot.py
  python scripts/flex_plot.py --window 30 --smooth 8
"""
from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
from collections import deque

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

from flex_shape import CAL_PATH, L_MM, load_model, measure_rest, open_port, poly_eval, resolve_r0, tip_geometry 
from read_serial import parse_sample

SAMPLE_HZ = 50


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", "--port")
    ap.add_argument("--window", type=float, default=20.0, help="history length in seconds")
    ap.add_argument("--smooth", type=int, default=5, help="moving-average window in samples")
    ap.add_argument("--gain", type=float,
                    help="ASSUMED linear gain G = (dR/R0) per (1/m) of curvature. "
                         "Overrides the calibration file. Shape is then only indicative.")
    ap.add_argument("--r0", type=float, help="flat resistance in ohms (skips zeroing)")
    ap.add_argument("--start-angle", type=float,
                    help="bend angle in degrees at startup, + = convex (no zeroing)")
    ap.add_argument("--saved-flat", action="store_true",
                    help="use R0 saved by 'flex_shape.py flat'")
    args = ap.parse_args()

    cal, src = load_model(args.gain)
    print(f"Using {src}")

    ser = open_port(args.port)
    ser.timeout = 0.02
    state = {"R0": resolve_r0(ser, cal, args.r0, args.start_angle, args.saved_flat)}
    print(f"R0 = {state['R0']/1000:.3f} kOhm")

    n_hist = int(args.window * SAMPLE_HZ * 1.2)
    raw = deque(maxlen=max(args.smooth, 2 * SAMPLE_HZ))   # for smoothing and re-zero
    T, DR, TIPY, RK = (deque(maxlen=n_hist) for _ in range(4))
    ts0 = {"v": None}
    rxbuf = bytearray()

    # ------------------------------------------------------------ figure
    fig = plt.figure(figsize=(12, 6))
    gs = fig.add_gridspec(2, 2, width_ratios=[1.1, 1])
    ax_shape = fig.add_subplot(gs[:, 0])
    ax_dr = fig.add_subplot(gs[0, 1])
    ax_bot = fig.add_subplot(gs[1, 1], sharex=ax_dr)

    ax_shape.set_aspect("equal")
    ax_shape.set_xlim(-0.35 * L_MM, 1.1 * L_MM)
    ax_shape.set_ylim(-0.9 * L_MM, 0.9 * L_MM)
    ax_shape.axhline(0, color="0.8", lw=1, ls="--")
    ax_shape.set_xlabel("x (mm)")
    ax_shape.set_ylabel("y (mm)   [+ = convex]")
    ax_shape.grid(alpha=0.3)
    ax_shape.plot([0], [0], "ks", ms=10, label="clamp")
    ref_line, = ax_shape.plot([0, L_MM], [0, 0], color="0.75", lw=3, label="rest")
    (arc_line,) = ax_shape.plot([0, L_MM], [0, 0], color="tab:blue", lw=5, solid_capstyle="round")
    (tip_dot,) = ax_shape.plot([L_MM], [0], "o", color="tab:red", ms=9, label="tip")
    off_line, = ax_shape.plot([L_MM, L_MM], [0, 0], color="tab:red", lw=1.5, ls=":")
    info = ax_shape.text(0.02, 0.97, "", transform=ax_shape.transAxes, va="top",
                         family="monospace", fontsize=10)
    ax_shape.legend(loc="lower right")

    (dr_line,) = ax_dr.plot([], [], color="tab:blue")
    ax_dr.set_ylabel("dR / R0 (%)")
    ax_dr.grid(alpha=0.3)
    plt.setp(ax_dr.get_xticklabels(), visible=False)

    (bot_line,) = ax_bot.plot([], [], color="tab:red" if cal else "tab:green")
    ax_bot.set_ylabel("tip offset (mm)" if cal else "R (kOhm)")
    ax_bot.set_xlabel("time (s)")
    ax_bot.grid(alpha=0.3)

    if cal is None:
        ax_shape.text(0.5, 0.5, "No calibration\nrun: flex_shape.py calibrate",
                      transform=ax_shape.transAxes, ha="center", va="center", fontsize=12, color="0.4")

    fig.suptitle("SpectraFlex live bending (constant-curvature model)"
                 + ("  -  ASSUMED GAIN, NOT CALIBRATED" if args.gain else ""))
    fig.tight_layout()

    # ------------------------------------------------------------ interaction
    def on_key(event):
        if event.key == "z" and len(raw) >= SAMPLE_HZ:
            state["R0"] = statistics.mean(list(raw)[-SAMPLE_HZ * 2:])
            print(f"Re-zeroed: R0 = {state['R0']/1000:.3f} kOhm")
        elif event.key == "q":
            plt.close(fig)

    fig.canvas.mpl_connect("key_press_event", on_key)

    # ------------------------------------------------------------ update loop
    def ingest() -> int:
        n = 0
        waiting = ser.in_waiting
        if waiting:
            rxbuf.extend(ser.read(waiting))
        *lines, rest = rxbuf.split(b"\n")
        rxbuf[:] = rest                                   # keep incomplete tail
        for ln in lines:
            s = parse_sample(ln.decode("utf-8", errors="replace").strip())
            if s is None or not math.isfinite(s.resistance_ohm):
                continue
            if ts0["v"] is None:
                ts0["v"] = s.timestamp_ms
            raw.append(s.resistance_ohm)
            R = statistics.mean(list(raw)[-args.smooth:])
            x = (R - state["R0"]) / state["R0"]
            T.append((s.timestamp_ms - ts0["v"]) / 1000.0)
            DR.append(x * 100.0)
            RK.append(R / 1000.0)
            if cal:
                k = poly_eval(cal["coeffs"], x)
                TIPY.append(tip_geometry(k)["tip_y"])
                state["kappa"] = k
            n += 1
        return n

    def update(_frame):
        if ingest() == 0 or not T:
            return
        t_now = T[-1]
        t_arr = np.fromiter(T, float)

        dr_line.set_data(t_arr, np.fromiter(DR, float))
        bot_line.set_data(t_arr, np.fromiter(TIPY if cal else RK, float))
        ax_dr.set_xlim(max(0.0, t_now - args.window), max(args.window, t_now))
        for ax in (ax_dr, ax_bot):
            ax.relim()
            ax.autoscale_view(scalex=False)

        if cal:
            k = state["kappa"] / 1000.0                    # 1/mm
            s = np.linspace(0.0, L_MM, 80)
            if abs(k) < 1e-7:
                xs, ys = s, np.zeros_like(s)
            else:
                xs, ys = np.sin(k * s) / k, (1.0 - np.cos(k * s)) / k
            arc_line.set_data(xs, ys)
            tip_dot.set_data([xs[-1]], [ys[-1]])
            off_line.set_data([xs[-1], xs[-1]], [0.0, ys[-1]])
            g = tip_geometry(state["kappa"])
            rad = "inf" if math.isinf(g["radius"]) else f"{g['radius']:.0f} mm"
            side = "convex" if state["kappa"] > 0 else "concave"
            if abs(state["kappa"]) < 1.0:
                side = "flat"
            info.set_text(
                f"dR/R0     {DR[-1]:+6.2f} %\n"
                f"curvature {state['kappa']:+7.1f} 1/m ({side})\n"
                f"radius    {rad}\n"
                f"angle     {g['theta_deg']:+6.1f} deg\n"
                f"tip offset{g['tip_y']:+7.1f} mm"
            )
        else:
            info.set_text(f"dR/R0 {DR[-1]:+6.2f} %\nR     {RK[-1]:.3f} kOhm")

    anim = FuncAnimation(fig, update, interval=40, cache_frame_data=False)  # noqa: F841 (keep ref)
    try:
        plt.show()
    finally:
        ser.close()


if __name__ == "__main__":
    main()