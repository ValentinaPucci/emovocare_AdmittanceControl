#!/usr/bin/env python3
"""Flex sensor shape estimation (95 mm SpectraFlex, 10 kOhm divider, rest = straight).

Place in scripts/ next to read_serial.py.

  flat      : measure and save the flat-sensor resistance R0
  calibrate : bend the sensor on known radii, fit  curvature = f(dR/R0)  -> JSON
  live      : show dR/R0, curvature, radius, bend angle and tip offset,
              optionally with a rate trigger on dR/R0

Model: constant-curvature arc, base clamped, tangent along x.
  theta = kappa * L,  tip_x = sin(theta)/kappa,  tip_y = (1 - cos(theta))/kappa
Sign convention: kappa > 0 = convex (the direction that RAISES resistance),
                 kappa < 0 = concave.

Trigger: fires when d(dR/R0)/dt, in percent per second, exceeds --trig-rate
in the chosen direction. It only prints a message for now.

Examples:
    python scripts/flex_shape.py flat                      # once, sensor flat
    python scripts/flex_shape.py live --saved-flat         # starts bent, uses saved R0
    python scripts/flex_shape.py live --saved-flat --trig 50
    python scripts/flex_plot.py --saved-flat
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import sys
import time
from collections import deque
from pathlib import Path

import numpy as np
import serial

from read_serial import DEFAULT_BAUD, find_nano_port, parse_sample

L_MM = 95.0
R_FIXED_OHM = 10_000.0  # informational; resistance is already computed by the firmware
CAL_PATH = Path("recordings/flex_cal.json")
SETTLE_S = 5.0
HOLD_S = 10.0
ZERO_S = 3.0
DEFAULT_GAIN = 0.075   # dR/R0 per (1/m); from ring closure (360 deg at dR/R0 ~ 5)
FLAT_PATH = Path("recordings/flex_flat.json")


# ----------------------------------------------------------------- serial helpers
def open_port(port: str | None) -> serial.Serial:
    port = port or find_nano_port()
    if port is None:
        sys.exit("No Nano 33 IoT found. Use --port.")
    ser = serial.Serial(port, DEFAULT_BAUD, timeout=1.0)
    ser.dtr = True
    time.sleep(0.1)
    ser.reset_input_buffer()
    return ser


def read_resistance(ser: serial.Serial) -> float | None:
    s = parse_sample(ser.readline().decode("utf-8", errors="replace").strip())
    if s is None or not math.isfinite(s.resistance_ohm):
        return None
    return s.resistance_ohm


def capture(ser: serial.Serial, seconds: float) -> list[float]:
    end, vals = time.time() + seconds, []
    while time.time() < end:
        r = read_resistance(ser)
        if r is not None:
            vals.append(r)
    return vals


def measure_rest(ser: serial.Serial) -> float:
    ser.reset_input_buffer()
    vals = capture(ser, ZERO_S)
    if len(vals) < 10:
        sys.exit("Not enough samples while zeroing; check the connection.")
    return statistics.mean(vals)


def resolve_r0(ser, cal, r0=None, start_angle=None, saved_flat=False) -> float:
    """Flat-sensor resistance. Order: --r0, --start-angle, --saved-flat, zero now."""
    if r0:
        return r0
    if start_angle is not None:
        if cal is None:
            sys.exit("--start-angle needs a gain or calibration.")
        R = measure_rest(ser)                                   # current (bent) resistance
        kappa0 = math.radians(start_angle) / (L_MM / 1000.0)    # 1/m, + = convex
        x0 = kappa0 / cal["coeffs"][0]                          # linear term only
        return R / (1.0 + x0)
    if saved_flat:
        if not FLAT_PATH.exists():
            sys.exit(f"{FLAT_PATH} not found. Run 'flat' with the sensor flat first.")
        return json.loads(FLAT_PATH.read_text())["R0_ohm"]
    return measure_rest(ser)


def cmd_flat(args) -> None:
    ser = open_port(args.port)
    print("Keep the sensor FLAT and still...")
    R0 = measure_rest(ser)
    FLAT_PATH.parent.mkdir(parents=True, exist_ok=True)
    FLAT_PATH.write_text(json.dumps({"R0_ohm": R0}))
    print(f"Saved flat R0 = {R0/1000:.3f} kOhm to {FLAT_PATH}")
    ser.close()


# ----------------------------------------------------------------- model
def poly_eval(coeffs: list[float], x: float) -> float:
    """kappa(x) = c1*x + c2*x^2 + ...  (no constant term: flat -> kappa = 0)."""
    return sum(c * x ** (i + 1) for i, c in enumerate(coeffs))


def tip_geometry(kappa_per_m: float, length_mm: float = L_MM) -> dict:
    k = kappa_per_m / 1000.0  # 1/mm
    theta = k * length_mm
    if abs(k) < 1e-9:
        return dict(theta_deg=0.0, tip_x=length_mm, tip_y=0.0, chord=length_mm, radius=math.inf)
    return dict(
        theta_deg=math.degrees(theta),
        tip_x=math.sin(theta) / k,
        tip_y=(1.0 - math.cos(theta)) / k,       # signed lateral tip offset
        chord=2.0 * math.sin(theta / 2.0) / k,
        radius=1.0 / abs(k),
    )

class DeltaTrigger:
    """Fires when dR/R0 (in %) moves DELTA away from its recent low point.

    delta > 0: fires on a rise (convex). delta < 0: fires on a fall (concave).
    The reference follows the signal the other way, so slow drift never fires,
    and it resets to the current value after each trigger.
    """
    def __init__(self, delta_pct: float):
        self.sign = 1.0 if delta_pct > 0 else -1.0
        self.th = abs(delta_pct)
        self.ref: float | None = None

    def update(self, x_pct: float) -> bool:
        v = self.sign * x_pct
        if self.ref is None or v < self.ref:
            self.ref = v
        if v - self.ref >= self.th:
            self.ref = v
            return True
        return False
def load_model(gain: float | None):
    """Priority: --gain, then calibration file, then DEFAULT_GAIN."""
    if gain:
        return {"coeffs": [1.0 / gain], "rms_error_per_m": float("nan")}, f"gain {gain} (--gain)"
    if CAL_PATH.exists():
        return json.loads(CAL_PATH.read_text()), f"calibration {CAL_PATH}"
    return ({"coeffs": [1.0 / DEFAULT_GAIN], "rms_error_per_m": float("nan")},
            f"default gain {DEFAULT_GAIN} (ring-closure estimate)")


# ----------------------------------------------------------------- calibrate
def cmd_calibrate(args) -> None:
    ser = open_port(args.port)
    print("Keep the sensor FLAT and still for zeroing...")
    R0 = measure_rest(ser)
    print(f"Rest resistance R0 = {R0/1000:.3f} kOhm")
    print(
        "\nEnter: <radius_mm> <convex|concave>   e.g. '50 convex'. 'z' = re-zero at rest, "
        "'f' = fit and save, 'q' = quit without saving."
    )
    print(f"Sensor must be wrapped on a known-radius mandrel; settle {SETTLE_S:.0f}s then average {HOLD_S-SETTLE_S:.0f}s.")

    points: list[dict] = []
    while True:
        line = input("> ").strip().lower()
        if line == "q":
            return
        if line == "z":
            R0 = measure_rest(ser)
            print(f"R0 = {R0/1000:.3f} kOhm")
            continue
        if line == "f":
            break
        try:
            r_txt, direction = line.split()
            radius = float(r_txt)
            sign = {"convex": 1.0, "concave": -1.0}[direction]
        except (ValueError, KeyError):
            print("format: <radius_mm> <convex|concave>")
            continue
        ser.reset_input_buffer()
        capture(ser, SETTLE_S)
        vals = capture(ser, HOLD_S - SETTLE_S)
        if len(vals) < 10:
            print("too few samples, retry")
            continue
        m, sd = statistics.mean(vals), statistics.stdev(vals)
        x = (m - R0) / R0
        kappa = sign * 1000.0 / radius
        points.append(dict(radius_mm=radius, direction=direction, kappa=kappa, x=x, mean_ohm=m, std_ohm=sd))
        print(f"  R = {m/1000:.3f} kOhm  dR/R0 = {x:+.4f}  kappa = {kappa:+.1f} 1/m")

    if len(points) < args.degree:
        sys.exit(f"Need at least {args.degree} calibration points for degree {args.degree}.")

    xs = np.array([p["x"] for p in points] + [0.0])      # rest point anchors the fit
    ks = np.array([p["kappa"] for p in points] + [0.0])
    A = np.vstack([xs ** (i + 1) for i in range(args.degree)]).T
    coeffs, *_ = np.linalg.lstsq(A, ks, rcond=None)
    pred = A @ coeffs
    rms = float(np.sqrt(np.mean((pred - ks) ** 2)))
    print(f"\nFit: kappa = " + " + ".join(f"{c:.4g}*x^{i+1}" for i, c in enumerate(coeffs)))
    print(f"RMS curvature error = {rms:.2f} 1/m")

    CAL_PATH.parent.mkdir(parents=True, exist_ok=True)
    CAL_PATH.write_text(json.dumps(dict(
        length_mm=L_MM, degree=args.degree, coeffs=coeffs.tolist(),
        rms_error_per_m=rms, points=points,
    ), indent=2))
    print(f"Saved {CAL_PATH}")


# ----------------------------------------------------------------- live
def cmd_live(args) -> None:
    cal, src = load_model(args.gain)
    print(f"Using {src}")

    ser = open_port(args.port)
    R0 = resolve_r0(ser, cal, args.r0, args.start_angle, args.saved_flat)
    print(f"Flat R0 = {R0/1000:.3f} kOhm. Ctrl+C to stop.\n")

    fh = writer = None
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        fh = args.out.open("a", newline="", encoding="utf-8")
        writer = csv.writer(fh)
        if fh.tell() == 0:
            writer.writerow(["t_s", "R_ohm", "dR_over_R0", "kappa_per_m", "radius_mm",
                             "theta_deg", "tip_x_mm", "tip_y_mm"])

    win: deque[float] = deque(maxlen=args.smooth)
    t0, last_print = time.time(), 0.0
    trig = DeltaTrigger(args.trig) if args.trig else None
    try:
        while True:
            r = read_resistance(ser)
            if r is None:
                continue
            win.append(r)
            R = statistics.mean(win)
            x = (R - R0) / R0
            t = time.time() - t0

            if cal:
                kappa = poly_eval(cal["coeffs"], x)
                g = tip_geometry(kappa)
                row = [f"{t:.3f}", f"{R:.1f}", f"{x:.5f}", f"{kappa:.2f}",
                       "inf" if math.isinf(g["radius"]) else f"{g['radius']:.1f}",
                       f"{g['theta_deg']:.1f}", f"{g['tip_x']:.2f}", f"{g['tip_y']:.2f}"]
            else:
                row = [f"{t:.3f}", f"{R:.1f}", f"{x:.5f}", "", "", "", "", ""]
            if writer:
                writer.writerow(row)

            if trig and trig.update(x * 100.0):
                print(f"\n>>> MOTOR WOULD TRIGGER  t={t:.2f}s  dR/R0={x*100:+.2f}%")

            if t - last_print >= 0.1:
                last_print = t
                if cal:
                    rad = "   inf" if math.isinf(g["radius"]) else f"{g['radius']:6.0f}"
                    side = "convex " if kappa > 0 else "concave"
                    print(f"\rR={R/1000:7.3f} kOhm  dR/R0={x*100:+6.2f}%  "
                        f"{side} r={rad} mm  theta={g['theta_deg']:+6.1f} deg  "
                        f"tip offset={g['tip_y']:+6.1f} mm (x={g['tip_x']:5.1f})   ",
                        end="", flush=True)
                else:
                    print(f"\rR={R/1000:7.3f} kOhm  dR/R0={x*100:+6.2f}%",
                          end="", flush=True)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        if fh:
            fh.close()
        ser.close()


# ----------------------------------------------------------------- main
def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", "--port")
    sub = ap.add_subparsers(dest="cmd", required=True)

    c = sub.add_parser("calibrate", help="fit curvature vs dR/R0 using known radii")
    c.add_argument("--degree", type=int, default=2, help="polynomial degree (default 2)")
    c.set_defaults(fn=cmd_calibrate)

    l = sub.add_parser("live", help="live shape estimate")
    l.add_argument("-o", "--out", type=Path, help="append results to CSV")
    l.add_argument("--smooth", type=int, default=5, help="moving-average window in samples")
    l.add_argument("--gain", type=float, help="linear gain, dR/R0 per (1/m); overrides everything")
    l.add_argument("--r0", type=float, help="flat resistance in ohms (skips zeroing)")
    l.add_argument("--start-angle", type=float,
                   help="bend angle in degrees at startup, + = convex (no zeroing)")
    l.add_argument("--saved-flat", action="store_true", help="use R0 saved by the 'flat' command")
    l.add_argument("--trig", type=float,
                   help="trigger when dR/R0 moves this many percent from its recent low "
                        "(positive = rising/convex, negative = falling/concave)")
    l.set_defaults(fn=cmd_live)
    f = sub.add_parser("flat", help="measure and save flat-sensor resistance")
    f.set_defaults(fn=cmd_flat)

    args = ap.parse_args()
    args.fn(args)


if __name__ == "__main__":
    main()