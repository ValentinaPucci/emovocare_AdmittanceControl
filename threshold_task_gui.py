#!/usr/bin/env python3
"""Reach the Line: a threshold task with a live bar (tkinter, no extra packages).

Each attempt: RELAX (sensor is zeroed) -> GO (push the bar up to the line, within --window s)
-> RESULT. Every attempt is saved to characterization/task_<time>.csv.

Place at the repo root, next to flex_triggered_motor.py.

    python threshold_task_gui.py --sim                          # no hardware: move the mouse up
    python threshold_task_gui.py --trig 20                      # real flex sensor, no motor
    python threshold_task_gui.py --trig 20 --motor --rest 8     # a success also moves the motor
    python threshold_task_gui.py --trig 20 --adapt 2            # 3 successes: +2 %, 3 failures: -2 %
"""
import argparse
import csv
import statistics
import time
import tkinter as tk
from collections import deque
from datetime import datetime
from pathlib import Path

W, H, TOP, BOT = 520, 480, 70, 420          # canvas size and the bar area
YMAX = 1.5                                   # bar scale: 0 .. YMAX * threshold


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sim", action="store_true", help="no hardware: the mouse height drives the bar")
    ap.add_argument("-p", "--port", help="flex sensor port (auto-detected if omitted)")
    ap.add_argument("--trig", type=float, default=20.0, help="threshold in percent dR/R")
    ap.add_argument("--attempts", type=int, default=10)
    ap.add_argument("--rest", type=float, default=5.0, help="seconds of relax before each attempt")
    ap.add_argument("--baseline", type=float, default=1.5, help="last seconds of the rest used as zero")
    ap.add_argument("--window", type=float, default=10.0, help="seconds allowed to reach the line")
    ap.add_argument("--smooth", type=int, default=5, help="moving-average window (samples)")
    ap.add_argument("--motor", action="store_true", help="a success moves the motor to the other end")
    ap.add_argument("--adapt", type=float, default=0.0, help="threshold step in percent (0 = fixed)")
    ap.add_argument("--out", default="characterization")
    args = ap.parse_args()

    S = dict(phase="rest", t0=time.time(), n=0, thr=abs(args.trig), sign=1, base=[], R_ref=None,
             peak=0.0, t_trig=None, win=deque(maxlen=args.smooth), mouse=0.0, rows=[],
             streak_ok=0, streak_bad=0, done=False, last_ok=False)

    # ---- signal source ----
    ftm = ser = None
    if args.sim:
        def read():
            return 10000.0 * (1 + S["sign"] * S["mouse"] / 100)
    else:
        import flex_triggered_motor as ftm
        ser = ftm.open_port(args.port)
        read = lambda: ftm.read_resistance(ser)
    if args.motor:
        if ftm is None:
            import flex_triggered_motor as ftm
        start = ftm.setup_motor()
        targets = dict(close=start - ftm.MOVE_DISTANCE, open=start)   # assumes the finger starts OPEN
        S["is_open"] = True

    Path(args.out).mkdir(parents=True, exist_ok=True)
    path = Path(args.out) / f"task_{datetime.now():%Y%m%d_%H%M%S}.csv"

    root = tk.Tk()
    root.title("Reach the line")
    cv = tk.Canvas(root, width=W, height=H, bg="white", highlightthickness=0)
    cv.pack()

    def set_phase(p):
        S["phase"], S["t0"] = p, time.time()

    def y_of(x):                                          # dR/R [%] -> canvas y
        x = max(0.0, min(x, YMAX * S["thr"]))
        return BOT - (BOT - TOP) * x / (YMAX * S["thr"])

    def finish():
        if S["done"] and path.exists():
            return
        S["done"] = True
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["attempt", "threshold_pct", "success", "time_to_trigger_s", "peak_pct", "R_ref_ohm"])
            w.writerows(S["rows"])
        print(f"Saved {path}")
        if args.motor:
            ftm.release_motor()
        if ser is not None:
            ser.close()
        root.destroy()

    def draw(x, msg, color):
        cv.delete("all")
        cv.create_text(W / 2, 28, text=msg, font=("Helvetica", 22, "bold"), fill=color)
        cv.create_text(W / 2, 54, text=f"attempt {min(S['n'] + 1, args.attempts)}/{args.attempts}   "
                       f"threshold {S['thr']:.1f} %", font=("Helvetica", 12), fill="gray30")
        cv.create_rectangle(170, TOP, 350, BOT, outline="gray60")
        if x is not None and S["phase"] == "go":
            cv.create_rectangle(170, y_of(x), 350, BOT, fill="#4a90d9", outline="")
        ty = y_of(S["thr"])
        cv.create_line(140, ty, 380, ty, fill="red", width=3)
        cv.create_text(395, ty, text="goal", fill="red", anchor="w", font=("Helvetica", 12, "bold"))

    def tick():
        if S["done"]:
            return
        el = time.time() - S["t0"]
        r = read()
        if r is not None:
            S["win"].append(r)
        R = statistics.mean(S["win"]) if S["win"] else None
        x = S["sign"] * (R - S["R_ref"]) / S["R_ref"] * 100 if (R and S["R_ref"]) else None
        ph = S["phase"]

        if ph == "rest":
            if r is not None and el >= args.rest - args.baseline:
                S["base"].append(r)
            draw(None, "RELAX", "#2a7f2a")
            if el >= args.rest and len(S["base"]) >= 5:
                S["R_ref"], S["base"] = statistics.mean(S["base"]), []
                S["win"].clear()
                S["peak"] = 0.0
                set_phase("go")

        elif ph == "go":
            if x is not None:
                S["peak"] = max(S["peak"], x)
            draw(x, "GO!", "#c0392b")
            if x is not None and x >= S["thr"]:
                S["t_trig"], S["last_ok"] = el, True
                record(True)
            elif el >= args.window:
                S["t_trig"], S["last_ok"] = None, False
                record(False)

        elif ph == "result":
            ok = S["last_ok"]
            draw(None, "SUCCESS" if ok else "TIME OUT", "#2a7f2a" if ok else "#c0392b")
            if el >= 1.5:
                if S["n"] >= args.attempts:
                    summary()
                else:
                    S["win"].clear()
                    set_phase("rest")
        elif ph == "summary":
            pass

        root.after(10, tick)

    def record(ok):
        S["n"] += 1
        S["rows"].append([S["n"], f"{S['thr']:.2f}", int(ok),
                          "" if S["t_trig"] is None else f"{S['t_trig']:.2f}",
                          f"{S['peak']:.2f}", f"{S['R_ref']:.1f}"])
        print(f"attempt {S['n']}: thr {S['thr']:.1f} %  {'OK' if ok else 'miss'}  peak {S['peak']:.1f} %")
        if args.adapt > 0:                                    # simple assist-as-needed rule
            S["streak_ok"] = S["streak_ok"] + 1 if ok else 0
            S["streak_bad"] = 0 if ok else S["streak_bad"] + 1
            if S["streak_ok"] >= 3:
                S["thr"] += args.adapt
                S["streak_ok"] = 0
            if S["streak_bad"] >= 3:
                S["thr"] = max(args.adapt, S["thr"] - args.adapt)
                S["streak_bad"] = 0
        set_phase("result")
        if ok and args.motor:                                 # complete the movement (blocks the GUI)
            draw(None, "Moving...", "gray30")
            root.update()
            target = targets["close"] if S["is_open"] else targets["open"]
            if ftm.move_to(target):
                S["is_open"] = not S["is_open"]
                S["sign"] = 1 if S["is_open"] else -1
            ser.reset_input_buffer()
            S["t0"] = time.time()

    def summary():
        n_ok = sum(int(r[2]) for r in S["rows"])
        times = [float(r[3]) for r in S["rows"] if r[3] != ""]
        med = f"{statistics.median(times):.2f} s" if times else "-"
        set_phase("summary")
        cv.delete("all")
        cv.create_text(W / 2, 150, text="Well done!", font=("Helvetica", 28, "bold"))
        cv.create_text(W / 2, 220, text=f"{n_ok} of {len(S['rows'])} attempts reached the line",
                       font=("Helvetica", 16))
        cv.create_text(W / 2, 260, text=f"median time to goal: {med}", font=("Helvetica", 16))
        cv.create_text(W / 2, 330, text="(window closes in 5 s)", font=("Helvetica", 11), fill="gray50")
        root.after(5000, finish)

    if args.sim:
        def on_move(e):                                       # mouse height -> signal
            S["mouse"] = max(0.0, (BOT - e.y) / (BOT - TOP)) * YMAX * S["thr"]
        root.bind("<Motion>", on_move)
    root.protocol("WM_DELETE_WINDOW", finish)
    root.after(10, tick)
    root.mainloop()


if __name__ == "__main__":
    main()
