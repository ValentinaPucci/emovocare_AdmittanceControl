#!/usr/bin/env python3
"""Reach the Line: a threshold task with a live bar (tkinter, no extra packages).

Each attempt: RELAX (sensor is zeroed) -> GO (push the bar up to the line, within --window s)
-> RESULT. Every attempt is saved to characterization/task_<time>.csv.

With --motor, a success moves the finger to the other end. The next attempt then goes the
OTHER way (open -> close needs dR/R to rise, closed -> open needs it to fall), exactly like
flex_triggered_motor.py. Tell the script where the finger starts with --start.

Place at the repo root, next to flex_triggered_motor.py.

    python threshold_task_gui.py --sim                                    # no hardware: move the mouse up
    python threshold_task_gui.py --trig 20                                # real flex sensor, no motor
    python threshold_task_gui.py --trig 20 --motor --start open --rest 5  # a success also moves the motor
    python threshold_task_gui.py --trig 20 --motor --start closed         # finger starts CLOSED
    python threshold_task_gui.py --trig 20 --adapt 2                      # 3 successes: +2 %, 3 failures: -2 %
"""
import argparse
import atexit
import csv
import os
import signal
import sys
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
    ap.add_argument("--start", choices=["open", "closed"], default="open",
                    help="is the finger OPEN or CLOSED when the script starts? (used with --motor)")
    ap.add_argument("--adapt", type=float, default=0.0, help="threshold step in percent (0 = fixed)")
    ap.add_argument("--out", default="characterization")
    args = ap.parse_args()

    S = dict(phase="rest", t0=time.time(), n=0, thr=abs(args.trig), sign=1, base=[], R_ref=None,
             peak=0.0, t_trig=None, win=deque(maxlen=args.smooth), mouse=0.0, rows=[],
             streak_ok=0, streak_bad=0, done=False, last_ok=False,
             is_open=(args.start == "open"), extra_rest=0.0,
             motor_on=False, released=False, finished=False, stop=False)

    # ---- signal source ----
    ftm = ser = None

    def release_all():
        """Torque OFF + close the sensor port. Safe to call many times."""
        if S["released"]:
            return
        S["released"] = True
        if S["motor_on"]:
            off = False
            for attempt in range(1, 4):
                try:
                    ftm.ph.is_using = False                   # clear a lock left by an interrupted transaction
                    ftm.ph.clearPort()                        # drop half-received bytes
                    ftm.write(ftm.ADDR_TORQUE_ENABLE, 0, 1)
                    data = ftm.read_bytes(ftm.ADDR_TORQUE_ENABLE, 1)
                    if data is not None and data[0] == 0:     # read it back to be sure
                        off = True
                        break
                    print(f"Torque release attempt {attempt}: not confirmed, retrying...")
                    ftm.ph.closePort()                        # reopen the port and try again
                    ftm.ph.openPort()
                    ftm.ph.setBaudRate(ftm.BAUD)
                except Exception as e:
                    print(f"Torque release attempt {attempt} error: {e}")
            if not off:                                       # last resort: a reboot always clears torque
                try:
                    ftm.pk.reboot(ftm.ph, ftm.MOTOR_ID)
                    print("Motor rebooted to clear torque.")
                    off = True
                except Exception as e:
                    print(f"Reboot failed: {e}")
            try:
                ftm.ph.closePort()
            except Exception:
                pass
            print("Motor torque OFF (confirmed)." if off else
                  "!!! COULD NOT TURN TORQUE OFF - power-cycle the motor !!!")
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass

    atexit.register(release_all)                              # any exit path: crash, sys.exit, Ctrl+C
    signal.signal(signal.SIGTERM, lambda *a: sys.exit(0))     # kill / terminal closed -> still releases

    def on_sigint(sig, frm):
        # Ctrl+C only sets a flag; the GUI loop / motor wait sees it within ~20 ms and shuts down cleanly.
        # (Raising KeyboardInterrupt inside Tk callbacks killed the process before torque was released.)
        if S["stop"]:                                         # second Ctrl+C: force it
            release_all()
            os._exit(1)
        S["stop"] = True
        print("\nStopping (Ctrl+C)...")

    def install_signals():
        for sig, h in ((signal.SIGINT, on_sigint), (signal.SIGHUP, lambda *a: sys.exit(0))):
            signal.signal(sig, h)
    install_signals()
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
        S["motor_on"] = True                                  # from here on, release on exit
        start = ftm.setup_motor()
        # the start position is one end; the other end is MOVE_DISTANCE ticks away (closing = ticks decrease)
        if S["is_open"]:
            open_pos, closed_pos = start, start - ftm.MOVE_DISTANCE
        else:
            open_pos, closed_pos = start + ftm.MOVE_DISTANCE, start
        print(f"Starting {args.start.upper()}: open = {open_pos}, closed = {closed_pos}")
    # open -> needs dR/R to RISE (+1); closed -> needs it to FALL (-1)
    S["sign"] = 1 if S["is_open"] else -1

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

    def go_text():
        if not args.motor:
            return "GO!"
        return "GO!  bend to CLOSE" if S["is_open"] else "GO!  unbend to OPEN"

    def finish():
        if S["finished"]:
            return
        S["finished"] = True
        S["done"] = True
        release_all()                                         # torque OFF first, before anything else
        if S["rows"]:
            try:
                with open(path, "w", newline="") as f:
                    w = csv.writer(f)
                    w.writerow(["attempt", "threshold_pct", "success", "time_to_trigger_s", "peak_pct", "R_ref_ohm"])
                    w.writerows(S["rows"])
                print(f"Saved {path}")
            except Exception as e:
                print(f"Could not save {path}: {e}")
        try:
            root.destroy()
        except Exception:
            pass

    def on_error(exc, val, tb):                               # tkinter swallows callback errors (even Ctrl+C)
        if exc is not KeyboardInterrupt:
            import traceback
            traceback.print_exception(exc, val, tb)
        print("\nStopped." if exc is KeyboardInterrupt else "Error -> stopping.")
        finish()
    root.report_callback_exception = on_error

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
        if S["stop"] and not S["finished"]:
            finish()
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
            rest_total = args.rest + S["extra_rest"]       # longer after a motor move (sensor settles)
            if r is not None and el >= rest_total - args.baseline:
                S["base"].append(r)
            draw(None, "RELAX - keep still", "#2a7f2a")
            if el >= rest_total and len(S["base"]) >= 5:
                S["R_ref"], S["base"] = statistics.mean(S["base"]), []
                S["win"].clear()
                S["peak"] = 0.0
                S["extra_rest"] = 0.0
                set_phase("go")

        elif ph == "go":
            if x is not None:
                S["peak"] = max(S["peak"], x)
            draw(x, go_text(), "#c0392b")
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
                    S["base"] = []
                    set_phase("rest")
        elif ph == "summary":
            pass

        if not S["done"]:
            root.after(10, tick)

    def move_motor(target):
        """Same logic as ftm.move_to, but keeps the window alive so Esc / close / Ctrl+C work mid-move."""
        if not ftm.set_goal(target):
            return False
        t0 = time.time()
        while time.time() - t0 < ftm.MOVE_TIMEOUT:
            try:
                root.update()
            except tk.TclError:
                return False
            if S["finished"] or S["stop"]:
                return False
            pos = ftm.read_position()
            if pos is not None and abs(pos - target) <= ftm.TOLERANCE:
                print("  target reached")
                return True
            time.sleep(0.02)
        pos = ftm.read_position()
        print(f"  stopped at {pos}, target {target}")
        if pos is None:
            return False
        ftm.set_goal(pos)                                     # hold here instead of pulling
        remaining = abs(pos - target)
        if remaining <= (1.0 - ftm.DONE_FRACTION) * ftm.MOVE_DISTANCE:
            print(f"  {remaining} ticks short but close enough -> counted as done")
            return True
        print(f"  target NOT reached ({remaining} ticks short) -> holding current position")
        return False

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
            target = closed_pos if S["is_open"] else open_pos
            print(f"  {'closing' if S['is_open'] else 'opening'} -> {target}")
            if move_motor(target):
                S["is_open"] = not S["is_open"]
                S["sign"] = 1 if S["is_open"] else -1         # next attempt goes the other way
                S["extra_rest"] = ftm.SETTLE_S                # let the sensor settle before re-zeroing
                print(f"  now {'OPEN' if S['is_open'] else 'CLOSED'}")
            else:
                print("  move not completed -> direction unchanged")
            if S["finished"] or S["stop"]:
                return
            if ser is not None:
                ser.reset_input_buffer()                      # drop samples buffered during the move
            S["win"].clear()
            S["base"] = []
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
    root.protocol("WM_DELETE_WINDOW", finish)                 # closing the window
    root.bind("<Escape>", lambda e: finish())                 # Esc = stop now
    root.after(10, tick)
    # Imported modules (flex_shape, dynamixel_sdk, ...) may have replaced the Ctrl+C handler: put ours back
    # right before the loop, and say so if it had been changed.
    if signal.getsignal(signal.SIGINT) is not on_sigint:
        print(f"(note: SIGINT handler had been changed to {signal.getsignal(signal.SIGINT)}; restored)")
    install_signals()
    print("Running. Ctrl+C or Esc stops and releases the motor torque.")
    try:
        root.mainloop()
    except KeyboardInterrupt:                                 # Ctrl+C in the terminal
        print("\nStopped.")
    finally:
        finish()                                              # torque OFF, always


if __name__ == "__main__":
    main()