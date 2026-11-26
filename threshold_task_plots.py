#!/usr/bin/env python3
"""Reach the Line + live dashboard (tkinter only, no extra packages).

Same task as threshold_task_gui.py:  RELAX (sensor zeroed) -> GO (push the bar to the line, within --window s)
-> RESULT.  With --motor a success moves the finger to the other end and the next attempt goes the other way.

NEW: a live dashboard next to the task
  * dR/R0 [%] of the flex sensor with the goal line (signed: +thr when closing, -thr when opening)
  * raw + smoothed sensor resistance, with the zero reference R0
  * motor position, goal position, profile trajectory, open/closed ends
  * motor velocity [rpm], PWM [% of max], current [mA]  (the "effort" of the motor)
  * per-attempt performance: peak vs goal, time to goal, success rate, noise floor, max rate of rise...
  * coloured background = phase (green relax, red go, grey result, yellow motor move)

Everything is also written to disk:
  characterization/task_<time>.csv        one row per attempt (old columns first, new ones appended)
  characterization/telemetry_<time>.csv   continuous log of sensor + motor (for MATLAB / pandas)

Place at the repo root, next to flex_triggered_motor.py.

    python threshold_task_plots.py --sim                                    # no hardware: move the mouse over the task bar
    python threshold_task_plots.py --trig 20                                # real flex sensor, no motor
    python threshold_task_plots.py --trig 20 --motor --start open --rest 5  # a success also moves the motor
    python threshold_task_plots.py --trig 20 --motor --start closed
    python threshold_task_plots.py --trig 20 --adapt 2 --span 30            # adaptive threshold, 30 s plot window

Motor telemetry is read from the Dynamixel X-series control table (XL330 / XL430 / XM430, protocol 2.0).
If your motor is another model, change MOT_ADDR / MOT_LEN / parse_motor() and the unit options
(--pwm-max, --cur-unit).
"""
import argparse
import atexit
import csv
import os
import signal
import sys
import statistics
import threading
import time
import tkinter as tk
from collections import deque
from datetime import datetime
from pathlib import Path

W, H, TOP, BOT = 520, 480, 70, 420          # task canvas size and the bar area
YMAX = 1.5                                   # bar scale: 0 .. YMAX * threshold
PERF_H = 190                                 # performance panel height (under the task)
PW, PH = 820, H + PERF_H                     # plots canvas size (2 columns x 3 rows)

# Dynamixel X-series control table: one block read gives everything (protocol 2.0)
#   offset 0  Goal Position (4)      116        offset 16 Present Position (4)   132
#   offset 6  Moving (1)             122        offset 20 Velocity Trajectory(4) 136
#   offset 8  Present PWM (2)        124        offset 24 Position Trajectory(4) 140
#   offset 10 Present Current (2)    126        offset 28 Input Voltage (2)      144
#   offset 12 Present Velocity (4)   128        offset 30 Temperature (1)        146
MOT_ADDR, MOT_LEN = 116, 32
RPM_PER_UNIT = 0.229                         # velocity unit of X-series motors

PH_COL = {"rest": "#eaf6ea", "go": "#fdecec", "result": "#f1f1f1", "move": "#fff2c9"}
C_BLUE, C_RED, C_PURPLE, C_ORANGE, C_GREEN = "#1f6fd1", "#d62728", "#7b3fb5", "#e67e22", "#2a9d3a"


def parse_motor(data):
    """Block read (MOT_LEN bytes from MOT_ADDR) -> dict of raw values, or None."""
    b = bytes(data)
    if len(b) < MOT_LEN:
        return None

    def i(o, n, signed=True):
        return int.from_bytes(b[o:o + n], "little", signed=signed)

    return dict(goal=i(0, 4), moving=b[6], pwm=i(8, 2), cur=i(10, 2), vel=i(12, 4),
                pos=i(16, 4), ptraj=i(24, 4), volt=i(28, 2, False) * 0.1, temp=b[30])


def visible(buf, tmin):
    """Samples of a deque with t >= tmin (walks from the newest backwards: cheap)."""
    out = []
    for s in reversed(buf):
        if s[0] < tmin:
            break
        out.append(s)
    out.reverse()
    return out


def draw_plot(c, cell, title, unit, series, tnow, span, hlines=(), bands=(), minspan=1.0,
              fmt="{:.1f}", label_idx=0, empty="no data"):
    """One strip chart on canvas c.
    series: [(points[(t, v|None)], color, width), ...]   hlines: [(value, color, dash, label), ...]
    bands:  [(t, phase), ...] -> coloured background by phase."""
    x0, y0, x1, y1 = cell
    px0, py0, px1, py1 = x0 + 56, y0 + 24, x1 - 10, y1 - 20
    tmin = tnow - span

    def X(t):
        return px0 + (t - tmin) / span * (px1 - px0)

    c.create_text(x0 + 8, y0 + 11, text=title, anchor="w", font=("Helvetica", 10, "bold"), fill="gray20")
    c.create_rectangle(px0, py0, px1, py1, outline="gray65", fill="white")

    def band(a, b, ph):
        xa, xb = X(max(a, tmin)), X(b)
        if xb - xa >= 1:
            c.create_rectangle(xa, py0, xb, py1, fill=PH_COL.get(ph, "white"), outline="")

    start = cph = None
    for t, ph in bands:                                       # contiguous phase segments
        if cph is None:
            start, cph = t, ph
        elif ph != cph:
            band(start, t, cph)
            start, cph = t, ph
    if cph is not None:
        band(start, bands[-1][0], cph)

    vals = [v for pts, _, _ in series for _, v in pts if v is not None] + [h[0] for h in hlines]
    if not vals:
        c.create_text((px0 + px1) / 2, (py0 + py1) / 2, text=empty, fill="gray55", font=("Helvetica", 10))
        return
    lo, hi = min(vals), max(vals)
    if hi - lo < minspan:
        mid = (hi + lo) / 2
        lo, hi = mid - minspan / 2, mid + minspan / 2
    pad = (hi - lo) * 0.08
    lo, hi = lo - pad, hi + pad

    def Y(v):
        return py1 - (v - lo) / (hi - lo) * (py1 - py0)

    for k in range(5):                                        # grid + y labels
        v = lo + (hi - lo) * k / 4
        y = Y(v)
        c.create_line(px0, y, px1, y, fill="#dedede")
        c.create_text(px0 - 4, y, text=f"{v:.5g}", anchor="e", font=("Helvetica", 8), fill="gray40")
    for v, col, dash, lab in hlines:
        y = Y(v)
        c.create_line(px0, y, px1, y, fill=col, dash=dash, width=1)
        if lab:
            c.create_text(px1 - 3, y - 2, text=lab, anchor="se", font=("Helvetica", 8, "bold"), fill=col)

    for pts, col, w in series:                                # polylines, broken at None
        seg = []
        for t, v in pts + [(0, None)]:
            if v is None:
                if len(seg) >= 2:
                    if len(seg) > 800:
                        seg = seg[::len(seg) // 800 + 1] + [seg[-1]]
                    c.create_line(*[co for p in seg for co in p], fill=col, width=w)
                seg = []
            else:
                seg.append((X(t), Y(v)))

    c.create_text(px0, y1 - 8, text=f"-{span:.0f} s", anchor="w", font=("Helvetica", 8), fill="gray45")
    c.create_text(px1, y1 - 8, text="now", anchor="e", font=("Helvetica", 8), fill="gray45")
    if series and series[label_idx][0] and series[label_idx][0][-1][1] is not None:
        c.create_text(x1 - 10, y0 + 11, text=f"{unit}   {fmt.format(series[label_idx][0][-1][1])}",
                      anchor="e", font=("Helvetica", 10, "bold"), fill=series[label_idx][1])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sim", action="store_true", help="no hardware: the mouse height over the task bar drives the signal")
    ap.add_argument("-p", "--port", help="flex sensor port (auto-detected if omitted)")
    ap.add_argument("--trig", type=float, default=10.0, help="threshold in percent dR/R")
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
    # dashboard options
    ap.add_argument("--span", type=float, default=20.0, help="seconds shown in the live plots")
    ap.add_argument("--fps", type=float, default=10.0, help="plot refresh rate (Hz)")
    ap.add_argument("--motor-hz", type=float, default=20.0, help="max motor telemetry polling rate (Hz)")
    ap.add_argument("--pwm-max", type=float, default=885.0, help="raw PWM value = 100 %% (X-series: 885)")
    ap.add_argument("--cur-unit", type=float, default=1.0,
                    help="mA per raw current unit (XL330: 1.0, XL430/XM430: 2.69)")
    ap.add_argument("--no-log", action="store_true", help="do not write the continuous telemetry csv")
    args = ap.parse_args()

    T0 = time.time()

    def now():
        return time.time() - T0

    S = dict(phase="wait", t0=time.time(), n=0, thr=abs(args.trig), sign=1, base=[], R_ref=None,
             peak=0.0, t_trig=None, win=deque(maxlen=args.smooth), mouse=0.0, rows=[],
             streak_ok=0, streak_bad=0, done=False, last_ok=False,
             is_open=(args.start == "open"), extra_rest=0.0,
             motor_on=False, released=False, finished=False, stop=False,
             # dashboard state
             rd_stop=False, rd_thread=None, hit=None, x_last=None,
             moving=False, mot=None, mot_fail=0, noise=0.0, rate_max=0.0, xh=deque(),
             mv_peak_cur=0.0, mv_peak_pwm=0.0,
             t_mot=0.0, t_plot=0.0, t_flush=time.time(), t_rate=time.time(),
             cnt_s=0, cnt_m=0, rate_s=0.0, rate_m=0.0, tele=None, tw=None)

    SENS = deque()      # (t, R_raw, R_smooth, dRR_pct|None, goal_pct|None, phase)
    MOT = deque()       # (t, pos, goal, pos_traj, vel_rpm, pwm_pct, cur_mA, phase)
    Q = deque(maxlen=5000)   # (time.time(), R_ohm) filled by the sensor reader thread
    open_pos = closed_pos = None

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
        S["rd_stop"] = True                                    # stop the sensor reader thread first
        if S["rd_thread"] is not None:
            S["rd_thread"].join(timeout=0.5)
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass

    atexit.register(release_all)                              # any exit path: crash, sys.exit, Ctrl+C
    signal.signal(signal.SIGTERM, lambda *a: sys.exit(0))     # kill / terminal closed -> still releases

    def on_sigint(sig, frm):
        # Ctrl+C only sets a flag; the GUI loop / motor wait sees it within ~20 ms and shuts down cleanly.
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

        def reader():
            """Reads the flex sensor continuously, independent of the GUI and the motor."""
            while not S["rd_stop"]:
                try:
                    r = ftm.read_resistance(ser)
                except Exception:
                    if S["rd_stop"]:
                        break
                    time.sleep(0.05)
                    continue
                if r is None:
                    time.sleep(0.001)
                else:
                    Q.append((time.time(), r))
        S["rd_thread"] = threading.Thread(target=reader, daemon=True)
        S["rd_thread"].start()
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
    stamp = f"{datetime.now():%Y%m%d_%H%M%S}"
    path = Path(args.out) / f"task_{stamp}.csv"
    if not args.no_log:
        tpath = Path(args.out) / f"telemetry_{stamp}.csv"
        S["tele"] = open(tpath, "w", newline="")
        S["tw"] = csv.writer(S["tele"])
        S["tw"].writerow(["t_s", "attempt", "phase", "R_ohm", "R_smooth_ohm", "R0_ohm", "dRR_pct", "goal_pct",
                          "motor_pos", "motor_goal_pos", "motor_pos_traj", "motor_vel_rpm", "motor_pwm_pct",
                          "motor_cur_mA", "motor_volt_V", "motor_temp_C", "motor_moving"])
        print(f"Logging telemetry to {tpath}")

    root = tk.Tk()
    root.title("Reach the line - live dashboard")
    left = tk.Frame(root)
    left.grid(row=0, column=0, sticky="n")
    cv = tk.Canvas(left, width=W, height=H, bg="white", highlightthickness=0)
    cv.pack()
    btn = tk.Button(left, text="START  (or press Space / Enter)", font=("Helvetica", 14, "bold"),
                    bg="#2a9d3a", fg="white", activebackground="#238a31", activeforeground="white",
                    command=lambda: start_task())
    btn.pack(fill="x", pady=2)
    pv = tk.Canvas(left, width=W, height=PERF_H, bg="#fafafa", highlightthickness=0)
    pv.pack()
    pcv = tk.Canvas(root, width=PW, height=PH, bg="#fafafa", highlightthickness=0)
    pcv.grid(row=0, column=1, sticky="n")

    def set_phase(p):
        S["phase"], S["t0"] = p, time.time()

    def start_task():
        """START button: leave the waiting screen and begin attempt 1 with a fresh relax period."""
        if S["phase"] != "wait" or S["finished"]:
            return
        S["win"].clear()
        S["base"] = []
        set_phase("rest")
        btn.config(state="disabled", text="running...  (Esc = stop)", bg="gray55")

    def y_of(x):                                          # dR/R [%] -> canvas y
        x = max(0.0, min(x, YMAX * S["thr"]))
        return BOT - (BOT - TOP) * x / (YMAX * S["thr"])

    def go_text():
        if not args.motor:
            return "GO!"
        return "GO!  bend to CLOSE" if S["is_open"] else "GO!  unbend to OPEN"

    # ------------------------------------------------------------------ logging / telemetry
    def log_row(r=None, R=None, dRR=None, goal=None, t=None):
        if S["tw"] is None:
            return
        m = S["mot"] or {}

        def fm(v, p=3):
            return "" if v is None else f"{v:.{p}f}"
        ph = "move" if S["moving"] else S["phase"]
        try:
            S["tw"].writerow([f"{(now() if t is None else t):.3f}", min(S["n"] + 1, args.attempts), ph, fm(r, 1), fm(R, 1),
                              fm(S["R_ref"], 1), fm(dRR, 3), fm(goal, 2),
                              m.get("pos", ""), m.get("goal", ""), m.get("ptraj", ""), fm(m.get("vel"), 2),
                              fm(m.get("pwm"), 2), fm(m.get("cur"), 1), fm(m.get("volt"), 1),
                              m.get("temp", ""), m.get("moving", "")])
        except ValueError:                                    # file already closed
            S["tw"] = None

    def sample_motor():
        """One block read of the motor. Returns True if new data."""
        try:
            data = ftm.read_bytes(MOT_ADDR, MOT_LEN)
            m = parse_motor(data) if data is not None else None
        except Exception:
            m = None
        if m is None:
            S["mot_fail"] += 1
            if S["mot_fail"] == 20:
                print("(warning: motor telemetry reads keep failing - check MOT_ADDR / MOT_LEN for your motor model)")
            return False
        m["pwm"] = m["pwm"] * 100.0 / args.pwm_max           # raw -> % of max PWM
        m["cur"] = m["cur"] * args.cur_unit                  # raw -> mA
        m["vel"] = m["vel"] * RPM_PER_UNIT                   # raw -> rpm
        S["mot"] = m
        S["cnt_m"] += 1
        t = now()
        MOT.append((t, m["pos"], m["goal"], m["ptraj"], m["vel"], m["pwm"], m["cur"],
                    "move" if S["moving"] else S["phase"]))
        while MOT and MOT[0][0] < t - args.span - 5:
            MOT.popleft()
        if S["moving"]:
            S["mv_peak_cur"] = max(S["mv_peak_cur"], abs(m["cur"]))
            S["mv_peak_pwm"] = max(S["mv_peak_pwm"], abs(m["pwm"]))
        return True

    # ------------------------------------------------------------------ drawing: dashboard
    def redraw():
        tn = now()
        try:
            sv = visible(SENS, tn - args.span)
            mv = visible(MOT, tn - args.span)
            pcv.delete("all")
            cw, ch = PW / 2, PH / 3

            def cell(r, c_):
                return (c_ * cw, r * ch, (c_ + 1) * cw, (r + 1) * ch)
            sb = [(s[0], s[5]) for s in sv]
            mb = [(m[0], m[7]) for m in mv]
            gdir = "rise" if S["sign"] == 1 else "fall"

            draw_plot(pcv, cell(0, 0), f"dR/R0  (goal: signal must {gdir})", "%",
                      [([(s[0], s[3]) for s in sv], C_BLUE, 2), ([(s[0], s[4]) for s in sv], C_RED, 2)],
                      tn, args.span, hlines=[(0, "#999999", None, "")], bands=sb, minspan=5.0, fmt="{:+.1f}")
            hl = [(S["R_ref"], "#888888", (4, 3), "R0")] if S["R_ref"] else []
            draw_plot(pcv, cell(0, 1), "Flex sensor resistance", "ohm",
                      [([(s[0], s[1]) for s in sv], "#cdbbe8", 1), ([(s[0], s[2]) for s in sv], C_PURPLE, 2)],
                      tn, args.span, hlines=hl, bands=sb, minspan=20.0, fmt="{:.0f}", label_idx=1)

            mm = "run with --motor" if not args.motor else "no motor data yet"
            hl = []
            if open_pos is not None:
                hl = [(open_pos, "#777777", (4, 3), "open"), (closed_pos, "#777777", (4, 3), "closed")]
            draw_plot(pcv, cell(1, 0), "Motor position  (blue = actual, orange = goal, green = trajectory)", "ticks",
                      [([(m[0], m[3]) for m in mv], C_GREEN, 1), ([(m[0], m[2]) for m in mv], C_ORANGE, 1),
                       ([(m[0], m[1]) for m in mv], C_BLUE, 2)],
                      tn, args.span, hlines=hl, bands=mb, minspan=50.0, fmt="{:.0f}", label_idx=2, empty=mm)
            draw_plot(pcv, cell(1, 1), "Motor velocity", "rpm", [([(m[0], m[4]) for m in mv], C_BLUE, 2)],
                      tn, args.span, hlines=[(0, "#999999", None, "")] if mv else [], bands=mb, minspan=5.0, empty=mm)
            draw_plot(pcv, cell(2, 0), "Motor PWM  (effort command)", "% of max",
                      [([(m[0], m[5]) for m in mv], C_ORANGE, 2)],
                      tn, args.span, hlines=[(0, "#999999", None, "")] if mv else [], bands=mb, minspan=5.0, empty=mm)
            draw_plot(pcv, cell(2, 1), "Motor current  (load on the finger)", "mA",
                      [([(m[0], m[6]) for m in mv], C_RED, 2)],
                      tn, args.span, hlines=[(0, "#999999", None, "")] if mv else [], bands=mb, minspan=20.0,
                      fmt="{:.0f}", empty=mm)
            draw_perf()
        except tk.TclError:
            pass

    def draw_perf():
        pv.delete("all")
        rows = S["rows"]
        n = max(args.attempts, len(rows), 1)
        bold, small = ("Helvetica", 9, "bold"), ("Helvetica", 8)

        def frame(x0, y0, x1, y1, title, ymax):
            pv.create_text(x0, y0 - 11, text=title, anchor="w", font=bold, fill="gray20")
            pv.create_rectangle(x0, y0, x1, y1, outline="gray65", fill="white")
            pv.create_text(x0 - 3, y0, text=f"{ymax:.3g}", anchor="e", font=small, fill="gray40")
            pv.create_text(x0 - 3, y1, text="0", anchor="e", font=small, fill="gray40")

        # left: peak reached (bar) vs goal (black tick) per attempt
        x0, y0, x1, y1 = 40, 24, 250, 104
        ymax = max([float(r[1]) for r in rows] + [float(r[4]) for r in rows] + [S["thr"], 1.0]) * 1.15
        frame(x0, y0, x1, y1, "Peak reached vs goal  [%]", ymax)
        bw = (x1 - x0) / n
        for i, r in enumerate(rows):
            bx0, bx1 = x0 + i * bw + 1.5, x0 + (i + 1) * bw - 1.5
            pk, th = float(r[4]), float(r[1])
            ytop = y1 - (y1 - y0) * max(0.0, min(pk, ymax)) / ymax
            pv.create_rectangle(bx0, ytop, bx1, y1, fill="#3a9d4a" if int(r[2]) else "#d9534f", outline="")
            yt = y1 - (y1 - y0) * min(th, ymax) / ymax
            pv.create_line(bx0 - 1, yt, bx1 + 1, yt, fill="black", width=2)
        # right: time to goal per attempt (failures = full pale bar)
        x0, y0, x1, y1 = 300, 24, 510, 104
        frame(x0, y0, x1, y1, "Time to goal  [s]   (pale = time out)", args.window)
        for i, r in enumerate(rows):
            bx0, bx1 = x0 + i * bw + 1.5, x0 + (i + 1) * bw - 1.5
            if r[3] == "":
                pv.create_rectangle(bx0, y0, bx1, y1, fill="#f2c4c2", outline="")
            else:
                ytop = y1 - (y1 - y0) * min(float(r[3]), args.window) / args.window
                pv.create_rectangle(bx0, ytop, bx1, y1, fill="#3a9d4a", outline="")
        if n <= 20:
            for i in range(n):
                pv.create_text(40 + (i + 0.5) * bw, 112, text=str(i + 1), font=("Helvetica", 7), fill="gray50")
                pv.create_text(300 + (i + 0.5) * bw, 112, text=str(i + 1), font=("Helvetica", 7), fill="gray50")

        # text block
        n_ok = sum(int(r[2]) for r in rows)
        times = [float(r[3]) for r in rows if r[3] != ""]
        l1 = f"success {n_ok}/{len(rows)}"
        if rows:
            l1 += f" ({100 * n_ok / len(rows):.0f} %)"
        if times:
            l1 += f"   median {statistics.median(times):.2f} s   best {min(times):.2f} s"
        l1 += f"   goal now {S['thr']:.1f} %"
        if rows:
            r = rows[-1]
            l2 = f"last: peak {r[4]} %   time {r[3] or '-'} s   max rate {r[7]} %/s   noise {r[6]} %"
        else:
            l2 = f"sensor noise at rest: {S['noise']:.3f} %"
        m = S["mot"]
        if not args.motor:
            l3 = "motor: not used (run with --motor)"
        elif m:
            l3 = (f"motor: pos {m['pos']} ({m['pos'] * 360 / 4096:.0f} deg)   err {m['goal'] - m['pos']}   "
                  f"{m['volt']:.1f} V   {m['temp']} C   {'MOVING' if m['moving'] else 'holding'}")
        else:
            l3 = "motor: no data"
        l4 = (f"sample rate: sensor {S['rate_s']:.0f} Hz   motor {S['rate_m']:.0f} Hz   "
              f"finger {'OPEN' if S['is_open'] else 'CLOSED'}" if args.motor else
              f"sample rate: sensor {S['rate_s']:.0f} Hz")
        for k, tx in enumerate((l1, l2, l3, l4)):
            pv.create_text(8, 126 + 16 * k, text=tx, anchor="w", font=("Helvetica", 9), fill="gray20")
        lx = 8                                                  # phase legend
        for ph, lab in (("rest", "relax"), ("go", "go"), ("result", "result"), ("move", "motor move")):
            pv.create_rectangle(lx, 182, lx + 12, 192 - 2, fill=PH_COL[ph], outline="gray60")
            pv.create_text(lx + 16, 186, text=lab, anchor="w", font=small, fill="gray35")
            lx += 90

    # ------------------------------------------------------------------ periodic service (poll motor, plots, flush)
    def drain():
        """All sensor samples collected since the last call: [(time.time(), R), ...]."""
        if args.sim:
            r = read()
            return [(time.time(), r)] if r is not None else []
        out = []
        while True:
            try:
                out.append(Q.popleft())
            except IndexError:
                return out

    def service(in_move=False):
        t = time.time()
        if in_move:                                       # keep showing the sensor while the motor moves
            for ts, r in drain():
                S["cnt_s"] += 1
                SENS.append((ts - T0, r, r, None, None, "move"))
                log_row(r, r, None, None, t=ts - T0)
        if S["motor_on"] and not S["released"] and t - S["t_mot"] >= 1.0 / args.motor_hz:
            S["t_mot"] = t
            if sample_motor() and in_move:
                log_row()                                     # sensor is not read during a move: motor-only row
        if t - S["t_plot"] >= 1.0 / args.fps:
            S["t_plot"] = t
            redraw()
        if t - S["t_rate"] >= 1.0:
            dt = t - S["t_rate"]
            S["rate_s"], S["rate_m"] = S["cnt_s"] / dt, S["cnt_m"] / dt
            S["cnt_s"] = S["cnt_m"] = 0
            S["t_rate"] = t
        if S["tele"] is not None and t - S["t_flush"] >= 1.0:
            S["t_flush"] = t
            try:
                S["tele"].flush()
            except Exception:
                pass

    def finish():
        if S["finished"]:
            return
        S["finished"] = True
        S["done"] = True
        release_all()                                         # torque OFF first, before anything else
        if S["tele"] is not None:
            try:
                S["tele"].close()
            except Exception:
                pass
            S["tele"], S["tw"] = None, None
        if S["rows"]:
            try:
                with open(path, "w", newline="") as f:
                    w = csv.writer(f)
                    w.writerow(["attempt", "threshold_pct", "success", "time_to_trigger_s", "peak_pct", "R_ref_ohm",
                                "baseline_noise_pct", "max_rate_pct_s", "move_peak_current_mA",
                                "move_peak_pwm_pct", "move_time_s"])
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
        if S["phase"] == "go":                                # live numbers under the bar
            el = time.time() - S["t0"]
            val = "-" if x is None else f"{x:.1f}"
            cv.create_text(W / 2, BOT + 24, text=f"{val} % of {S['thr']:.1f} %     t = {el:.1f} / {args.window:.0f} s",
                           font=("Helvetica", 12), fill="gray30")
            frac = max(0.0, 1.0 - el / args.window)           # time-left bar
            cv.create_rectangle(170, BOT + 38, 170 + 180 * frac, BOT + 46, fill="#c0392b", outline="")
            cv.create_rectangle(170, BOT + 38, 350, BOT + 46, outline="gray60")

    def tick():
        if S["stop"] and not S["finished"]:
            finish()
        if S["done"]:
            return
        el = time.time() - S["t0"]
        samples = drain()                                     # EVERY sample since the last tick (no backlog)
        ph = S["phase"]
        sg = S["sign"]
        rest_total = args.rest + S["extra_rest"]
        x = None
        for ts, r in samples:
            S["win"].append(r)
            S["cnt_s"] += 1
            R = statistics.mean(S["win"])
            xs = sg * (R - S["R_ref"]) / S["R_ref"] * 100 if S["R_ref"] else None
            e = ts - S["t0"]                                  # time of THIS sample inside the phase
            in_go = ph == "go" and xs is not None
            dRR = sg * xs if in_go else None                  # true dR/R0 (sign = direction of the goal)
            goal = sg * S["thr"] if ph == "go" else None
            SENS.append((ts - T0, r, R, dRR, goal, ph))
            log_row(r, R, dRR, goal, t=ts - T0)
            if ph == "rest" and e >= rest_total - args.baseline:
                S["base"].append(r)
            if in_go:
                S["peak"] = max(S["peak"], xs)
                S["xh"].append((e, xs))                       # max rate of rise over a ~0.2 s window
                while S["xh"] and e - S["xh"][0][0] > 0.2:
                    S["xh"].popleft()
                if len(S["xh"]) >= 2 and e - S["xh"][0][0] > 0.05:
                    S["rate_max"] = max(S["rate_max"], (xs - S["xh"][0][1]) / (e - S["xh"][0][0]))
                if xs >= S["thr"] and S["hit"] is None:
                    S["hit"] = max(e, 0.0)                    # first sample that reached the line
            x = xs
        while SENS and SENS[0][0] < now() - args.span - 5:
            SENS.popleft()
        if samples:
            S["x_last"] = x
        x = S["x_last"]

        if ph == "wait":                                      # live plots run, the task waits for START
            draw(None, "Press START when ready", "#555555")

        elif ph == "rest":
            rest_total = args.rest + S["extra_rest"]       # longer after a motor move (sensor settles)
            draw(None, "RELAX - keep still", "#2a7f2a")
            if el >= rest_total and len(S["base"]) >= 5:
                S["R_ref"] = statistics.mean(S["base"])
                S["noise"] = statistics.pstdev(S["base"]) / S["R_ref"] * 100   # noise floor of the zeroing window
                S["base"] = []
                S["win"].clear()
                S["peak"] = 0.0
                S["rate_max"] = 0.0
                S["xh"].clear()
                S["hit"], S["x_last"] = None, None
                S["extra_rest"] = 0.0
                set_phase("go")

        elif ph == "go":
            draw(x, go_text(), "#c0392b")
            if S["hit"] is not None:
                S["t_trig"], S["last_ok"] = S["hit"], True
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
            service()
            root.after(10, tick)

    def move_motor(target):
        """Same logic as ftm.move_to, but keeps the window alive so Esc / close / Ctrl+C work mid-move,
        and keeps the motor plots running during the move."""
        if not ftm.set_goal(target):
            return False
        t0 = time.time()
        while time.time() - t0 < ftm.MOVE_TIMEOUT:
            try:
                service(in_move=True)
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
        row = [S["n"], f"{S['thr']:.2f}", int(ok),
               "" if S["t_trig"] is None else f"{S['t_trig']:.2f}",
               f"{S['peak']:.2f}", f"{S['R_ref']:.1f}",
               f"{S['noise']:.3f}", f"{S['rate_max']:.2f}", "", "", ""]   # last 3: filled after a motor move
        S["rows"].append(row)
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
        if ok and args.motor:                                 # complete the movement (blocks the GUI, plots keep running)
            draw(None, "Moving...", "gray30")
            root.update()
            target = closed_pos if S["is_open"] else open_pos
            print(f"  {'closing' if S['is_open'] else 'opening'} -> {target}")
            S["moving"], S["mv_peak_cur"], S["mv_peak_pwm"] = True, 0.0, 0.0
            t_mv = time.time()
            done = move_motor(target)
            S["moving"] = False
            row[8], row[9] = f"{S['mv_peak_cur']:.0f}", f"{S['mv_peak_pwm']:.1f}"
            row[10] = f"{time.time() - t_mv:.2f}"
            if done:
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
            Q.clear()
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
        def on_move(e):                                       # mouse height over the task bar -> signal
            S["mouse"] = max(0.0, (BOT - e.y) / (BOT - TOP)) * YMAX * S["thr"]
        cv.bind("<Motion>", on_move)
    root.protocol("WM_DELETE_WINDOW", finish)                 # closing the window
    root.bind("<Escape>", lambda e: finish())                 # Esc = stop now
    root.bind("<space>", lambda e: start_task())              # Space / Enter = START
    root.bind("<Return>", lambda e: start_task())
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