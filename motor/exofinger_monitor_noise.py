"""
Exofinger LIVE MONITOR - NO movement control.

Purpose: see in real time what the motor "feels" while it holds the finger softly, so you can choose the
trigger threshold AND the amount of filtering. The motor NEVER moves on its own here: the goal position is
always the position where the finger was when the hold started (or torque is OFF, if you choose so).

What you see (5 stacked plots, last WINDOW_S seconds, all share the same time axis):
  1. POSITION     displacement of the finger from the hold position [ticks]   (+ PUSH_TICKS lines)
  2. CURRENT      raw current, filtered current (active filter), baseline and baseline +/- threshold [mA]
  3. PUSH SIGNAL  "push toward the target" for SEVERAL filter strengths at the same time (one line per
                  EMA alpha; the active one is thick) versus the threshold (orange dashed).
                  Triangles = the moment the real control script WOULD have started the motor.
  4. NOISE        for every filter strength: K x (noise of that filtered signal at rest) [mA].
                  Compare it with the threshold (orange dashed) and the minimum threshold (gray dotted):
                  a filter is good enough when its line stays BELOW the threshold lines.
                  A table in the corner lists noise, K x noise, and the filter delay (lag) for each alpha.
  5. PWM          present PWM output [% of PWM limit]

How to choose the filter: smaller alpha = more filtering = less noise, but more delay.
Take the LARGEST alpha (least delay) whose "K x noise" is below the threshold, then check in plot 3
that a real push still crosses the threshold clearly.

The noise numbers are measured only while the finger is idle (not pushing), so keep the finger resting
while you read them, and then push to see the signal.

KEYS (click on the plot window first):
  r        re-zero: hold at the present position, then re-measure baseline + noise (do NOT touch the finger)
  d        toggle the direction to monitor: OPEN (+1) <-> CLOSE (-1)
  t        toggle torque ON (soft hold) <-> OFF (finger free, just reading)
  + / -    raise / lower the minimum current threshold by 1 mA
  ] / [    raise / lower NOISE_K by 0.5
  f / g    MORE / LESS filtering (changes the active alpha; the trigger uses the active one)
  s        save a PNG snapshot of the figure
  q        quit (torque is turned OFF, data is saved to CSV)
"""
import bisect
import time
from datetime import datetime

import matplotlib.pyplot as plt

from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

# ------------------------- SETTINGS -------------------------
CURRENT_UNIT_MA = 2.69      # mA per raw current unit
PWM_MAX = 885               # raw PWM value = 100 %  (default limit of the X-series; change if yours differs)

PORT = "/dev/cu.usbserial-FT763G3X"
BAUDRATE = 57600
ID = 1

# --- what the motor does ---
TORQUE_ON = True            # True = soft hold at the present position; False = torque off (finger free)
SOFT_CURRENT = 60           # current limit while holding (1 unit = 2.69 mA) -> ~161 mA. Same as the control script
HOLD_SPEED = 10             # Profile Velocity, only a safety net (the goal never changes, so nothing moves)

# --- trigger parameters: keep them equal to the control script ---
DIRECTION = +1              # +1 = monitor an OPENING push (position increases), -1 = a CLOSING push
CURRENT_TRIGGER_MA = 15     # minimum change of the current vs rest that counts as a push [mA]
NOISE_K = 3.0               # threshold is at least NOISE_K * (noise of the filtered current)
THRESHOLD_MAX_MA = 20       # the noise-based part of the threshold never exceeds this [mA]
FILTER_ALPHA = 0.5          # EMA filter on the current (the ACTIVE filter at start-up; change live with f / g)
BASELINE_ALPHA = 0.002      # baseline drift tracking speed while idle
NOISE_TRACK_ALPHA = 0.005   # live noise estimate (for the threshold) while idle
MIN_DISPLACEMENT = 3        # ticks: motion-trigger fallback
PUSH_TICKS = 10             # ticks: displacement trigger (0 = off)
N_CONSEC = 3                # consecutive samples that must satisfy a trigger condition

# --- noise visualization ---
FILTER_CANDIDATES = [1.0, 0.5, 0.3, 0.2, 0.1, 0.05]   # EMA alphas compared side by side (1.0 = no filter)
NOISE_BETA = 0.01           # speed of the per-filter noise estimate (0.01 ~ averages the last ~100 samples)

# --- calibration ---
SETTLE_TIME = 0.7           # seconds to wait after (re)starting the hold before measuring
CALIB_TIME = 1.0            # seconds to measure the resting baseline (do NOT touch the finger)

# --- display ---
WINDOW_S = 10               # seconds shown on the plots
PLOT_PERIOD = 0.05          # seconds between plot refreshes (raise it if the sampling rate drops too much)
STATUS_PERIOD = 0.2         # seconds between text refreshes
MONITOR_PERIOD = 1.0        # seconds between voltage / temperature reads
SAVE_CSV = True             # save every sample to a CSV when you quit

WRITE_TRIES = 5

# ------------------------- ADDRESSES -------------------------
ADDR_OPERATING_MODE   = 11
ADDR_TORQUE_ENABLE    = 64
ADDR_HARDWARE_ERROR   = 70
ADDR_GOAL_CURRENT     = 102
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION    = 116
ADDR_PRESENT_PWM      = 124   # 2 bytes; current (2) at 126, velocity (4) at 128, position (4) at 132
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_VOLTAGE  = 144   # 2 bytes (0.1 V); temperature (1 byte, deg C) follows at 146

port = PortHandler(PORT)
pk = PacketHandler(2.0)

# filter strengths compared on the plots, from weakest filtering (1.0) to strongest
ALPHAS = sorted(set(FILTER_CANDIDATES) | {FILTER_ALPHA}, reverse=True)
N_F = len(ALPHAS)

# values that can be changed live with the keyboard
params = {"direction": DIRECTION, "trigger_ma": CURRENT_TRIGGER_MA, "noise_k": NOISE_K, "torque": TORQUE_ON,
          "alpha_idx": ALPHAS.index(FILTER_ALPHA)}
cmd = {"recal": False, "toggle_torque": False, "snapshot": False, "quit": False}

# full history (used for the CSV) - the plots show only the last WINDOW_S seconds
L = {k: [] for k in ("t", "pos", "disp", "cur", "curf", "base", "hi", "lo", "toward", "thr", "pwm", "hit", "fired")}
for _i in range(N_F):
    L[f"tw{_i}"] = []      # push toward target, filter i [mA]
    L[f"nz{_i}"] = []      # noise (std at rest) of filter i [mA]
events = {"t": [], "toward": [], "curf": []}     # moments where the control would have started the motor
t0 = None


def active_alpha():
    return ALPHAS[params["alpha_idx"]]


# ------------------------- LOW-LEVEL HELPERS (same as the control script) -------------------------
def write_verified(address, value, n_bytes, tries=WRITE_TRIES, quiet=False):
    """Write with retries. Returns True only if the motor acknowledged without error."""
    fn = {1: pk.write1ByteTxRx, 2: pk.write2ByteTxRx, 4: pk.write4ByteTxRx}[n_bytes]
    last = ""
    for _ in range(tries):
        result, error = fn(port, ID, address, value)
        if result == COMM_SUCCESS and (error & 0x7F) == 0:
            return True
        last = pk.getTxRxResult(result) if result != COMM_SUCCESS else pk.getRxPacketError(error & 0x7F)
        time.sleep(0.01)
    if not quiet:
        print(f"WRITE FAILED (address {address}) after {tries} tries: {last}")
    return False


def read_raw(address, length, tries=3):
    """Read bytes. Returns a list of bytes, or None."""
    for _ in range(tries):
        data, result, error = pk.readTxRx(port, ID, address, length)
        if result == COMM_SUCCESS and (error & 0x7F) == 0 and len(data) == length:
            return data
        time.sleep(0.005)
    return None


def read_position():
    data = read_raw(ADDR_PRESENT_POSITION, 4)
    if data is None:
        return None
    return int.from_bytes(bytes(data), "little", signed=True)


def set_goal_verified(goal, tries=10):
    """Write the goal position and read it back until it matches (or give up)."""
    for _ in range(tries):
        if write_verified(ADDR_GOAL_POSITION, goal & 0xFFFFFFFF, 4, tries=1, quiet=True):
            gp = read_raw(ADDR_GOAL_POSITION, 4)
            if gp is not None and int.from_bytes(bytes(gp), "little", signed=True) == goal:
                return True
        time.sleep(0.02)
    return False


def hardware_error_status():
    data = read_raw(ADDR_HARDWARE_ERROR, 1)
    return None if data is None else data[0]


def describe_hw_error(hw):
    names = {0: "input voltage", 2: "overheating", 3: "motor encoder", 4: "electrical shock", 5: "overload"}
    return ", ".join(n for bit, n in names.items() if hw & (1 << bit)) or "unknown"


def clear_hardware_error():
    """If the motor latched a hardware error, report it and reboot to clear it."""
    hw = hardware_error_status()
    if hw:
        print(f"Hardware error status = {hw:#010b} ({describe_hw_error(hw)}) -> rebooting motor")
        pk.reboot(port, ID)
        time.sleep(1.5)
        hw = hardware_error_status()
        if hw:
            print(f"Still in error after reboot: {hw:#010b} ({describe_hw_error(hw)}). Check power supply.")
            return False
    return True


def release_motor():
    """Torque OFF no matter what state the port is in (also after Ctrl+C mid-transaction)."""
    port.is_using = False
    try:
        port.ser.reset_input_buffer()
    except Exception:
        pass
    ok = write_verified(ADDR_TORQUE_ENABLE, 0, 1, tries=10)
    if not ok:
        print("!!! COULD NOT TURN TORQUE OFF - power-cycle the motor !!!")
    port.closePort()


def sample():
    """One packet with PWM + current + velocity + position (12 bytes from address 124).
    Returns (position, current_raw, pwm_raw) or None on a communication error."""
    data, result, error = pk.readTxRx(port, ID, ADDR_PRESENT_PWM, 12)
    if result != COMM_SUCCESS or (error & 0x7F) != 0 or len(data) != 12:
        return None
    pwm = int.from_bytes(bytes(data[0:2]), "little", signed=True)
    cur = int.from_bytes(bytes(data[2:4]), "little", signed=True)
    pos = int.from_bytes(bytes(data[8:12]), "little", signed=True)
    return pos, cur, pwm


def read_volt_temp():
    data = read_raw(ADDR_PRESENT_VOLTAGE, 3, tries=1)
    if data is None:
        return None
    return int.from_bytes(bytes(data[0:2]), "little") / 10.0, data[2]


def set_torque(on):
    """Torque ON: first make the goal = present position (so the motor can NOT move), then enable.
    Torque OFF: simply disable."""
    if on:
        pos = read_position()
        if pos is None or not set_goal_verified(pos):
            print("Could not set the hold position -> torque stays OFF")
            return False
        if not write_verified(ADDR_GOAL_CURRENT, SOFT_CURRENT, 2):
            return False
        return write_verified(ADDR_TORQUE_ENABLE, 1, 1)
    return write_verified(ADDR_TORQUE_ENABLE, 0, 1)


# ------------------------- TRIGGER LOGIC (same as exofinger_v2_fixed.py) -------------------------
def filter_noise_factor(alpha):
    """Noise of an EMA-filtered white signal relative to the raw noise."""
    return (alpha / (2 - alpha)) ** 0.5


def threshold_from_noise(sigma):
    """Trigger threshold (raw units) from the raw-current noise sigma (raw units).
    max( minimum threshold , noise_k * noise of the filtered current ), capped at THRESHOLD_MAX_MA."""
    noise_part = params["noise_k"] * sigma * filter_noise_factor(active_alpha())
    return max(params["trigger_ma"] / CURRENT_UNIT_MA, min(noise_part, THRESHOLD_MAX_MA / CURRENT_UNIT_MA))


class PushDetector:
    """Everything the control script does while WAITING for a push, but without ever moving the motor.
    Additionally runs ALL candidate filters in parallel and measures the resting noise of each."""

    def __init__(self):
        self.baseline = 0.0      # resting current [raw]
        self.var = 0.0           # running variance of the resting current noise [raw^2] (used by the threshold)
        self.hold_pos = 0        # position where the finger is held [ticks]
        self.cf = [0.0] * N_F    # filtered current for every candidate alpha [raw]
        self.nvar = [0.0] * N_F  # resting variance of every filtered signal around the baseline [raw^2]
        self.prev_cur = None
        self.count = 0           # consecutive samples that satisfied a trigger condition
        self.thr = 0.0           # present threshold [raw]

    @property
    def cur_f(self):
        return self.cf[params["alpha_idx"]]

    def calibrate(self, baseline, sigma, hold_pos):
        self.baseline, self.var, self.hold_pos = baseline, sigma ** 2, hold_pos
        self.cf = [baseline] * N_F
        self.nvar = [sigma ** 2 * a / (2 - a) for a in ALPHAS]    # starting guess (white noise), then measured
        self.prev_cur, self.count = None, 0
        self.thr = threshold_from_noise(sigma)

    def update(self, pos, cur):
        direction = params["direction"]
        self.thr = threshold_from_noise(self.var ** 0.5)
        thr = self.thr

        for i, a in enumerate(ALPHAS):                                # all EMA filters in parallel
            self.cf[i] += a * (cur - self.cf[i])
        cur_f = self.cur_f                                            # the ACTIVE filter drives the trigger
        dd = 0 if self.prev_cur is None else cur - self.prev_cur      # sample-to-sample change (noise)
        self.prev_cur = cur

        moved = (pos - self.hold_pos) * direction    # ticks moved toward the target (positive = right way)
        d = cur_f - self.baseline                    # signed change of the filtered current vs rest
        toward = -direction * d                      # push toward the target (positive = right way)

        by_current = toward >= thr                                    # 1) the motor works harder against me
        by_motion = abs(d) >= thr and moved >= MIN_DISPLACEMENT       # 2) current changed AND finger moved
        by_disp = PUSH_TICKS > 0 and moved >= PUSH_TICKS              # 3) finger clearly moved
        hit = by_current or by_motion or by_disp

        toward_all = [-direction * (c - self.baseline) for c in self.cf]

        if hit:
            self.count += 1
        else:
            self.count = 0
            if toward < 0.5 * thr and abs(pos - self.hold_pos) <= MIN_DISPLACEMENT:   # finger is idle
                for i in range(N_F):                                  # resting noise of every filter
                    e = self.cf[i] - self.baseline
                    self.nvar[i] += NOISE_BETA * (e * e - self.nvar[i])
                self.baseline += BASELINE_ALPHA * d                   # slow drift tracking while idle
                if NOISE_TRACK_ALPHA > 0:
                    self.var += NOISE_TRACK_ALPHA * (dd * dd / 2 - self.var)   # live noise estimate
        fired = hit and self.count == N_CONSEC    # exactly the sample where the control would start the motor
        return {"d": d, "toward": toward, "thr": thr, "moved": moved, "hit": hit, "fired": fired,
                "by_current": by_current, "by_motion": by_motion, "by_disp": by_disp,
                "toward_all": toward_all, "sigma_all": [v ** 0.5 for v in self.nvar]}


det = PushDetector()


def measure_baseline(duration):
    """Mean and noise (std of consecutive-sample differences / sqrt 2) of the resting current [raw], or None."""
    vals = []
    t_c = time.time()
    while time.time() - t_c < duration:
        s = sample()
        if s is not None:
            vals.append(s[1])
    if not vals:
        return None
    mean = sum(vals) / len(vals)
    if len(vals) >= 3:
        diffs = [b - a for a, b in zip(vals, vals[1:])]
        std = (sum(x * x for x in diffs) / len(diffs)) ** 0.5 / (2 ** 0.5)
    else:
        std = 0.0
    return mean, std


# ------------------------- FIGURE -------------------------
ui = {}


def build_figure():
    # take the default matplotlib shortcuts that clash with ours
    for key in ("keymap.home", "keymap.save", "keymap.quit", "keymap.fullscreen", "keymap.grid",
                "keymap.grid_minor"):
        if key in plt.rcParams:
            plt.rcParams[key] = []
    plt.ion()
    fig, axes = plt.subplots(5, 1, sharex=True, figsize=(12, 11))
    fig.subplots_adjust(top=0.84, bottom=0.07, left=0.07, right=0.98, hspace=0.12)
    a_pos, a_cur, a_push, a_noise, a_pwm = axes
    cmap = plt.get_cmap("tab10")
    colors = [cmap(i) for i in range(N_F)]
    ln = {}

    # 1. position
    ln["disp"], = a_pos.plot([], [], color="tab:blue", label="position - hold position")
    a_pos.axhline(0, color="gray", linewidth=0.8)
    if PUSH_TICKS > 0:
        a_pos.axhline(PUSH_TICKS, color="orange", linestyle=":", label="+/- PUSH_TICKS")
        a_pos.axhline(-PUSH_TICKS, color="orange", linestyle=":")
    a_pos.set_ylabel("ticks")

    # 2. current
    ln["cur"], = a_cur.plot([], [], color="tab:red", alpha=0.35, label="current (raw)")
    ln["curf"], = a_cur.plot([], [], color="tab:red", linewidth=1.8, label="current (filtered, active)")
    ln["base"], = a_cur.plot([], [], color="gray", linestyle="--", label="baseline")
    ln["hi"], = a_cur.plot([], [], color="orange", linestyle=":", label="baseline +/- threshold")
    ln["lo"], = a_cur.plot([], [], color="orange", linestyle=":")
    ln["ev_cur"], = a_cur.plot([], [], "v", color="green", markersize=9)
    a_cur.set_ylabel("mA")

    # 3. push signal, one line per filter
    ln["push"] = [a_push.plot([], [], color=colors[i], linewidth=1.0)[0] for i in range(N_F)]
    ln["thr"], = a_push.plot([], [], color="orange", linestyle="--", linewidth=1.6, label="threshold")
    ln["ev_push"], = a_push.plot([], [], "v", color="green", markersize=9, label="control would START here")
    a_push.axhline(0, color="gray", linewidth=0.8)
    a_push.set_ylabel("push toward target [mA]")

    # 4. noise, one line per filter (K x noise) versus the thresholds
    ln["noise"] = [a_noise.plot([], [], color=colors[i], linewidth=1.0)[0] for i in range(N_F)]
    ln["thr_n"], = a_noise.plot([], [], color="orange", linestyle="--", linewidth=1.6, label="threshold in use")
    ln["min_n"], = a_noise.plot([], [], color="gray", linestyle=":", linewidth=1.6, label="minimum threshold")
    a_noise.set_ylabel("K x noise [mA]")
    ui["table"] = a_noise.text(0.995, 0.97, "", transform=a_noise.transAxes, ha="right", va="top",
                               family="monospace", fontsize=7.5,
                               bbox=dict(facecolor="white", alpha=0.88, edgecolor="gray", boxstyle="round,pad=0.3"))

    # 5. pwm
    ln["pwm"], = a_pwm.plot([], [], color="tab:brown", label="PWM output")
    a_pwm.axhline(0, color="gray", linewidth=0.8)
    a_pwm.set_ylabel("% of PWM limit")
    a_pwm.set_xlabel("Time [s]")

    for ax in axes:
        ax.grid(True)
    for ax in (a_pos, a_cur, a_pwm):
        ax.legend(loc="upper left", fontsize=8)

    ui["status"] = fig.text(0.01, 0.99, "", va="top", ha="left", family="monospace", fontsize=9,
                            bbox=dict(facecolor="whitesmoke", edgecolor="gray", boxstyle="round,pad=0.4"))
    fig.text(0.01, 0.012, "keys: r re-zero | d direction | t torque | +/- min threshold | [ ] noise K | "
                          "f/g more/less filter | s snapshot | q quit", fontsize=8, family="monospace")
    fig.canvas.mpl_connect("key_press_event", on_key)
    ui.update(fig=fig, axes=axes, ln=ln)
    refresh_styles()
    plt.show(block=False)


def refresh_styles():
    """Thick line + '(active)' label for the filter used by the trigger."""
    ln, axes = ui["ln"], ui["axes"]
    a_push, a_noise = axes[2], axes[3]
    for i, a in enumerate(ALPHAS):
        active = (i == params["alpha_idx"])
        label = f"alpha={a:.2f}" + (" (active)" if active else "")
        for line in (ln["push"][i], ln["noise"][i]):
            line.set_linewidth(2.6 if active else 0.9)
            line.set_alpha(1.0 if active else 0.75)
            line.set_label(label)
    a_push.legend(loc="upper left", fontsize=7, ncol=4)
    a_noise.legend(loc="upper left", fontsize=7, ncol=4)


def on_key(event):
    k = event.key
    if k == "r":
        cmd["recal"] = True
    elif k == "d":
        params["direction"] *= -1
        det.count = 0
    elif k == "t":
        cmd["toggle_torque"] = True
    elif k in ("+", "="):
        params["trigger_ma"] += 1
    elif k in ("-", "_"):
        params["trigger_ma"] = max(1, params["trigger_ma"] - 1)
    elif k == "]":
        params["noise_k"] += 0.5
    elif k == "[":
        params["noise_k"] = max(0.0, params["noise_k"] - 0.5)
    elif k == "f":                                    # more filtering = smaller alpha = next in the list
        params["alpha_idx"] = min(N_F - 1, params["alpha_idx"] + 1)
        det.count = 0
        refresh_styles()
    elif k == "g":                                    # less filtering
        params["alpha_idx"] = max(0, params["alpha_idx"] - 1)
        det.count = 0
        refresh_styles()
    elif k == "s":
        cmd["snapshot"] = True
    elif k == "q":
        cmd["quit"] = True


def fit_y(ax, arrays, include=(), min_span=1.0):
    """Set the y-limits to the visible data (plus fixed values in `include`), with a minimum span so that
    pure noise is not blown up to fill the whole plot."""
    vals = [v for a in arrays for v in a] + list(include)
    if not vals:
        return
    lo, hi = min(vals), max(vals)
    if hi - lo < min_span:
        mid = (hi + lo) / 2
        lo, hi = mid - min_span / 2, mid + min_span / 2
    pad = (hi - lo) * 0.1
    ax.set_ylim(lo - pad, hi + pad)


def update_plot():
    if not L["t"]:
        return
    t_now = L["t"][-1]
    i0 = bisect.bisect_left(L["t"], t_now - WINDOW_S)
    x = L["t"][i0:]
    ln, axes = ui["ln"], ui["axes"]
    for key in ("disp", "cur", "curf", "base", "hi", "lo", "thr", "pwm"):
        ln[key].set_data(x, L[key][i0:])
    ln["thr_n"].set_data(x, L["thr"][i0:])
    ln["min_n"].set_data(x, [params["trigger_ma"]] * len(x))

    k_noise = params["noise_k"]
    noise_k_arrays = []
    for i in range(N_F):
        ln["push"][i].set_data(x, L[f"tw{i}"][i0:])
        nk = [k_noise * v for v in L[f"nz{i}"][i0:]]
        ln["noise"][i].set_data(x, nk)
        noise_k_arrays.append(nk)

    j0 = bisect.bisect_left(events["t"], t_now - WINDOW_S)
    ln["ev_cur"].set_data(events["t"][j0:], events["curf"][j0:])
    ln["ev_push"].set_data(events["t"][j0:], events["toward"][j0:])

    a_pos, a_cur, a_push, a_noise, a_pwm = axes
    a_pos.set_xlim(t_now - WINDOW_S, t_now + 0.2)
    fit_y(a_pos, [L["disp"][i0:]], include=[PUSH_TICKS, -PUSH_TICKS] if PUSH_TICKS > 0 else [], min_span=12)
    fit_y(a_cur, [L["cur"][i0:], L["curf"][i0:], L["hi"][i0:], L["lo"][i0:]], min_span=40)
    fit_y(a_push, [L[f"tw{i}"][i0:] for i in range(N_F)] + [L["thr"][i0:]], include=[0], min_span=30)
    fit_y(a_noise, noise_k_arrays + [L["thr"][i0:]], include=[0, params["trigger_ma"]], min_span=10)
    fit_y(a_pwm, [L["pwm"][i0:]], min_span=4)


def noise_table(res, rate):
    """Text table: noise of every filter at rest, K x noise, delay, and whether it is below the minimum threshold."""
    rows = ["alpha   noise  Kxnoise  delay   vs min thr"]
    for i, a in enumerate(ALPHAS):
        sig = res["sigma_all"][i] * CURRENT_UNIT_MA
        kn = params["noise_k"] * sig
        delay_ms = (1 - a) / a / rate * 1000 if rate > 0 else 0.0     # low-frequency group delay of an EMA
        ok = "OK" if kn <= params["trigger_ma"] else "too noisy"
        mark = ">" if i == params["alpha_idx"] else " "
        rows.append(f"{mark}{a:4.2f}  {sig:5.1f}  {kn:6.1f}  {delay_ms:5.0f}ms  {ok}")
    rows.append(f"(mA; K={params['noise_k']:.1f}; min thr={params['trigger_ma']} mA)")
    return "\n".join(rows)


def update_status(res, pos, cur_ma, rate, vt):
    dir_txt = "OPEN (+)" if params["direction"] > 0 else "CLOSE (-)"
    tq = "ON (soft hold)" if params["torque"] else "OFF (free)"
    t_now = L["t"][-1]
    i1 = bisect.bisect_left(L["t"], t_now - 1.0)
    peak = max(L["toward"][i1:]) if L["toward"][i1:] else 0.0
    sigma_ma = (det.var ** 0.5) * CURRENT_UNIT_MA
    vt_txt = f"{vt[0]:.1f} V  {vt[1]} C" if vt else "-- V  -- C"
    conds = [n for n, f in (("CURRENT", res["by_current"]), ("MOTION", res["by_motion"]), ("DISPLACEMENT", res["by_disp"])) if f]
    state = f"TRIGGER CONDITION: {'+'.join(conds)}  count {det.count}/{N_CONSEC}" if conds else "idle (no trigger condition)"
    ui["status"].set_text(
        f"direction {dir_txt} | torque {tq} | filter alpha {active_alpha():.2f} | sampling {rate:.0f} Hz | {vt_txt}\n"
        f"position {pos} (moved {res['moved']:+.0f} ticks)  current {cur_ma:+.0f} mA  baseline {det.baseline * CURRENT_UNIT_MA:+.0f} mA  "
        f"raw noise {sigma_ma:.1f} mA\n"
        f"push toward target {res['toward'] * CURRENT_UNIT_MA:+.0f} mA (peak last 1 s: {peak:+.0f})   "
        f"THRESHOLD {res['thr'] * CURRENT_UNIT_MA:.0f} mA  (min {params['trigger_ma']} mA, K {params['noise_k']:.1f}, "
        f"max {THRESHOLD_MAX_MA})\n"
        f"{state}   |   would-have-started events: {len(events['t'])}")
    ui["table"].set_text(noise_table(res, rate))
    box = ui["status"].get_bbox_patch()
    box.set_facecolor("lightgreen" if det.count >= N_CONSEC else ("khaki" if det.count > 0 else "whitesmoke"))


def set_status(text):
    ui["status"].set_text(text)
    ui["fig"].canvas.draw_idle()
    plt.pause(0.001)


def recalibrate():
    """Hold at the present position, wait for the transient to die out, then measure baseline + noise."""
    set_status("RE-ZEROING ...  do NOT touch the finger  (settling, then measuring the resting current)")
    pos = read_position()
    if pos is None:
        print("Cannot read the position")
        return False
    if params["torque"] and not set_goal_verified(pos):      # new hold position = where the finger is now
        print("Could not set the hold position")
        return False
    t_s = time.time()
    while time.time() - t_s < SETTLE_TIME:
        sample()
    cal = measure_baseline(CALIB_TIME)
    if cal is None:
        print("No samples during calibration")
        return False
    det.calibrate(cal[0], cal[1], pos)
    print(f"baseline = {cal[0] * CURRENT_UNIT_MA:.0f} mA, noise = {cal[1] * CURRENT_UNIT_MA:.1f} mA, "
          f"threshold = {det.thr * CURRENT_UNIT_MA:.0f} mA")
    return True


def record(res, pos, cur, pwm):
    t = time.time() - t0
    unit = CURRENT_UNIT_MA
    L["t"].append(t)
    L["pos"].append(pos)
    L["disp"].append(pos - det.hold_pos)
    L["cur"].append(cur * unit)
    L["curf"].append(det.cur_f * unit)
    L["base"].append(det.baseline * unit)
    L["hi"].append((det.baseline + res["thr"]) * unit)
    L["lo"].append((det.baseline - res["thr"]) * unit)
    L["toward"].append(res["toward"] * unit)
    L["thr"].append(res["thr"] * unit)
    L["pwm"].append(100.0 * pwm / PWM_MAX)
    L["hit"].append(int(res["hit"]))
    L["fired"].append(int(res["fired"]))
    for i in range(N_F):
        L[f"tw{i}"].append(res["toward_all"][i] * unit)
        L[f"nz{i}"].append(res["sigma_all"][i] * unit)
    if res["fired"]:
        events["t"].append(t)
        events["toward"].append(res["toward"] * unit)
        events["curf"].append(det.cur_f * unit)


def save_csv():
    if not (SAVE_CSV and L["t"]):
        return
    name = datetime.now().strftime("exofinger_monitor_%Y%m%d_%H%M%S") + ".csv"
    keys = ["t", "pos", "disp", "cur", "curf", "base", "thr", "toward", "pwm", "hit", "fired"]
    header = ["time_s", "position_ticks", "displacement_ticks", "current_mA", "current_filtered_active_mA",
              "baseline_mA", "threshold_mA", "push_toward_target_active_mA", "pwm_percent",
              "trigger_condition", "would_start"]
    for i, a in enumerate(ALPHAS):
        keys += [f"tw{i}", f"nz{i}"]
        header += [f"push_toward_alpha_{a:.2f}_mA", f"noise_alpha_{a:.2f}_mA"]
    with open(name, "w") as f:
        f.write(",".join(header) + "\n")
        for row in zip(*[L[k] for k in keys]):
            f.write(",".join(f"{v:.3f}" if isinstance(v, float) else str(v) for v in row) + "\n")
    print(f"Saved {name}")


# ------------------------- MAIN -------------------------
def main():
    global t0

    if not port.openPort():
        print(f"Cannot open port {PORT}")
        return
    if not port.setBaudRate(BAUDRATE):
        print("Cannot set baud rate")
        port.closePort()
        return

    try:
        # ---------- setup: the goal is set to the PRESENT position BEFORE torque is enabled, so nothing moves ----------
        if not clear_hardware_error():
            return
        ok = (write_verified(ADDR_TORQUE_ENABLE, 0, 1)
              and write_verified(ADDR_OPERATING_MODE, 5, 1)           # 5: current-based position control
              and write_verified(ADDR_GOAL_CURRENT, SOFT_CURRENT, 2)
              and write_verified(ADDR_PROFILE_VELOCITY, HOLD_SPEED, 4))
        if not ok:
            print("Setup failed (check wiring / power / baud rate / ID). Exiting.")
            return
        if params["torque"] and not set_torque(True):
            print("Could not start the soft hold -> exiting")
            return

        t0 = time.time()
        build_figure()
        if not recalibrate():
            return

        # ---------- live loop ----------
        t_plot = t_status = t_mon = t_rate = time.time()
        n_rate, rate, vt = 0, 0.0, None
        res = {"toward": 0.0, "thr": det.thr, "moved": 0, "hit": False, "fired": False,
               "by_current": False, "by_motion": False, "by_disp": False,
               "toward_all": [0.0] * N_F, "sigma_all": [v ** 0.5 for v in det.nvar]}
        pos, cur_ma = 0, 0.0
        while plt.fignum_exists(ui["fig"].number) and not cmd["quit"]:
            # --- commands from the keyboard ---
            if cmd["toggle_torque"]:
                cmd["toggle_torque"] = False
                params["torque"] = not params["torque"]
                set_torque(params["torque"])
                cmd["recal"] = True
            if cmd["recal"]:
                cmd["recal"] = False
                recalibrate()
                t_plot = t_status = t_mon = t_rate = time.time()
                n_rate = 0
            if cmd["snapshot"]:
                cmd["snapshot"] = False
                name = datetime.now().strftime("exofinger_monitor_%Y%m%d_%H%M%S") + ".png"
                ui["fig"].savefig(name, dpi=150)
                print(f"Saved {name}")

            # --- one sample ---
            s = sample()
            now = time.time()
            if s is None:
                plt.pause(0.005)
                continue
            pos, cur, pwm = s
            res = det.update(pos, cur)
            record(res, pos, cur, pwm)
            cur_ma = cur * CURRENT_UNIT_MA
            n_rate += 1

            if now - t_mon > MONITOR_PERIOD:
                t_mon = now
                vt = read_volt_temp()
            if now - t_rate > 1.0:
                rate = n_rate / (now - t_rate)
                t_rate, n_rate = now, 0
            if now - t_status > STATUS_PERIOD:
                t_status = now
                update_status(res, pos, cur_ma, rate, vt)
            if now - t_plot > PLOT_PERIOD:
                t_plot = now
                update_plot()
                plt.pause(0.001)       # lets the window redraw and handle key presses

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        release_motor()                # torque OFF, always
        save_csv()                     # data is never lost


if __name__ == "__main__":
    main()
