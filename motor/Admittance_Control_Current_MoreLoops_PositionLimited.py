"""
Exofinger - assist-as-needed with a current-based trigger (V2).  [COMMENTED VERSION]

Built on the compliant-hold / current-trigger script that already works. Improvements, all in software:

  1. FILTERED, ADAPTIVE TRIGGER
       - the current is low-pass filtered (EMA) before it is compared with the threshold
       - the resting baseline is slowly tracked while idle (compensates drift / temperature / friction)
       - the threshold adapts to the measured noise: max(CURRENT_TRIGGER_MA, NOISE_K * noise)
  2. INTENT-SCALED ASSIST (the admittance-like part)
       - after the trigger, the strength of the push is measured for INTENT_WINDOW seconds
       - a stronger push -> faster assist speed, a gentle push -> slow assist
  3. SMOOTH START: Profile Acceleration ramps the speed up (no jerk at the start of the assist)
  4. STALL / RESISTANCE PROTECTION: if the finger stops making progress while the motor is pushing hard
       (spasm, obstruction, patient resisting), the assist stops and releases to the soft hold
  5. SAFETY MONITOR: temperature (abort) and input voltage (warning) are checked once per second
  6. REPEATED CYCLES: after each movement the exo goes back to the compliant hold and waits for the next
       push (open <-> close, decided from the position). CYCLES = 0 repeats until Ctrl+C.
  7. FULL LOG: position, current, velocity and phase are saved to CSV + PNG.

Phases: WAIT (soft hold, waiting for a push) -> INTENT (measuring push strength) -> ASSIST -> HOLD.

=====================================================================================================
QUICK GUIDE: HOW "OPENING THE HAND" WORKS (read this first)
=====================================================================================================
Position convention used in this script:
    MAX_POSITION =     0     -> hand fully OPEN
    MIN_POSITION = -1200     -> hand fully CLOSED
So OPENING = position INCREASES (direction = +1), CLOSING = position DECREASES (direction = -1).

Sign convention of the motor current (Dynamixel, current-based position mode):
    positive current = torque toward INCREASING position
    negative current = torque toward DECREASING position

Story of one "open" movement, starting from a closed hand (e.g. position = -1200):
  1. plan_move():  start <= CLOSED_THRESHOLD (-500) -> direction = +1, target = min(start + 1200, 0)
  2. SOFT HOLD:    goal position = where the finger is now, current limit is low (SOFT_CURRENT, ~161 mA).
                   The motor behaves like a light spring that keeps the finger where it is.
  3. BASELINE:     we measure the "resting" current (mean) and its noise (std) for 0.5 s.
  4. WAIT:         the patient tries to open the finger, i.e. pushes it toward +direction.
                   The motor resists this push (it wants to stay at the hold position), so it produces
                   torque in the opposite direction -> the measured current goes NEGATIVE.
                   => "push toward opening" = current drops below (baseline - threshold).
  5. INTENT:       once the push is detected, we look at how strong it was for 0.1 s.
                   Strong push -> fast assist, gentle push -> slow assist.
  6. ASSIST:       we command the goal position = target (open) with a higher current limit and the
                   chosen speed. The motor now moves the finger by itself.
  7. HOLD:         target reached -> hold with HOLD_CURRENT, rest, then the next cycle starts
                   (now the finger is open, so the next movement will be CLOSING).
=====================================================================================================
"""
import itertools
import time
from datetime import datetime

import matplotlib
SHOW_PLOT = False            # True = also open a window at the end (blocks until you close it)
if not SHOW_PLOT:
    matplotlib.use("Agg")    # "Agg" = draw to a file only, no window (works without a display)
import matplotlib.pyplot as plt

from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

# ------------------------- SETTINGS -------------------------
# Conversion factors from the motor's raw units to physical units (depend on the Dynamixel model).
CURRENT_UNIT_MA = 2.69      # mA per raw current unit
VELOCITY_UNIT_RPM = 0.229   # rpm per raw velocity unit

PORT = "/dev/cu.usbserial-FT763G3X"
BAUDRATE = 57600
ID = 1

# --- movement ---
MOVE_DISTANCE = 1200     # ticks to open/close (one full stroke)
MAX_POSITION = 0         # safety limit: never command or allow beyond this (= fully OPEN)
MIN_POSITION = -1200     # safety limit on the other side (= fully CLOSED)
CLOSED_THRESHOLD = -500  # start position at or below this = exo is CLOSED -> the movement is opening
CYCLES = 3               # number of movements (0 = repeat until Ctrl+C)
REST_TIME = 1.0          # seconds to hold after each movement before going back to the soft hold

# --- compliant (waiting) phase ---
SOFT_CURRENT = 40        # current limit while waiting (1 unit = 2.69 mA) -> ~161 mA. Light spring.
                         # The lower it is, the easier it is for the patient to move the finger by hand,
                         # but also the less room the current has to change (see the WARNING in run_cycle).
CURRENT_TRIGGER_MA = 5  # minimum change of the current vs rest that counts as a push [mA]
CURRENT_TRIGGER = CURRENT_TRIGGER_MA / CURRENT_UNIT_MA   # same, in raw units (the code works in raw units)
NOISE_K = 3.0            # noise weight: threshold is at least NOISE_K * (noise of the FILTERED current). Lower = more sensitive
                         # -> if the sensor is noisy, the threshold automatically becomes higher
FILTER_ALPHA = 0.6       # EMA filter on the current (1 = no filtering, smaller = smoother but slower)
BASELINE_ALPHA = 0.002   # speed of the baseline drift tracking while idle (0 = off)
MIN_DISPLACEMENT = 3     # ticks toward the target: fallback trigger (current changed + finger moved)
LIMIT_MARGIN = 50        # ticks beyond the limits (or beyond the start, if it started outside) before aborting
N_CONSEC = 3             # consecutive samples above the threshold (rejects noise spikes)
CALIB_TIME = 0.5         # seconds to measure the resting baseline (do NOT touch the finger)
WAIT_TIMEOUT = 120       # seconds: give up waiting for a push

# --- FIX for "the 2nd movement never triggers" ---
# Right after a movement the current is still settling (cable/finger relaxation, stiction). Measuring the
# baseline during that transient gives (a) an offset baseline and (b) an inflated noise value -> a much
# higher threshold. The three settings below remove both problems.
SETTLE_WINDOW = 0.3      # seconds per window used to check that the current has stopped drifting
SETTLE_MAX_TIME = 3.0    # max seconds to wait for the current to settle before measuring anyway
SETTLE_DELTA_MA = 5      # settled when the mean current of two consecutive windows differs by less than this [mA]
SETTLE_POS_TICKS = 2     # ...and the finger moved less than this many ticks inside the window
THRESHOLD_MAX_MA = 15    # the trigger threshold is never larger than this [mA] (noise can't inflate it forever)
NOISE_TRACK_ALPHA = 0.005  # live noise estimate while waiting (0 = off). ~0.005 -> adapts over about 1 s;
                           # the threshold can go DOWN once the settling noise has died out
PUSH_TICKS = 10          # offset-independent trigger: finger pushed this many ticks toward the target (0 = off)

# --- intent measurement: push strength -> assist speed ---
INTENT_WINDOW = 0.1      # seconds of push measured after the trigger
SPEED_MIN = 10           # assist speed for a gentle push (1 unit = 0.229 rpm) -> ~2.3 rpm
SPEED_MAX = 40           # assist speed for a strong push                       -> ~9.2 rpm
SPEED_GAIN = 1.0         # speed units per current unit of push above the threshold
                         # (raw units on both sides: 1 raw current unit = 2.69 mA, 1 speed unit = 0.229 rpm)

# --- assist phase ---
ASSIST_CURRENT = 150     # force limit while helping (150 units ~ 404 mA)
ASSIST_ACCEL = 5         # Profile Acceleration: smaller = gentler start (0 = instant, no ramp)
TOLERANCE = 15           # ticks: "target reached"
TIMEOUT = 15             # seconds: stop the assist if the target is not reached
HOLD_CURRENT = 100       # current limit while holding the target

# --- stall / resistance protection ---
STALL_TIME = 1.0         # seconds without progress...
STALL_TICKS = 5          # ...(progress = moved at least this many ticks)...
STALL_CURRENT_FRAC = 0.8 # ...while the current is above this fraction of ASSIST_CURRENT -> stop

# --- safety monitor ---
TEMP_LIMIT = 60          # degrees C: abort above this
VOLT_MIN = 11.0          # volts: warn below this (supply sagging)
MONITOR_PERIOD = 1.0     # seconds between checks

WRITE_TRIES = 5          # how many times a write to the motor is retried before giving up

# ------------------------- ADDRESSES -------------------------
# Control-table addresses of the Dynamixel (X-series, protocol 2.0).
ADDR_OPERATING_MODE   = 11
ADDR_TORQUE_ENABLE    = 64
ADDR_HARDWARE_ERROR   = 70
ADDR_GOAL_CURRENT     = 102   # this is the current LIMIT in mode 5 (current-based position control)
ADDR_PROFILE_ACCEL    = 108
ADDR_PROFILE_VELOCITY = 112   # max speed of the move (the "assist speed" chosen by the intent phase)
ADDR_GOAL_POSITION    = 116
ADDR_PRESENT_CURRENT  = 126   # 2 bytes; velocity (4) follows at 128, position (4) at 132
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_VOLTAGE  = 144   # 2 bytes (0.1 V); temperature (1 byte, deg C) follows at 146

port = PortHandler(PORT)
pk = PacketHandler(2.0)

# phases (used for the log and the plot)
S_WAIT, S_INTENT, S_ASSIST, S_HOLD = 0, 1, 2, 3
STATE_NAMES = {S_WAIT: "wait", S_INTENT: "intent", S_ASSIST: "assist", S_HOLD: "hold"}

# ------------------------- DATA LOG -------------------------
log_t, log_pos, log_cur, log_vel, log_state = [], [], [], [], []
cycles = []              # one dict per movement: t_start, t_trigger, t_end, baseline_mA, thr_mA, target, speed
t_zero = None            # time of the start of the session: all logged times are relative to this
state = S_WAIT           # current phase, stored together with every logged sample
_saved = False           # True once the plot/CSV has been written (avoids saving twice)
_alert_reported = False  # the "hardware alert" warning is printed only once
_last_monitor = 0.0
_volt_warned = False


# ------------------------- LOW-LEVEL HELPERS -------------------------
def write_verified(address, value, n_bytes, tries=WRITE_TRIES, quiet=False):
    """Write with retries. Returns True only if the motor acknowledged without error."""
    fn = {1: pk.write1ByteTxRx, 2: pk.write2ByteTxRx, 4: pk.write4ByteTxRx}[n_bytes]
    last = ""
    for _ in range(tries):
        result, error = fn(port, ID, address, value)
        # (error & 0x7F) ignores bit 7 = "hardware alert" flag; only real packet errors count here
        if result == COMM_SUCCESS and (error & 0x7F) == 0:
            return True
        last = pk.getTxRxResult(result) if result != COMM_SUCCESS else pk.getRxPacketError(error & 0x7F)
        time.sleep(0.01)
    if not quiet:
        print(f"WRITE FAILED (address {address}) after {tries} tries: {last}")
    return False


def read_raw(address, length, tries=3):
    """Read bytes. Returns a list of bytes, or None. Never raises on error packets."""
    for _ in range(tries):
        data, result, error = pk.readTxRx(port, ID, address, length)
        if result == COMM_SUCCESS and (error & 0x7F) == 0 and len(data) == length:
            return data
        time.sleep(0.005)
    return None


def read_position():
    """Present position in ticks (signed 32-bit), or None on a communication error."""
    data = read_raw(ADDR_PRESENT_POSITION, 4)
    if data is None:
        return None
    return int.from_bytes(bytes(data), "little", signed=True)


def set_goal_verified(goal, tries=10):
    """Write the goal position and read it back until it matches (or give up).
    Important because a lost packet here would silently leave the finger without the right goal."""
    for _ in range(tries):
        # "& 0xFFFFFFFF" converts a negative position to its 32-bit two's-complement form for the packet
        if write_verified(ADDR_GOAL_POSITION, goal & 0xFFFFFFFF, 4, tries=1, quiet=True):
            gp = read_raw(ADDR_GOAL_POSITION, 4)
            if gp is not None and int.from_bytes(bytes(gp), "little", signed=True) == goal:
                return True
        time.sleep(0.02)
    return False


def set_state(s):
    """Change the current phase (so the following log samples are tagged with it)."""
    global state
    state = s


def sample():
    """One packet: present current + velocity + position (10 bytes from address 126).
    Logs the sample and returns (position, current_raw), or None on a communication error.
    NOTE: the returned current is RAW and SIGNED (units of 2.69 mA); the log stores it in mA."""
    global _alert_reported
    data, result, error = pk.readTxRx(port, ID, ADDR_PRESENT_CURRENT, 10)
    if result != COMM_SUCCESS or (error & 0x7F) != 0 or len(data) != 10:
        return None
    if (error & 0x80) and not _alert_reported:
        _alert_reported = True
        print("WARNING: motor raised a HARDWARE ALERT flag")
    cur = int.from_bytes(bytes(data[0:2]), "little", signed=True)   # signed: sign = torque direction
    vel = int.from_bytes(bytes(data[2:6]), "little", signed=True)
    pos = int.from_bytes(bytes(data[6:10]), "little", signed=True)
    log_t.append(time.time() - t_zero)
    log_pos.append(pos)
    log_cur.append(cur * CURRENT_UNIT_MA)
    log_vel.append(vel * VELOCITY_UNIT_RPM)
    log_state.append(state)
    return pos, cur


def hardware_error_status():
    """Hardware error register (0 = no error), or None if it could not be read."""
    data = read_raw(ADDR_HARDWARE_ERROR, 1)
    return None if data is None else data[0]


def describe_hw_error(hw):
    """Human-readable names of the bits set in the hardware error register."""
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


def monitor():
    """Temperature / voltage check, at most once per MONITOR_PERIOD. Returns False if the session must stop."""
    global _last_monitor, _volt_warned
    now = time.time()
    if now - _last_monitor < MONITOR_PERIOD:     # rate-limit: don't slow down the main loop
        return True
    _last_monitor = now
    data = read_raw(ADDR_PRESENT_VOLTAGE, 3, tries=1)
    if data is None:
        return True
    volt = int.from_bytes(bytes(data[0:2]), "little") / 10.0   # raw unit is 0.1 V
    temp = data[2]
    if temp >= TEMP_LIMIT:
        print(f"MOTOR TOO HOT ({temp} C >= {TEMP_LIMIT} C) -> stopping")
        return False
    if volt < VOLT_MIN and not _volt_warned:
        _volt_warned = True
        print(f"WARNING: input voltage is low ({volt:.1f} V < {VOLT_MIN} V) - check the power supply")
    return True


def release_motor():
    """Torque OFF no matter what state the port is in (also after Ctrl+C mid-transaction)."""
    port.is_using = False            # clear the lock left by an interrupted transaction
    try:
        port.ser.reset_input_buffer()
    except Exception:
        pass
    ok = write_verified(ADDR_TORQUE_ENABLE, 0, 1, tries=10)
    if not ok:
        print("!!! COULD NOT TURN TORQUE OFF - power-cycle the motor !!!")
    port.closePort()


# ------------------------- PLOT / SAVE -------------------------
def save_plot():
    """Draw the figure once, at the end, and save it (PNG + CSV)."""
    global _saved
    if _saved or not log_t:
        return
    t_end_all = log_t[-1]
    fig, (ax_pos, ax_cur) = plt.subplots(2, 1, sharex=True, figsize=(11, 7))

    # top plot: position, with the two safety limits
    ax_pos.plot(log_t, log_pos, color="tab:blue", label="position")
    ax_pos.axhline(MAX_POSITION, color="black", linewidth=1.5, label="MAX_POSITION")
    ax_pos.axhline(MIN_POSITION, color="black", linewidth=1.5, linestyle="--", label="MIN_POSITION")
    ax_pos.set_ylabel("Position [ticks]")

    # bottom plot: motor current, with the soft-hold current limit (+/-)
    ax_cur.plot(log_t, log_cur, color="tab:red", label="present current")
    ax_cur.axhline(0, color="gray", linewidth=0.8)
    soft_mA = SOFT_CURRENT * CURRENT_UNIT_MA
    ax_cur.axhline(soft_mA, color="purple", linestyle="--", linewidth=1, label="SOFT_CURRENT limit")
    ax_cur.axhline(-soft_mA, color="purple", linestyle="--", linewidth=1)
    ax_cur.set_ylabel("Current [mA]")
    ax_cur.set_xlabel("Time [s]")

    # shaded regions per movement: cyan = waiting, green = assist, orange dashed = trigger moment
    first = True   # only the first movement adds legend entries (avoids duplicates)
    for i, c in enumerate(cycles):
        t0 = c["t_start"]
        t1 = c.get("t_trigger")
        t2 = c.get("t_end")
        wait_end = t1 if t1 is not None else t_end_all
        for ax in (ax_pos, ax_cur):
            ax.axvspan(t0, wait_end, color="tab:cyan", alpha=0.15, label="soft hold (waiting)" if first and ax is ax_pos else None)
            if t1 is not None:
                ax.axvline(t1, color="orange", linestyle="--", linewidth=1)
                if t2 is not None:
                    ax.axvspan(t1, t2, color="tab:green", alpha=0.12, label="assist" if first and ax is ax_pos else None)
                    ax.axvline(t2, color="green", linestyle="--", linewidth=1)
        if c.get("baseline_mA") is not None:
            # dotted orange lines = baseline +/- threshold: the band the current must leave to trigger
            b, th = c["baseline_mA"], c["thr_mA"]
            ax_cur.hlines([b + th, b - th], t0, wait_end, colors="orange", linestyles=":",
                          label="trigger thresholds" if first else None)
        if c.get("target") is not None:
            ax_pos.hlines(c["target"], t0, t2 if t2 is not None else t_end_all, colors="green", linestyles=":",
                          label="target" if first else None)
        ax_pos.text(t0, ax_pos.get_ylim()[1], f" #{i + 1}", va="top")   # movement number
        first = False

    for ax in (ax_pos, ax_cur):
        ax.grid(True)
        ax.legend(loc="best")
    fig.suptitle("Exofinger V2: position and current")
    fig.tight_layout()

    name = datetime.now().strftime("exofinger_v2_%Y%m%d_%H%M%S")
    fig.savefig(name + ".png", dpi=150)
    with open(name + ".csv", "w") as f:
        f.write("time_s,position_ticks,current_mA,velocity_rpm,phase\n")
        for t, p, c, v, s in zip(log_t, log_pos, log_cur, log_vel, log_state):
            f.write(f"{t:.4f},{p},{c:.2f},{v:.2f},{STATE_NAMES[s]}\n")
    print(f"Saved {name}.png and {name}.csv")
    _saved = True
    if SHOW_PLOT:
        plt.show()
    else:
        plt.close(fig)


# ------------------------- CONTROL -------------------------
def plan_move(start):
    """Direction (+1 open / -1 close) and target from the starting position.

    The exo does not need to be told "open" or "close": it looks at where the finger is.
      - at or below CLOSED_THRESHOLD (-500)  -> hand is CLOSED -> OPEN it:  direction = +1
            target = start + 1200, but never above MAX_POSITION (0)
      - otherwise                            -> hand is OPEN   -> CLOSE it: direction = -1
            target = start - 1200, but never below MIN_POSITION (-1200)
    Example: start = -1200 (closed)  ->  target = min(-1200 + 1200, 0) = 0 (open).
    """
    if start <= CLOSED_THRESHOLD:                  # currently CLOSED -> open
        print("Currently in CLOSED position -> will OPEN")
        return +1, min(start + MOVE_DISTANCE, MAX_POSITION)
    print("Currently in OPEN position -> will CLOSE")   # currently OPEN -> close
    return -1, max(start - MOVE_DISTANCE, MIN_POSITION)


def calibrate_baseline():
    """Mean and standard deviation of the resting (signed) current in raw units, or None.

    Measured while the motor holds the finger softly and the patient is NOT pushing.
      mean (baseline) = the "zero" of the push sensor: the current the motor needs just to hold the
                        finger in place (gravity, cable friction, spring effect of the soft hold, ...)
      std  (sigma)    = how noisy the current signal is at rest; used to set the trigger threshold
    """
    vals = []
    t_cal = time.time()
    while time.time() - t_cal < CALIB_TIME:
        s = sample()
        if s is not None:
            vals.append(s[1])
    if not vals:
        return None
    mean = sum(vals) / len(vals)
    # FIX: noise from the DIFFERENCES between consecutive samples instead of the std around the mean.
    # A slow drift (current still settling) makes the normal std large, but barely affects the
    # sample-to-sample differences. For white noise, std(diff) = sqrt(2) * sigma, hence the / sqrt(2).
    if len(vals) >= 3:
        diffs = [b - a for a, b in zip(vals, vals[1:])]
        std = (sum(x * x for x in diffs) / len(diffs)) ** 0.5 / (2 ** 0.5)
    else:
        std = 0.0
    return mean, std


def wait_until_settled():
    """FIX: wait until the resting current has stopped drifting (and the finger is still).

    Compares the mean current of consecutive SETTLE_WINDOW-long windows. Settled when the mean changed
    by less than SETTLE_DELTA_MA and the finger moved less than SETTLE_POS_TICKS in the window.
    Gives up after SETTLE_MAX_TIME and measures anyway. Returns True if settled."""
    delta = SETTLE_DELTA_MA / CURRENT_UNIT_MA          # in raw units
    t_start = time.time()
    prev_mean = None
    while time.time() - t_start < SETTLE_MAX_TIME:
        vals, poss = [], []
        t_w = time.time()
        while time.time() - t_w < SETTLE_WINDOW:
            s = sample()
            if s is not None:
                poss.append(s[0])
                vals.append(s[1])
        if not vals:
            continue
        mean = sum(vals) / len(vals)
        if prev_mean is not None and abs(mean - prev_mean) <= delta and (max(poss) - min(poss)) <= SETTLE_POS_TICKS:
            return True
        prev_mean = mean
    print("Note: the current did not fully settle - measuring the baseline anyway.")
    return False


# The current is compared with the threshold AFTER the EMA filter, which already reduces the noise.
# For white noise the filtered std is sigma * sqrt(alpha / (2 - alpha)) (= 0.58 * sigma for alpha = 0.5).
# FIX: the old code multiplied the RAW noise by NOISE_K, so the real margin was ~7 sigma instead of 4.
FILTER_NOISE_FACTOR = (FILTER_ALPHA / (2 - FILTER_ALPHA)) ** 0.5


def threshold_from_noise(sigma):
    """Trigger threshold in raw units from the raw-current noise sigma (raw units).
    max( minimum threshold , NOISE_K * noise of the filtered current ), capped at THRESHOLD_MAX_MA."""
    noise_part = NOISE_K * sigma * FILTER_NOISE_FACTOR
    return max(CURRENT_TRIGGER, min(noise_part, THRESHOLD_MAX_MA / CURRENT_UNIT_MA))


def run_cycle(n):
    """One full movement. Returns 'reached', 'partial' or 'stalled' (session continues),
    or 'idle', 'stop', 'abort' (session ends)."""
    rec = {"t_start": time.time() - t_zero}   # per-movement record, used by the plot
    cycles.append(rec)

    start = read_position()
    if start is None:
        print("Cannot read the present position.")
        return "abort"
    direction, target = plan_move(start)      # direction = +1 when the hand is closed (we will OPEN)
    rec["target"] = target
    if abs(target - start) < 3 * MIN_DISPLACEMENT:
        print("Already at/near the limit in that direction, nothing to do.")
        return "stop"
    print(f"\n--- Movement {n}: start = {start}, target = {target} ---")

    # ---------- soft hold at the present position ----------
    # Goal = where the finger is NOW, with a low current limit: the motor acts like a weak spring that
    # keeps the finger in place but lets the patient move it. When the patient pushes, the motor's
    # current changes -> that change is our "intent" signal.
    set_state(S_WAIT)
    if not set_goal_verified(start):                 # goal first (no jerk), then the low current limit
        print("Could not set the hold position -> aborting")
        return "abort"
    if not write_verified(ADDR_GOAL_CURRENT, SOFT_CURRENT, 2):
        print("Could not set the soft current limit -> aborting")
        return "abort"
    hold_pos = start     # reference position: "moved" below is measured relative to this
    rec["t_start"] = time.time() - t_zero

    # ---------- settle, then baseline ----------
    # FIX: let the transient after the previous movement die out BEFORE measuring the baseline.
    print("Waiting for the current to settle - do NOT touch the finger...")
    wait_until_settled()
    print(f"Measuring resting current for {CALIB_TIME} s - do NOT touch the finger...")
    cal = calibrate_baseline()
    if cal is None:
        print("No samples during calibration -> aborting")
        return "abort"
    baseline, sigma = cal # baseline: the resting current measured at the start of the cycle
    # Adaptive threshold: at least CURRENT_TRIGGER (15 mA), but if the signal is noisy, at least
    # NOISE_K (4) standard deviations. Both are in raw current units.
    # FIX: the noise-based part is capped at THRESHOLD_MAX_MA so it can't grow out of reach of a gentle push.
    thr = threshold_from_noise(sigma)
    rec["baseline_mA"] = baseline * CURRENT_UNIT_MA
    rec["thr_mA"] = thr * CURRENT_UNIT_MA
    print(f"baseline current = {baseline * CURRENT_UNIT_MA:.0f} mA, noise = {sigma * CURRENT_UNIT_MA:.1f} mA, "
          f"trigger when it changes by {thr * CURRENT_UNIT_MA:.0f} mA")
    # The current can never exceed SOFT_CURRENT (it is the limit). If baseline + threshold is already
    # beyond that limit, the current physically cannot reach the trigger level.
    if abs(baseline) + thr >= SOFT_CURRENT:
        print(f"WARNING: |baseline| + threshold ({abs(baseline) + thr:.0f}) >= SOFT_CURRENT ({SOFT_CURRENT}): "
              "the trigger may never fire. Lower the threshold or raise SOFT_CURRENT.")
    print("Soft hold active. Push the finger toward the target...")

    # =====================================================================================
    # 1) WAIT for a push  (this is where the exo decides "the patient wants to move")
    # =====================================================================================
    t_wait = time.time()
    count = 0            # how many CONSECUTIVE samples have satisfied the trigger condition
    t_diag = 0.0         # time of the last diagnostic print (to avoid spamming the console)
    cur_f = baseline     # filtered current, starts at the baseline so there is no initial transient
    peak = 0.0           # FIX: biggest push toward the target seen in the last second (for live feedback)
    t_live = time.time() # FIX: time of the last live-feedback print
    prev_cur = None      # FIX: previous raw current sample (for the live noise estimate)
    var_live = sigma ** 2  # FIX: running estimate of the resting noise variance (raw units^2)
    while True:
        if time.time() - t_wait > WAIT_TIMEOUT:
            print("No push detected, giving up.")
            return "idle"
        if not monitor():
            return "abort"
        s = sample()
        if s is None:
            continue
        pos, cur = s

        # Low-pass (EMA) filter:  cur_f = cur_f + alpha * (new - cur_f)
        # With alpha = 0.5 each new sample counts for half. This removes single-sample spikes.
        cur_f += FILTER_ALPHA * (cur - cur_f)
        # cur_f = filtered current
        dd = 0 if prev_cur is None else cur - prev_cur   # FIX: sample-to-sample change (noise, not drift)
        prev_cur = cur

        # How far the finger has been pushed from the hold position, IN THE DIRECTION OF THE MOVEMENT.
        # Opening (direction = +1): moved = pos - hold_pos        -> positive if pushed toward open
        # Closing (direction = -1): moved = -(pos - hold_pos)     -> positive if pushed toward close
        moved = (pos - hold_pos) * direction

        # Signed change of the FILTERED current relative to rest.
        d = cur_f - baseline # the push signal

        # --- THE TRIGGER, explained for OPENING (direction = +1) ---
        # The motor is holding the finger. The patient pushes toward open (+). The motor opposes that
        # push, so it produces torque toward closed (-) and the current goes NEGATIVE:
        #       d <= -thr   <=>   (-direction * d) >= thr      (with direction = +1: -d >= thr)
        # When CLOSING (direction = -1) it is the opposite: the current goes POSITIVE (d >= +thr),
        # and the same expression (-direction * d) >= thr still describes it. One formula, both directions.
        by_current = (-direction * d) >= thr

        # Fallback trigger: the current changed by at least thr in EITHER sign (abs) AND the finger has
        # actually moved at least MIN_DISPLACEMENT ticks toward the target. Useful when the sign of the
        # current is not as expected (different friction, different cable routing, ...).
        by_motion = abs(d) >= thr and moved >= MIN_DISPLACEMENT   # fallback

        # FIX: offset-independent trigger. If the finger itself has been pushed PUSH_TICKS toward the
        # target, that is a push regardless of what the current baseline/offset looks like.
        by_displacement = PUSH_TICKS > 0 and moved >= PUSH_TICKS

        # FIX: "toward" = push toward the target in mA-independent raw units (positive = right direction).
        # Live feedback once per second so you can SEE how close the push gets to the threshold.
        toward = -direction * d
        peak = max(peak, toward)
        if time.time() - t_live > 1.0:
            t_live = time.time()
            if peak > 0.3 * thr:
                print(f"  push toward target (last 1 s peak): {peak * CURRENT_UNIT_MA:.0f} mA, "
                      f"trigger needs {thr * CURRENT_UNIT_MA:.0f} mA")
            peak = 0.0

        if by_current or by_motion or by_displacement:
            count += 1       # one more consecutive sample that looks like a push
        else:
            count = 0        # the streak is broken -> start counting again (noise spike rejected)
            if abs(d) >= thr and time.time() - t_diag > 0.5:
                # The current changed a lot but in the WRONG direction (e.g. the patient is pulling
                # the finger the other way, or a spasm). Report it at most twice per second.
                t_diag = time.time()
                print(f"current changed by {d * CURRENT_UNIT_MA:+.0f} mA but NOT toward the target "
                      f"(moved={moved} ticks; a push toward the target should make the current go "
                      f"{'negative' if direction > 0 else 'positive'})")
            # FIX: this used to be "abs(d) < 0.5*thr", so a drift AWAY from the target larger than half the
            # threshold (e.g. the current relaxing after the previous movement) was never tracked and
            # permanently masked the push. Now any drift in the wrong direction is tracked, and only a
            # (partial) push toward the target (toward >= 0.5*thr) freezes the baseline.
            if toward < 0.5 * thr and abs(pos - hold_pos) <= MIN_DISPLACEMENT:
                # Idle (no push toward the target AND the finger has not moved): the baseline drifts
                # slowly toward the present value (0.2 % of the error per sample) to follow
                # temperature / friction / relaxation changes. It is NOT updated while a push is in
                # progress, otherwise the baseline would "eat" the push signal.
                baseline += BASELINE_ALPHA * d       # slow drift tracking while idle
                if NOISE_TRACK_ALPHA > 0:
                    # FIX: live noise estimate, only while idle (a push must not raise the noise value).
                    # var(diff) = 2 * sigma^2, hence dd^2 / 2. The threshold follows it, so if the noise
                    # measured at the start was inflated, the threshold comes down after a second or so.
                    var_live += NOISE_TRACK_ALPHA * (dd * dd / 2 - var_live)
                    thr = threshold_from_noise(var_live ** 0.5)

        # N_CONSEC consecutive samples above the threshold -> real push, leave the WAIT loop.
        if count >= N_CONSEC:
            break

        # Safety: the patient (or something else) moved the finger beyond the allowed range.
        # The limits are extended by LIMIT_MARGIN, or by where we started if we started outside them.
        if pos > max(MAX_POSITION, start) + LIMIT_MARGIN or pos < min(MIN_POSITION, start) - LIMIT_MARGIN:
            print(f"Finger pushed beyond the safety limits (pos={pos}) -> aborting")
            return "abort"

    # =====================================================================================
    # 2) INTENT: measure how strongly the patient pushes  ->  decides the assist speed
    # =====================================================================================
    # Idea: the harder the patient pushes against the soft hold, the larger the current change.
    # So the size of the current change is used as a measure of "how much the patient wants to move".
    rec["t_trigger"] = time.time() - t_zero
    rec["baseline_mA"] = baseline * CURRENT_UNIT_MA   # baseline may have drifted during the wait
    rec["thr_mA"] = thr * CURRENT_UNIT_MA             # FIX: threshold may have changed during the wait
    set_state(S_INTENT)

    # Start from the push size at the moment of the trigger (d is the last value from the WAIT loop).
    # abs() because only the MAGNITUDE of the push matters here (direction was already checked).
    effort = abs(d)
    t_i = time.time()
    while time.time() - t_i < INTENT_WINDOW:      # keep measuring for 0.1 s
        s = sample()
        if s is None:
            continue
        cur_f += FILTER_ALPHA * (s[1] - cur_f)    # keep filtering with the same EMA
        # Keep the PEAK push seen during the window (the push usually keeps growing after the trigger).
        effort = max(effort, abs(cur_f - baseline))

    # Map the push strength to a speed (all in raw units):
    #       speed = SPEED_MIN + SPEED_GAIN * (effort - thr)
    #   - effort - thr   = how much the push exceeds the minimum needed to trigger (0 = just barely)
    #   - SPEED_GAIN     = how many speed units we add per raw current unit of extra push
    #   - clamped to [SPEED_MIN, SPEED_MAX] so the assist is never too slow or too fast
    # Example: thr = 6 units (~16 mA), effort = 20 units (~54 mA) -> speed = 10 + 1.0*(20-6) = 24 (~5.5 rpm)
    #   gentle push (effort ~ thr)  -> speed = 10 (slow, ~2.3 rpm)
    #   strong push (effort >= thr+30) -> speed = 40 (fast, ~9.2 rpm)
    speed = int(max(SPEED_MIN, min(SPEED_MAX, SPEED_MIN + SPEED_GAIN * (effort - thr))))
    rec["speed"] = speed
    print(f"Push detected: {effort * CURRENT_UNIT_MA:.0f} mA above rest -> assist speed {speed}")

    # =====================================================================================
    # 3) ASSIST: the motor moves the finger to the target (for a closed hand: OPENS it)
    # =====================================================================================
    # Profile Velocity = the speed cap chosen by the intent measurement above.
    if not write_verified(ADDR_PROFILE_VELOCITY, speed, 4):
        return "abort"
    # Raise the current limit from SOFT_CURRENT to ASSIST_CURRENT: now the motor is allowed to push hard.
    if not write_verified(ADDR_GOAL_CURRENT, ASSIST_CURRENT, 2):
        print("Could not raise the current limit -> aborting")
        return "abort"
    # Finally set the goal to the target: THIS is the command that makes the finger move.
    # (Profile Acceleration was set once in main(), so the speed ramps up smoothly.)
    if not set_goal_verified(target):
        print("Could not set the goal position (read-back mismatch) -> aborting")
        return "abort"
    set_state(S_ASSIST)
    print(f"goal position = {target} (verified)")

    hw = hardware_error_status()
    if hw:
        print(f"Motor shut down by hardware error {hw:#010b} ({describe_hw_error(hw)}). "
              "Check the power supply (use 12 V, not USB only).")
        return "abort"

    t0 = time.time()
    reached = stalled = aborted = False
    stop_goal = None     # if set, the position to hold at when the loop ends (instead of the target)
    last_prog_pos = read_position()   # position at the last moment we saw "progress"
    if last_prog_pos is None:
        last_prog_pos = start
    t_prog = time.time()              # time of the last progress
    while time.time() - t0 < TIMEOUT:
        s = sample()
        if s is None:
            continue
        pos, cur = s
        # Software safety limits (in addition to the target): opening must never go beyond MAX_POSITION,
        # closing never below MIN_POSITION.
        if direction > 0 and pos >= MAX_POSITION:
            stop_goal = MAX_POSITION
            print(f"MAX_POSITION reached (pos={pos}) -> holding at {MAX_POSITION}")
            break
        if direction < 0 and pos <= MIN_POSITION:
            stop_goal = MIN_POSITION
            print(f"MIN_POSITION reached (pos={pos}) -> holding at {MIN_POSITION}")
            break
        # Normal success: within TOLERANCE ticks of the target.
        if abs(pos - target) <= TOLERANCE:
            reached = True
            break
        # Stall detection: any movement of >= STALL_TICKS counts as progress and resets the timer.
        if abs(pos - last_prog_pos) >= STALL_TICKS:
            last_prog_pos = pos
            t_prog = time.time()
        # No progress for STALL_TIME AND the motor is pushing hard (>= 80 % of ASSIST_CURRENT)
        # -> the finger is blocked (spasm / obstruction / patient resisting) -> stop pushing.
        elif time.time() - t_prog > STALL_TIME and abs(cur) >= STALL_CURRENT_FRAC * ASSIST_CURRENT:
            stalled = True
            print(f"STALL: no progress for {STALL_TIME:.1f} s at high current "
                  f"({abs(cur) * CURRENT_UNIT_MA:.0f} mA) at pos={pos} -> stopping assist")
            break
        if _alert_reported:
            hw = hardware_error_status()
            if hw:
                print(f"Hardware error {hw:#010b} ({describe_hw_error(hw)}) -> stopping assist")
                aborted = True
                break
        if not monitor():
            aborted = True
            break

    # =====================================================================================
    # 4) HOLD: keep the finger where it ended up, then go back to waiting for the next cycle
    # =====================================================================================
    rec["t_end"] = time.time() - t_zero
    set_state(S_HOLD)
    if reached:
        print("Target reached -> holding position")
    else:
        if stop_goal is None:               # timeout / stall / error: hold where we are, don't keep pulling
            stop_goal = read_position()
        if stop_goal is not None:
            set_goal_verified(stop_goal)    # new goal = present position -> the motor stops pulling
        if not (stalled or aborted):
            print("Assist ended before target -> holding current position")
    # After a stall go back to the SOFT current (don't keep forcing against resistance);
    # otherwise hold more firmly with HOLD_CURRENT.
    write_verified(ADDR_GOAL_CURRENT, SOFT_CURRENT if stalled else HOLD_CURRENT, 2)

    t_r = time.time()
    while time.time() - t_r < REST_TIME:
        sample()                            # keep logging during the rest time

    if aborted:
        return "abort"
    return "reached" if reached else ("stalled" if stalled else "partial")


# ------------------------- MAIN -------------------------
def main():
    global t_zero

    if not port.openPort():
        print(f"Cannot open port {PORT}")
        return
    if not port.setBaudRate(BAUDRATE):
        print("Cannot set baud rate")
        port.closePort()
        return

    try:
        # ------------------------- SETUP -------------------------
        if not clear_hardware_error():
            return
        # Settings that can only be changed with torque OFF are written first.
        ok = (write_verified(ADDR_TORQUE_ENABLE, 0, 1)
              and write_verified(ADDR_OPERATING_MODE, 5, 1)          # 5: current-based position control
              and write_verified(ADDR_GOAL_CURRENT, SOFT_CURRENT, 2)  # LOW current limit while waiting
              and write_verified(ADDR_PROFILE_VELOCITY, SPEED_MIN, 4))
        if not ok:
            print("Setup failed (check wiring / power / baud rate / ID). Exiting.")
            return
        # Profile Acceleration = the smooth-start ramp of the assist. Not critical if it fails.
        if not write_verified(ADDR_PROFILE_ACCEL, ASSIST_ACCEL, 4):
            print("Note: could not set Profile Acceleration - continuing without the smooth start ramp.")

        t_zero = time.time()
        if not write_verified(ADDR_TORQUE_ENABLE, 1, 1):
            print("Could not enable torque -> aborting")
            return
        time.sleep(0.02)

        # ------------------------- CYCLES -------------------------
        # Each cycle = one movement. Because plan_move() looks at the position, consecutive cycles
        # automatically alternate: open, close, open, ...
        result = "stop"
        for n in (itertools.count(1) if CYCLES == 0 else range(1, CYCLES + 1)):
            result = run_cycle(n)
            if result in ("idle", "stop", "abort"):
                break

        if result in ("reached", "partial", "stalled"):
            input("Done. Holding position. Press Enter to release the motor and save the plot (or Ctrl+C)...")

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        release_motor()          # torque OFF, always
        save_plot()              # data is never lost


if __name__ == "__main__":
    main()