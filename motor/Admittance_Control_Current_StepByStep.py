"""
Exofinger V3 - CURRENT-ONLY step-follower.

No open/closed logic, no targets. The only input is the motor current:
  - the motor softly holds the finger where it is (low current limit)
  - if the patient pushes the finger, the motor opposes it -> the current moves away from its resting value
  - the sign of that change gives the direction, the motor makes one small RELATIVE step that way
  - back to the soft hold; if the push is still there the current deviates again -> next step
  - when the push stops, the current stays at its resting value -> no more steps
"""
import csv
import time
from datetime import datetime

from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

# ------------------------- SETTINGS -------------------------
PORT = "/dev/cu.usbserial-FT763G3X"
BAUDRATE = 57600
ID = 1
CURRENT_UNIT_MA = 2.69          # mA per raw current unit

# --- soft hold (waiting) ---
SOFT_CURRENT = 30               # current limit while waiting (~161 mA)
POSITION_P_GAIN = 200           # stiffness of the hold (X-series default ~800). Lower = easier to push,

# --- trigger (on the filtered current) ---
TRIGGER_MA = 8                 # minimum change vs. rest that counts as a push [mA]
THRESHOLD_MAX_MA = 20           # the noise-based threshold is never larger than this [mA]
NOISE_K = 3.0                   # threshold >= NOISE_K * noise of the filtered current
FILTER_ALPHA = 0.6              # EMA filter on the current (1 = no filter)
BASELINE_ALPHA = 0.002          # slow baseline drift tracking while idle (0 = off)
N_CONSEC = 4                    # consecutive samples above the threshold before stepping
CALIB_TIME = 0.5                # seconds to measure the resting current (don't touch the finger)

# --- one step ---
STEP_TICKS = 50                 # size of one relative step
STEP_CURRENT = 150              # current limit during a step (~404 mA)
STEP_SPEED = 60                 # Profile Velocity during a step (0.229 rpm per unit)
STEP_ACCEL = 10                  # Profile Acceleration (smooth start)
STEP_TOL = 5                    # ticks: step considered finished
STEP_TIMEOUT = 1.0              # seconds: max duration of one step
SETTLE_AFTER_STEP = 0.1         # seconds of soft hold after a step before looking at the current again

# --- sign / safety ---
INVERT_DIRECTION = False        # flip if the motor steps against the push instead of with it
MAX_TRAVEL = 1000               # optional safety: max ticks away from the start position (None = off)
TEMP_LIMIT = 60                 # deg C, stop above this
WRITE_TRIES = 5

# ------------------------- ADDRESSES (X-series, protocol 2.0) -------------------------
ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_CURRENT = 102         # current LIMIT in mode 5
ADDR_PROFILE_ACCEL = 108
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_CURRENT = 126      # 2 bytes; velocity (4) at 128, position (4) at 132
ADDR_PRESENT_TEMPERATURE = 146
ADDR_POSITION_P_GAIN = 84       # 2 bytes

port = PortHandler(PORT)
pk = PacketHandler(2.0)

FILTER_NOISE_FACTOR = (FILTER_ALPHA / (2 - FILTER_ALPHA)) ** 0.5
TRIGGER = TRIGGER_MA / CURRENT_UNIT_MA
THRESHOLD_MAX = THRESHOLD_MAX_MA / CURRENT_UNIT_MA

log = []        # (time, position, current_mA, phase)
phase = "wait"
t_zero = 0.0


# ------------------------- LOW LEVEL -------------------------
def write(address, value, n_bytes, tries=WRITE_TRIES):
    fn = {1: pk.write1ByteTxRx, 2: pk.write2ByteTxRx, 4: pk.write4ByteTxRx}[n_bytes]
    for _ in range(tries):
        result, error = fn(port, ID, address, value)
        if result == COMM_SUCCESS and (error & 0x7F) == 0:
            return True
        time.sleep(0.01)
    print(f"WRITE FAILED (address {address})")
    return False


def sample():
    """Returns (position, current_raw_signed) and logs it, or None on a communication error."""
    data, result, error = pk.readTxRx(port, ID, ADDR_PRESENT_CURRENT, 10)
    if result != COMM_SUCCESS or (error & 0x7F) != 0 or len(data) != 10:
        return None
    cur = int.from_bytes(bytes(data[0:2]), "little", signed=True)
    pos = int.from_bytes(bytes(data[6:10]), "little", signed=True)
    log.append((time.time() - t_zero, pos, cur * CURRENT_UNIT_MA, phase))
    return pos, cur


def set_goal(goal):
    return write(ADDR_GOAL_POSITION, goal & 0xFFFFFFFF, 4)


def hold_here(pos):
    """Soft hold at 'pos': goal first (no jerk), then the low current limit."""
    return set_goal(pos) and write(ADDR_GOAL_CURRENT, SOFT_CURRENT, 2)


def too_hot():
    data, result, error = pk.readTxRx(port, ID, ADDR_PRESENT_TEMPERATURE, 1)
    return result == COMM_SUCCESS and len(data) == 1 and data[0] >= TEMP_LIMIT


# ------------------------- LOGIC -------------------------
def calibrate():
    """Resting current (mean) and its noise (from sample-to-sample differences), raw units."""
    vals = []
    t = time.time()
    while time.time() - t < CALIB_TIME:
        s = sample()
        if s:
            vals.append(s[1])
    if len(vals) < 3:
        return None
    mean = sum(vals) / len(vals)
    diffs = [b - a for a, b in zip(vals, vals[1:])]
    sigma = (sum(x * x for x in diffs) / len(diffs)) ** 0.5 / 2 ** 0.5
    return mean, sigma


def do_step(pos, direction):
    """One RELATIVE step from the present position, then soft hold at wherever the finger ended up.
    Returns the position after the step."""
    global phase
    phase = "step"
    goal = pos + direction * STEP_TICKS
    write(ADDR_GOAL_CURRENT, STEP_CURRENT, 2)     # allow more force
    set_goal(goal)                                # this makes the motor move
    t0 = time.time()
    last = pos
    while time.time() - t0 < STEP_TIMEOUT:
        s = sample()
        if s:
            last = s[0]
            if abs(last - goal) <= STEP_TOL:
                break
    phase = "settle"
    hold_here(last)                               # goal = present position, low current again
    t0 = time.time()
    while time.time() - t0 < SETTLE_AFTER_STEP:   # let the current transient die out
        s = sample()
        if s:
            last = s[0]
    phase = "wait"
    return last


def run():
    global phase
    s = None
    while s is None:
        s = sample()
    pos = origin = s[0]
    hold_here(pos)

    print("Measuring resting current - do NOT touch the finger...")
    cal = None
    while cal is None:
        cal = calibrate()
    baseline, sigma = cal
    thr = max(TRIGGER, min(NOISE_K * sigma * FILTER_NOISE_FACTOR, THRESHOLD_MAX))
    print(f"baseline = {baseline * CURRENT_UNIT_MA:.0f} mA, noise = {sigma * CURRENT_UNIT_MA:.1f} mA, "
          f"threshold = {thr * CURRENT_UNIT_MA:.0f} mA")
    print("Soft hold active. Push the finger in either direction (Ctrl+C to stop)...")

    cur_f = baseline
    count = 0
    t_check = time.time()
    while True:
        if time.time() - t_check > 1.0:
            t_check = time.time()
            if too_hot():
                print("MOTOR TOO HOT -> stopping")
                return
        s = sample()
        if s is None:
            continue
        pos, cur = s
        cur_f += FILTER_ALPHA * (cur - cur_f)
        d = cur_f - baseline                      # signed change of the filtered current vs. rest

        if abs(d) < thr:                          # no push: idle
            count = 0
            baseline += BASELINE_ALPHA * d        # slow drift tracking (never while pushing)
            continue

        count += 1                                # push candidate
        if count < N_CONSEC:
            continue

        # The motor OPPOSES the push, so the current has the opposite sign of the push:
        # current negative -> push toward +ticks -> step +; current positive -> step -.
        direction = -1 if d > 0 else +1
        if INVERT_DIRECTION:
            direction = -direction
        if MAX_TRAVEL is not None and abs(pos + direction * STEP_TICKS - origin) > MAX_TRAVEL:
            print("MAX_TRAVEL reached in that direction -> no step")
            count = 0
            continue

        pos = do_step(pos, direction)
        print(f"step {'+' if direction > 0 else '-'}{STEP_TICKS}  (current change {d * CURRENT_UNIT_MA:+.0f} mA)  pos={pos}")
        count = 0
        s = sample()                              # restart the filter from the settled current
        if s:
            cur_f = s[1]


def save():
    if not log:
        return
    name = datetime.now().strftime("exofinger_v3_%Y%m%d_%H%M%S")
    with open(name + ".csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["time_s", "position_ticks", "current_mA", "phase"])
        w.writerows(log)
    print(f"Saved {name}.csv")


def main():
    global t_zero
    if not port.openPort() or not port.setBaudRate(BAUDRATE):
        print(f"Cannot open {PORT}")
        return
    try:
        ok = (write(ADDR_TORQUE_ENABLE, 0, 1)
              and write(ADDR_OPERATING_MODE, 5, 1)             # current-based position control
              and write(ADDR_POSITION_P_GAIN, POSITION_P_GAIN, 2)   # soft spring stiffness
              and write(ADDR_GOAL_CURRENT, SOFT_CURRENT, 2)
              and write(ADDR_PROFILE_VELOCITY, STEP_SPEED, 4)
              and write(ADDR_PROFILE_ACCEL, STEP_ACCEL, 4))
        if not ok:
            print("Setup failed.")
            return
        t_zero = time.time()
        if not write(ADDR_TORQUE_ENABLE, 1, 1):
            return
        time.sleep(0.02)
        run()
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        port.is_using = False
        write(ADDR_TORQUE_ENABLE, 0, 1, tries=10)      # torque OFF, always
        port.closePort()
        save()


if __name__ == "__main__":
    main()
