"""
Assist when the patient moves - motor FREE (torque off) while waiting, then it finishes the movement.
Version 5: robust communication.
  - every write is verified and retried
  - reads never crash (error packets are reported, not turned into IndexError)
  - goal position is written BEFORE torque is enabled (no stale-goal jump, fewer packets at torque-on)
  - hardware errors are detected, reported and cleared (reboot) at start
  - torque-off at the end is guaranteed, even after Ctrl+C
  - no blocking plt.show(): the plot is saved as PNG + CSV
"""
import time
from datetime import datetime

import matplotlib
SHOW_PLOT = False            # True = also open a window at the end (blocks until you close it)
if not SHOW_PLOT:
    matplotlib.use("Agg")
import matplotlib.pyplot as plt

from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

# ------------------------- SETTINGS -------------------------
PORT = "/dev/cu.usbserial-FT763G3X"
BAUDRATE = 57600
ID = 1

MOVE_DISTANCE = 1200   # ticks to open/close
MAX_POSITION = 0       # safety limit: never command or allow beyond this
MIN_POSITION = -1200
DELTA = 100
TRIGGER = 30           # ticks the patient must move before the motor helps
ASSIST_CURRENT = 150   # force limit while helping (1 unit = 2.69 mA)
ASSIST_SPEED = 20      # max speed while helping (1 unit = 0.229 rpm)
TOLERANCE = 15         # ticks: "target reached"
TIMEOUT = 15           # seconds: stop the assist if the target is not reached
FREE_TIMEOUT = 120     # seconds: give up waiting for the patient to move
HOLD_CURRENT = 100
CURRENT_UNIT_MA = 2.69
WRITE_TRIES = 5

# ------------------------- ADDRESSES -------------------------
ADDR_OPERATING_MODE   = 11
ADDR_TORQUE_ENABLE    = 64
ADDR_HARDWARE_ERROR   = 70
ADDR_GOAL_CURRENT     = 102
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION    = 116
ADDR_PRESENT_CURRENT  = 126   # 2 bytes; velocity (4) follows at 128, position (4) at 132
ADDR_PRESENT_POSITION = 132

port = PortHandler(PORT)
pk = PacketHandler(2.0)

# ------------------------- DATA LOG -------------------------
log_t, log_pos, log_cur = [], [], []
t_zero = None
t_torque_on = None   # time when torque was enabled (end of the free phase)
t_assist_end = None
_saved = False
_alert_reported = False


# ------------------------- LOW-LEVEL HELPERS -------------------------
def write_verified(address, value, n_bytes, tries=WRITE_TRIES, quiet=False):
    """Write with retries. Returns True only if the motor acknowledged without error.
    quiet=True suppresses the failure message (the caller handles retries/reporting)."""
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
    """Read bytes. Returns a list of bytes, or None. Never raises on error packets."""
    for _ in range(tries):
        data, result, error = pk.readTxRx(port, ID, address, length)
        if result == COMM_SUCCESS and (error & 0x7F) == 0 and len(data) == length:
            return data
        time.sleep(0.005)
    return None


def to_signed(value, bits):
    return value - (1 << bits) if value >= (1 << (bits - 1)) else value


def read_position():
    data = read_raw(ADDR_PRESENT_POSITION, 4)
    if data is None:
        return None
    return int.from_bytes(bytes(data), "little", signed=True)


def sample():
    """One packet: present current + velocity + position (10 bytes from address 126).
    Logs the sample and returns (position, current_raw), or None on a communication error.
    Prints once if the motor raises its hardware alert flag."""
    global _alert_reported
    data, result, error = pk.readTxRx(port, ID, ADDR_PRESENT_CURRENT, 10)
    if result != COMM_SUCCESS or (error & 0x7F) != 0 or len(data) != 10:
        return None
    if (error & 0x80) and not _alert_reported:
        _alert_reported = True
        print("WARNING: motor raised a HARDWARE ALERT flag")
    cur = int.from_bytes(bytes(data[0:2]), "little", signed=True)
    pos = int.from_bytes(bytes(data[6:10]), "little", signed=True)
    log_t.append(time.time() - t_zero)
    log_pos.append(pos)
    log_cur.append(cur * CURRENT_UNIT_MA)
    return pos, cur


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
def save_plot(start, target, direction):
    """Draw the figure once, at the end, and save it (PNG + CSV)."""
    global _saved
    if _saved or not log_t or start is None:
        return
    fig, (ax_pos, ax_cur) = plt.subplots(2, 1, sharex=True, figsize=(10, 7))

    ax_pos.plot(log_t, log_pos, color="tab:blue", label="position")
    ax_pos.axhline(start, color="gray", linestyle=":", label="start")
    ax_pos.axhline(start + direction * TRIGGER, color="orange", linestyle=":", label="trigger threshold")
    ax_pos.axhline(target, color="green", linestyle=":", label="target")
    ax_pos.axhline(MAX_POSITION, color="black", linewidth=1.5, label="MAX_POSITION")
    ax_pos.axhline(MIN_POSITION, color="black", linewidth=1.5, linestyle="--", label="MIN_POSITION")
    ax_pos.set_ylabel("Position [ticks]")
    ax_pos.legend(loc="best")

    ax_cur.plot(log_t, log_cur, color="tab:red", label="present current")
    ax_cur.axhline(0, color="gray", linewidth=0.8)
    ax_cur.set_ylabel("Current [mA]")
    ax_cur.set_xlabel("Time [s]")
    ax_cur.legend(loc="best")

    # shade the free phase (torque OFF) and mark the torque-on moment
    free_end = t_torque_on if t_torque_on is not None else log_t[-1]
    for ax in (ax_pos, ax_cur):
        ax.axvspan(0, free_end, color="tab:cyan", alpha=0.15)
        ax.grid(True)
        if t_torque_on is not None:
            ax.axvline(t_torque_on, color="orange", linestyle="--", linewidth=1)
        if t_assist_end is not None:
            ax.axvline(t_assist_end, color="green", linestyle="--", linewidth=1)
    ax_pos.text(free_end / 2, ax_pos.get_ylim()[1], "torque OFF (free)", ha="center", va="top")

    fig.suptitle("Exofinger: position and current")
    fig.tight_layout()

    name = datetime.now().strftime("exofinger_%Y%m%d_%H%M%S")
    fig.savefig(name + ".png", dpi=150)
    with open(name + ".csv", "w") as f:
        f.write("time_s,position_ticks,current_mA\n")
        for t, p, c in zip(log_t, log_pos, log_cur):
            f.write(f"{t:.4f},{p},{c:.2f}\n")
    print(f"Saved {name}.png and {name}.csv")
    _saved = True
    if SHOW_PLOT:
        plt.show()
    else:
        plt.close(fig)


# ------------------------- MAIN -------------------------
def main():
    global t_zero, t_torque_on, t_assist_end

    if not port.openPort():
        print(f"Cannot open port {PORT}")
        return
    if not port.setBaudRate(BAUDRATE):
        print("Cannot set baud rate")
        port.closePort()
        return

    start = target = direction = None
    try:
        # ------------------------- SETUP -------------------------
        if not clear_hardware_error():
            return
        ok = (write_verified(ADDR_TORQUE_ENABLE, 0, 1)             # torque OFF: motor free
              and write_verified(ADDR_OPERATING_MODE, 5, 1)        # 5: current-based position control
              and write_verified(ADDR_GOAL_CURRENT, ASSIST_CURRENT, 2)
              and write_verified(ADDR_PROFILE_VELOCITY, ASSIST_SPEED, 4))
        if not ok:
            print("Setup failed (check wiring / power / baud rate / ID). Exiting.")
            return

        start = read_position()
        if start is None:
            print("Cannot read the present position. Exiting.")
            return

        if  start <= -500:    # currently CLOSED -> open
            print("Currently in CLOSED position")
            direction = +1
            target = min(start + MOVE_DISTANCE, MAX_POSITION)
        else:
            print("Currently in OPEN position")# currently OPEN -> close
            direction = -1
            target = max(start - MOVE_DISTANCE, MIN_POSITION)

        # if abs(target - start) < TRIGGER:
        #     print("Already at/near the limit in that direction, nothing to do. Exiting.")
        #     return

        print(f"start = {start}, target = {target}")
        print("Motor is FREE. Move it by hand toward the target...")

        t_zero = time.time()

        # ------------------------- 1) FREE PHASE: log only -------------------------
        t_free = time.time()
        while True:
            s = sample()
            if s is not None and (s[0] - start) * direction >= TRIGGER:
                break
            if time.time() - t_free > FREE_TIMEOUT:
                print("No movement detected, giving up.")
                return

        # ------------------------- 2) ASSIST -------------------------
        print("Movement detected -> turning torque ON, then setting goal")
        t_torque_on = time.time() - t_zero

        # Torque ON first: enabling torque resets the goal to the present position on this motor,
        # so the goal must be written AFTER it, and verified by reading it back.
        if not write_verified(ADDR_TORQUE_ENABLE, 1, 1):
            print("Could not enable torque -> aborting")
            return
        time.sleep(0.02)          # let the motor settle before the goal write
        if not set_goal_verified(target):
            print("Could not set the goal position (read-back mismatch) -> aborting")
            return
        print(f"goal position = {target} (verified)")

        hw = hardware_error_status()
        if hw:
            print(f"Motor shut down by hardware error {hw:#010b} ({describe_hw_error(hw)}). "
                  "Check the power supply (use 12 V, not USB only).")
            return

        t0 = time.time()
        reached = False
        stop_goal = None            # where to hold if the assist ends early
        while time.time() - t0 < TIMEOUT:
            s = sample()
            if s is None:
                continue
            pos = s[0]
            if pos >= MAX_POSITION:
                stop_goal = MAX_POSITION
                print(f"MAX_POSITION reached (pos={pos}) -> holding at {MAX_POSITION}")
                break
            if pos <= MIN_POSITION:
                stop_goal = MIN_POSITION
                print(f"MIN_POSITION reached (pos={pos}) -> holding at {MIN_POSITION}")
                break
            if abs(pos - target) <= TOLERANCE:
                reached = True
                break
            if _alert_reported:
                hw = hardware_error_status()
                if hw:
                    print(f"Hardware error {hw:#010b} ({describe_hw_error(hw)}) -> stopping assist")
                    break

        t_assist_end = time.time() - t_zero
        if reached:
            print("Target reached -> holding position")
        else:
            if stop_goal is None:           # timeout / error: hold where we are, don't keep pulling
                stop_goal = read_position()
            if stop_goal is not None:
                write_verified(ADDR_GOAL_POSITION, stop_goal & 0xFFFFFFFF, 4)
            print("Assist ended before target -> holding current position")
        write_verified(ADDR_GOAL_CURRENT, HOLD_CURRENT, 2)

        input("Press Enter to release the motor and save the plot (or Ctrl+C)...")

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        release_motor()                          # torque OFF, always
        save_plot(start, target, direction)      # free-phase data is never lost


if __name__ == "__main__":
    main()