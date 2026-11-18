#!/usr/bin/env python3
"""Flex sensor -> Dynamixel motor trigger.

Bend the flex sensor (dR/R0 moves by --trig percent) and the motor moves the finger
to the other end: open -> closed, then closed -> open, and so on.

How it works:
  1. The motor is in current-based position control (mode 5) with a max current of 100.
  2. The position where the motor starts is one end of the movement (--start says if
     it is OPEN or CLOSED); the other end is MOVE_DISTANCE ticks away. No fixed
     position limits are needed, so it does not matter where the motor's zero is.
  3. The flex sensor is read continuously (code reused from flex_sensors/).
  4. When the trigger fires, the sensor is NOT read while the motor moves (the motor
     itself bends the sensor). After the move the sensor settles and is ZEROED: its
     dR/R0 at that moment becomes the new baseline. The next trigger fires only when
     dR/R0 moves --trig percent away from that baseline, in the OPPOSITE direction.
     Keep the finger still for ~1 s after each move while the baseline is measured.

Place this file at the repo root, next to the flex_sensors/ and motor/ folders.

Examples:
    python flex_triggered_motor.py --start open --trig 20
    python flex_triggered_motor.py --start closed --trig -20
    python flex_triggered_motor.py --start open --trig 20 --saved-flat
    Ctrl+C stops the script and turns the motor torque OFF.
"""
import argparse
import glob
import statistics
import sys
import time
from collections import deque
from pathlib import Path

# ------------------------- FLEX SENSOR CODE (from flex_sensors/) -------------------------
# flex_shape.py does "from read_serial import ...", so the folder that holds read_serial.py
# must be on the import path. Both likely locations are added here.
ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / "flex_sensors" / "scripts"))
sys.path.insert(0, str(ROOT / "flex_sensors"))

from flex_shape import (open_port,         # opens the Nano 33 IoT serial port
                        read_resistance,   # reads one resistance sample [ohm]
                        resolve_r0)        # flat-sensor resistance (zero now, or saved)

# ------------------------- MOTOR SETTINGS -------------------------
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

MOTOR_ID = 1
BAUD = 57600
MOTOR_PORT = (glob.glob("/dev/cu.usbserial*") or [None])[0]   # same auto-detection as the ping script

MAX_CURRENT = 100        # current limit (raw units, 1 unit = 2.69 mA) -> ~269 mA
PROFILE_VELOCITY = 20    # max speed of the movement (1 unit = 0.229 rpm)

MOVE_DISTANCE = 1200     # ticks between OPEN and CLOSED (closing = position decreases)
TOLERANCE = 15           # ticks: "target reached"
MOVE_TIMEOUT = 15        # seconds: give up waiting for the target
SETTLE_S = 1.0           # seconds to let the sensor settle after the motor stops
BASELINE_S = 1.0         # seconds averaged to zero the sensor (keep the finger still)

# Dynamixel control table addresses (X series, protocol 2.0)
ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_HARDWARE_ERROR = 70
ADDR_GOAL_CURRENT = 102
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132

ph = PortHandler(MOTOR_PORT or "")
pk = PacketHandler(2.0)


# ------------------------- MOTOR HELPERS -------------------------
def write(address, value, n_bytes):
    """Write a register and return True if the motor acknowledged it without error."""
    fn = {1: pk.write1ByteTxRx, 2: pk.write2ByteTxRx, 4: pk.write4ByteTxRx}[n_bytes]
    for _ in range(5):                                   # a few retries against noise
        result, error = fn(ph, MOTOR_ID, address, value)
        if result == COMM_SUCCESS and (error & 0x7F) == 0:
            return True
        time.sleep(0.01)
    print(f"WRITE FAILED (address {address})")
    return False


def read_bytes(address, length):
    """Read raw bytes. Returns a list of bytes or None. Never raises on a motor error packet
    (pk.read4ByteTxRx does raise an IndexError in that case, so it is not used)."""
    data, result, error = pk.readTxRx(ph, MOTOR_ID, address, length)
    if result != COMM_SUCCESS or (error & 0x7F) != 0 or len(data) != length:
        return None
    return data


def read_position():
    """Present position in ticks (signed), or None on a communication/motor error."""
    data = read_bytes(ADDR_PRESENT_POSITION, 4)
    if data is None:
        return None
    return int.from_bytes(bytes(data), "little", signed=True)


def hardware_error():
    """Hardware error status byte (0 = OK), or None if unreadable."""
    data = read_bytes(ADDR_HARDWARE_ERROR, 1)
    return None if data is None else data[0]


def set_goal(position):
    """Command a goal position (negative numbers need the 32-bit wrap)."""
    return write(ADDR_GOAL_POSITION, position & 0xFFFFFFFF, 4)


def setup_motor():
    """Current-based position control (mode 5) with a max current of MAX_CURRENT. Torque ON.
    Returns the starting position."""
    if not MOTOR_PORT:
        sys.exit("No /dev/cu.usbserial* port found for the motor.")
    if not ph.openPort() or not ph.setBaudRate(BAUD):
        sys.exit(f"Cannot open the motor port {MOTOR_PORT}")

    # If the motor latched a hardware error (overload, voltage...), reboot to clear it.
    hw = hardware_error()
    if hw:
        print(f"Motor hardware error {hw:#010b} -> rebooting the motor")
        pk.reboot(ph, MOTOR_ID)
        time.sleep(1.5)

    # The operating mode can only be changed while torque is OFF.
    ok = (write(ADDR_TORQUE_ENABLE, 0, 1)
          and write(ADDR_OPERATING_MODE, 5, 1)                 # 5 = current-based position control
          and write(ADDR_GOAL_CURRENT, MAX_CURRENT, 2)         # max current while moving/holding
          and write(ADDR_PROFILE_VELOCITY, PROFILE_VELOCITY, 4)
          and write(ADDR_TORQUE_ENABLE, 1, 1))
    if not ok:
        sys.exit("Motor setup failed (check wiring / 12 V power / baud rate / ID).")

    # Hold the current position until the first trigger.
    pos = read_position()
    if pos is None or not set_goal(pos):
        sys.exit("Cannot read/hold the present position.")
    print(f"Motor ready at position {pos} (max current {MAX_CURRENT} = {MAX_CURRENT * 2.69:.0f} mA)")
    return pos


def release_motor():
    """Torque OFF no matter what happened (also after Ctrl+C)."""
    ph.is_using = False                                  # clear a lock left by an interrupted transaction
    if not write(ADDR_TORQUE_ENABLE, 0, 1):
        print("!!! COULD NOT TURN TORQUE OFF - power-cycle the motor !!!")
    ph.closePort()


def move_to(target):
    """Send the motor to `target` and block until it arrives (or timeout).
    Returns True if the target was reached."""
    if not set_goal(target):
        return False
    t0 = time.time()
    while time.time() - t0 < MOVE_TIMEOUT:
        pos = read_position()
        if pos is not None and abs(pos - target) <= TOLERANCE:
            print("  target reached")
            return True
    # Timeout: hold where we are instead of continuing to pull.
    hw = hardware_error()
    if hw:
        print(f"  motor hardware error status = {hw:#010b} (check the 12 V supply, not USB only)")
    pos = read_position()
    if pos is not None:
        set_goal(pos)
    print("  target NOT reached (timeout) -> holding current position")
    return False


# ------------------------- SENSOR ZEROING -------------------------
def measure_baseline(ser, R0):
    """Wait for the sensor to settle, then average dR/R0 [%] for BASELINE_S. This is the new zero."""
    time.sleep(SETTLE_S)
    ser.reset_input_buffer()                             # drop everything read/buffered during the move
    vals, t_end = [], time.time() + BASELINE_S
    while time.time() < t_end or len(vals) < 5:
        r = read_resistance(ser)
        if r is not None:
            vals.append(r)
    return (statistics.mean(vals) - R0) / R0 * 100.0


# ------------------------- MAIN -------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", "--port", help="flex sensor (Nano 33 IoT) serial port; auto-detected if omitted")
    ap.add_argument("--start", choices=["open", "closed"], required=True,
                    help="is the finger OPEN or CLOSED when the script starts?")
    ap.add_argument("--trig", type=float, default=-20.0,
                    help="first trigger: dR/R0 moves this many percent away from the zeroed baseline "
                         "(positive = rising, negative = falling). The direction flips after each move.")
    ap.add_argument("--smooth", type=int, default=5, help="moving-average window in samples")
    ap.add_argument("--saved-flat", action="store_true",
                    help="use the flat R0 saved by 'flex_shape.py flat' instead of zeroing now")
    args = ap.parse_args()

    # 1) Flex sensor: open the port and get the flat resistance R0
    ser = open_port(args.port)
    print("Using saved R0" if args.saved_flat else "Keep the flex sensor FLAT and still while zeroing...")
    R0 = resolve_r0(ser, None, saved_flat=args.saved_flat)
    print(f"Flat R0 = {R0 / 1000:.3f} kOhm")

    # 2) Motor. The start position is one end; the other end is MOVE_DISTANCE ticks away
    #    (closing = position decreases, opening = position increases).
    start = setup_motor()
    is_open = (args.start == "open")
    open_pos = start if is_open else start + MOVE_DISTANCE
    closed_pos = start - MOVE_DISTANCE if is_open else start
    print(f"Starting {args.start.upper()}: open = {open_pos}, closed = {closed_pos}")

    delta = args.trig
    win = deque(maxlen=args.smooth)
    print("\nKeep the finger still, zeroing the sensor...")
    baseline = measure_baseline(ser, R0)                 # dR/R0 [%] that counts as "no movement"
    print(f"Baseline dR/R0 = {baseline:+.2f} %. Watching for {delta:+.1f} % from it. Ctrl+C to stop.\n")

    try:
        while True:
            r = read_resistance(ser)
            if r is None:
                continue
            win.append(r)
            R = statistics.mean(win)
            x_pct = (R - R0) / R0 * 100.0                # dR/R0 in percent

            change = x_pct - baseline                    # movement since the last zero [% points]
            print(f"\rdR/R0 = {x_pct:+6.2f} %   change from zero = {change:+6.2f} %   ", end="", flush=True)

            # fire when the change reaches |delta| in the chosen direction (negative delta = falling)
            if (change >= delta) if delta > 0 else (change <= delta):
                print(f"\n>>> TRIGGER  change = {change:+.2f} %  (dR/R0 = {x_pct:+.2f} %)")
                if is_open:
                    print(f"  open -> closing to {closed_pos}")
                    moved = move_to(closed_pos)          # the sensor is NOT read while the motor moves
                else:
                    print(f"  closed -> opening to {open_pos}")
                    moved = move_to(open_pos)

                if moved:
                    is_open = not is_open
                    delta = -delta                       # now watch for the opposite movement
                win.clear()
                print("  keep the finger still, zeroing the sensor...")
                baseline = measure_baseline(ser, R0)     # ZERO at the new curvature
                print(f"  now {'OPEN' if is_open else 'CLOSED'}; baseline = {baseline:+.2f} %, "
                      f"watching for {delta:+.1f} % from it")
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        release_motor()                                  # torque OFF, always
        ser.close()


if __name__ == "__main__":
    main()