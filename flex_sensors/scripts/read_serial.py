#!/usr/bin/env python3
"""Read and optionally log ADC telemetry streamed from Arduino Nano 33 IoT over USB Serial."""

from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import TextIO

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:
    raise SystemExit(
        "Missing dependency 'pyserial'. Activate the virtual environment and install it:\n"
        "  python3 -m venv .venv\n"
        "  source .venv/bin/activate\n"
        "  pip install -r requirements.txt"
    ) from exc


NANO_33_IOT_VID = 0x2341
NANO_33_IOT_PIDS = {0x0057, 0x8057}  # Normal USB CDC & SAM-BA bootloader PIDs
DEFAULT_BAUD = 115200
CSV_HEADER = ("timestamp_ms", "raw_adc", "voltage_v", "resistance_ohm")


@dataclass(frozen=True)
class FlexSample:
    """Single parsed telemetry sample from the Nano 33 IoT."""

    timestamp_ms: int
    raw_adc: float
    voltage_v: float
    resistance_ohm: float


def find_nano_port() -> str | None:
    """Return the serial device path for a connected Arduino Nano 33 IoT, if found."""
    ports = list(list_ports.comports())
    for port in ports:
        if port.vid == NANO_33_IOT_VID and port.pid in NANO_33_IOT_PIDS:
            return port.device

    for port in ports:
        desc = (port.description or "").lower()
        dev = port.device.lower()
        if "nano 33 iot" in desc or "arduino" in desc or "usbmodem" in dev or "ttyacm" in dev:
            return port.device

    return None


def parse_sample(line: str) -> FlexSample | None:
    """Parse one CSV line (`timestamp_ms,raw_adc,voltage_v,resistance_ohm`)."""
    parts = [part.strip() for part in line.split(",")]
    if len(parts) != 4:
        return None
    if parts[0] == "timestamp_ms":
        return None

    try:
        timestamp_ms = int(parts[0])
        raw_adc = float(parts[1])
        voltage_v = float(parts[2])
        resistance_ohm = math.inf if parts[3].lower() == "inf" else float(parts[3])
    except ValueError:
        return None

    return FlexSample(
        timestamp_ms=timestamp_ms,
        raw_adc=raw_adc,
        voltage_v=voltage_v,
        resistance_ohm=resistance_ohm,
    )


def format_resistance(ohms: float) -> str:
    """Format resistance with Ω or kΩ units for readable console display."""
    if not math.isfinite(ohms):
        return "inf"
    if ohms >= 1000.0:
        return f"{ohms / 1000.0:7.3f} kΩ"
    return f"{ohms:7.1f} Ω "


def open_csv_writer(path: Path | None) -> tuple[TextIO | None, csv.writer | None]:
    """Open an optional CSV file and write the header if empty."""
    if path is None:
        return None, None
    path.parent.mkdir(parents=True, exist_ok=True)
    file_exists = path.exists() and path.stat().st_size > 0
    fh = path.open("a", newline="", encoding="utf-8")
    writer = csv.writer(fh)
    if not file_exists:
        writer.writerow(CSV_HEADER)
        fh.flush()
    return fh, writer


def stream_serial(
    port: str,
    baud: int,
    out_path: Path | None,
    raw_mode: bool,
    max_samples: int | None,
) -> int:
    """Stream samples from the serial port until interrupted or `max_samples` reached."""
    csv_file, csv_writer = open_csv_writer(out_path)
    samples_read = 0

    try:
        with serial.Serial(port, baud, timeout=1.0, exclusive=True) as ser:
            # Assert DTR so SAMD21 native USB CDC knows a host listener is open.
            ser.dtr = True
            time.sleep(0.1)
            ser.reset_input_buffer()

            if raw_mode:
                print(",".join(CSV_HEADER), flush=True)
            else:
                print(f"Connected to {port} @ {baud} baud. Press Ctrl+C to stop.", file=sys.stderr)
                print(f"{'TIME (ms)':>10} | {'ADC (12-bit)':>12} | {'VOLTAGE':>9} | {'FLEX R':>11}", flush=True)
                print("-" * 51, flush=True)

            while max_samples is None or samples_read < max_samples:
                raw_line = ser.readline().decode("utf-8", errors="replace").strip()
                if not raw_line:
                    continue

                sample = parse_sample(raw_line)
                if sample is None:
                    continue

                samples_read += 1
                if csv_writer is not None and csv_file is not None:
                    res_str = "inf" if not math.isfinite(sample.resistance_ohm) else f"{sample.resistance_ohm:.1f}"
                    csv_writer.writerow([sample.timestamp_ms, f"{sample.raw_adc:.1f}", f"{sample.voltage_v:.4f}", res_str])
                    csv_file.flush()

                if raw_mode:
                    print(raw_line, flush=True)
                else:
                    print(
                        f"{sample.timestamp_ms:10d} | {sample.raw_adc:12.1f} | {sample.voltage_v:7.4f} V | {format_resistance(sample.resistance_ohm):>11}",
                        flush=True,
                    )
    except KeyboardInterrupt:
        print(f"\nStopped after {samples_read} samples.", file=sys.stderr)
    except serial.SerialException as exc:
        print(
            f"Serial error on {port}: {exc}\n"
            "Make sure no other monitor (e.g. `pio device monitor`) is currently using the port.",
            file=sys.stderr,
        )
        return 1
    finally:
        if csv_file is not None:
            csv_file.close()

    return 0


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Read SpectraFlex ADC readings streamed from Arduino Nano 33 IoT."
    )
    parser.add_argument(
        "-p",
        "--port",
        help="Serial port (auto-detects Arduino Nano 33 IoT if omitted).",
    )
    parser.add_argument(
        "-b",
        "--baud",
        type=int,
        default=DEFAULT_BAUD,
        help=f"Serial baud rate (default: {DEFAULT_BAUD}).",
    )
    parser.add_argument(
        "-o",
        "--out",
        type=Path,
        help="Optional CSV file path to append recorded samples.",
    )
    parser.add_argument(
        "--raw",
        action="store_true",
        help="Print raw CSV lines to stdout instead of formatted columns.",
    )
    parser.add_argument(
        "-n",
        "--count",
        type=int,
        default=None,
        help="Stop after reading N samples (useful for scripted captures).",
    )
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    port = args.port or find_nano_port()
    if port is None:
        print(
            "Could not auto-detect an Arduino Nano 33 IoT serial port.\n"
            "Check the USB connection or specify one explicitly with --port.",
            file=sys.stderr,
        )
        return 1

    return stream_serial(
        port=port,
        baud=args.baud,
        out_path=args.out,
        raw_mode=args.raw,
        max_samples=args.count,
    )


if __name__ == "__main__":
    raise SystemExit(main())
