#!/usr/bin/env python3
"""
Forward-left thruster unit test (forward_port / idx=1).

Compatible with Teensy firmware that supports:
  - MOTOR,idx,pwm
  - RAW,p1,p2,p3,p4  (used here only for neutral/arming)
  - RAW_OFF

This version assumes the Teensy is always on:
  /dev/ttyACM0

Usage:
  python3 test_forward_left_thruster_acm0.py
  python3 test_forward_left_thruster_acm0.py --pwm 1600 --duration 2
"""

from __future__ import annotations

import argparse
import time

try:
    import serial  # pyserial
except ImportError as e:
    raise SystemExit("Missing dependency: pyserial. Install with: pip install pyserial") from e


# Fixed settings
PORT = "/dev/ttyACM0"
BAUD = 115200
HZ = 20.0
ARM_TIME_S = 1.0
OPEN_RESET_DELAY_S = 2.0  # Teensy often reboots when serial opens

# Forward-left mapping: forward_port = idx 1
THRUSTER_IDX = 1

# Firmware/ESC assumptions
MIN_PULSE = 1100
MAX_PULSE = 1900
NEUTRAL = 1500


def clamp_pwm(pwm: int) -> int:
    return max(MIN_PULSE, min(MAX_PULSE, int(pwm)))


def send_line(ser: serial.Serial, line: str) -> None:
    if not line.endswith("\n"):
        line += "\n"
    ser.write(line.encode("utf-8"))


def hold_line(ser: serial.Serial, line: str, duration_s: float) -> None:
    period = 1.0 / HZ
    t0 = time.time()
    while time.time() - t0 < duration_s:
        send_line(ser, line)
        time.sleep(period)


def main():
    ap = argparse.ArgumentParser(description="Unit test forward-left thruster (idx=1) on /dev/ttyACM0.")
    ap.add_argument("--pwm", type=int, default=1600, help="PWM us (1100-1900). 1500=neutral. Default 1600.")
    ap.add_argument("--duration", type=float, default=2.0, help="Seconds to run the test. Default 2.")
    args = ap.parse_args()

    pwm = clamp_pwm(args.pwm)
    duration = max(0.0, float(args.duration))

    neutral_raw = f"RAW,{NEUTRAL},{NEUTRAL},{NEUTRAL},{NEUTRAL}"
    motor_line = f"MOTOR,{THRUSTER_IDX},{pwm}"

    try:
        with serial.Serial(PORT, BAUD, timeout=1) as ser:
            time.sleep(OPEN_RESET_DELAY_S)

            try:
                # Arm / neutral
                hold_line(ser, neutral_raw, ARM_TIME_S)

                # Run forward-left thruster only
                hold_line(ser, motor_line, duration)

            finally:
                # Stop safely
                hold_line(ser, neutral_raw, 0.5)
                send_line(ser, "RAW_OFF")

    except serial.SerialException as e:
        raise SystemExit(
            f"Could not open serial port {PORT}. "
            f"Is the Teensy connected and does it appear as {PORT}?\n"
            f"Original error: {e}"
        ) from e


if __name__ == "__main__":
    main()
