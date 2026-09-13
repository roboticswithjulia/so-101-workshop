#!/usr/bin/env python3
"""Print the current position of each SO-101 servo using the Feetech SDK.

Install the dependencies first (pyserial and the ftservo-python-sdk package,
which provides the `scservo_sdk` module):

    pip install -r requirements.txt

Usage:

    python3 servo_positions.py                    # /dev/ttyACM0, IDs 1..6, one reading
    python3 servo_positions.py --loop             # refresh continuously (Ctrl+C to stop)
    python3 servo_positions.py --port /dev/ttyUSB0 --ids 1,2,3
    python3 servo_positions.py --set-zero         # take the current pose as the zero pose

Positions are raw encoder values, 0..4095 for one full turn (about 0.088 degrees
per step). Reading never writes to the servos. `--set-zero` does: it tells each
servo to treat its current position as the middle of its range (2048), which the
servo stores in EEPROM, so the value survives power cycles.
"""

import argparse
import time

import sys
from pathlib import Path

# Allow running this file directly (python3 <folder>/<file>.py) as well as with python3 -m
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from src.so101_bus import (
    ARM_IDS,
    COMM_SUCCESS,
    DEFAULT_BAUDRATE,
    DEFAULT_PORT,
    REG_TORQUE_ENABLE,
    STEPS_PER_TURN,
    describe,
    joint_name,
    open_bus,
    parse_ids,
)

# STS3215 torque register (40): 0 = off, 1 = on, 128 = "take the current position
# as the middle (2048)". The servo computes and stores the offset itself.
TORQUE_SET_MIDDLE = 128
POSITION_MIDDLE = 2048
ZERO_TOLERANCE = 8  # steps; how close to 2048 the servo must read after zeroing
# Speed (steps/s) and acceleration (100 steps/s^2 units) used only to hold a
# joint in place; the same gentle values as the app's moves.
HOLD_SPEED = 300
HOLD_ACC = 15


def read_positions(packet_handler, ids):
    """Return {id: {"position", "speed", "moving", "error"}} for the servos that answer."""
    readings = {}
    for servo_id in ids:
        position, speed, result, error = packet_handler.ReadPosSpeed(servo_id)
        if result != COMM_SUCCESS:
            continue
        moving, result, _ = packet_handler.ReadMoving(servo_id)
        readings[servo_id] = {
            "position": int(position),
            "speed": int(speed),
            "moving": bool(moving) if result == COMM_SUCCESS else None,
            "error": packet_handler.getRxPacketError(error) if error else "",
        }
    return readings


def set_zero_positions(packet_handler, ids, log=print):
    """Make each servo read its current position as the zero pose (middle, 2048).

    For every servo that answers: read the position, send the "set middle"
    command, restore the torque state and read the position again. Returns
    {id: {"before", "after", "ok"}}; "ok" is True when the servo now reports
    2048 within ZERO_TOLERANCE steps.
    """
    results = {}
    for servo_id in ids:
        name = joint_name(servo_id)
        before, _, result, error = packet_handler.ReadPosSpeed(servo_id)
        if result != COMM_SUCCESS:
            log(f"{name} (ID {servo_id}): no answer, skipped. {describe(packet_handler, result, error)}")
            continue
        torque, result, _ = packet_handler.read1ByteTxRx(servo_id, REG_TORQUE_ENABLE)
        torque_was_on = result == COMM_SUCCESS and torque == 1

        packet_handler.unLockEprom(servo_id)
        result, error = packet_handler.write1ByteTxRx(servo_id, REG_TORQUE_ENABLE, TORQUE_SET_MIDDLE)
        packet_handler.LockEprom(servo_id)
        if result != COMM_SUCCESS:
            log(f"{name} (ID {servo_id}): set-zero command failed. {describe(packet_handler, result, error)}")
            continue
        time.sleep(0.05)
        after, _, result, _ = packet_handler.ReadPosSpeed(servo_id)
        after = int(after) if result == COMM_SUCCESS else None
        if after is not None:
            # Hold the joint where it is in the new coordinates before the torque
            # comes back, so a stale goal position cannot make it jump.
            packet_handler.WritePosEx(servo_id, after, HOLD_SPEED, HOLD_ACC)
        if torque_was_on:
            packet_handler.write1ByteTxRx(servo_id, REG_TORQUE_ENABLE, 1)

        ok = after is not None and abs(after - POSITION_MIDDLE) <= ZERO_TOLERANCE
        results[servo_id] = {"before": int(before), "after": after, "ok": ok}
        state = "ok" if ok else "NOT applied"
        log(f"{name} (ID {servo_id}): {before} -> {after if after is not None else '--'} ({state})")
    return results


def print_table(ids, readings):
    print(f"{'ID':>3}  {'Joint':<11} {'Position':>8} {'Degrees':>8} {'Speed':>6}  Status")
    for servo_id in ids:
        name = joint_name(servo_id)
        reading = readings.get(servo_id)
        if reading is None:
            print(f"{servo_id:>3}  {name:<11} {'--':>8} {'--':>8} {'--':>6}  no answer")
            continue
        degrees = reading["position"] * 360.0 / STEPS_PER_TURN
        status = "moving" if reading["moving"] else "idle"
        if reading["error"]:
            status += " " + reading["error"]
        print(f"{servo_id:>3}  {name:<11} {reading['position']:>8} {degrees:>8.1f} {reading['speed']:>6}  {status}")


def main(argv=None):
    parser = argparse.ArgumentParser(description="Read the current position of each SO-101 servo.")
    parser.add_argument("--port", default=DEFAULT_PORT, help="serial device (default: %(default)s)")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUDRATE, help="baud rate (default: %(default)s)")
    parser.add_argument("--ids", default=ARM_IDS, help="servo IDs, e.g. '1-6' or '1,2,3' (default: %(default)s)")
    parser.add_argument("--loop", action="store_true", help="keep reading until Ctrl+C")
    parser.add_argument("--interval", type=float, default=0.5, help="seconds between readings with --loop (default: %(default)s)")
    parser.add_argument("--set-zero", action="store_true",
                        help="make every servo treat its current position as the zero pose (stored in the servo EEPROM)")
    args = parser.parse_args(argv)

    ids = parse_ids(args.ids)
    try:
        port_handler, packet_handler = open_bus(args.port, args.baud)
    except RuntimeError as exc:
        print(exc)
        return 1
    print(f"Connected to {args.port} @ {args.baud} bps\n")

    try:
        if args.set_zero:
            readings = read_positions(packet_handler, ids)
            print_table(ids, readings)
            if not readings:
                print("\nNo servo answered. Nothing changed.")
                return 1
            print("\nThe current pose will become the zero pose: each servo above will read 2048 here.")
            print("This is written to the servo EEPROM and replaces any previous zero.")
            answer = input("Type 'yes' to continue: ")
            if answer.strip().lower() != "yes":
                print("Cancelled. Nothing changed.")
                return 1
            print()
            results = set_zero_positions(packet_handler, ids)
            failed = [servo_id for servo_id, r in results.items() if not r["ok"]]
            print()
            print_table(ids, read_positions(packet_handler, ids))
            if failed:
                print(f"\nZero NOT applied on IDs {failed}. Check the servo firmware or use the Feetech tool.")
                return 1
            print("\nZero pose stored on all servos that answered.")
            return 0

        while True:
            readings = read_positions(packet_handler, ids)
            print_table(ids, readings)
            if not readings:
                print("\nNo servo answered. Check the 7.4V power, the cable chain and the servo IDs.")
            if not args.loop:
                return 0 if readings else 1
            time.sleep(args.interval)
            print()
    except KeyboardInterrupt:
        print()
        return 0
    finally:
        port_handler.closePort()


if __name__ == "__main__":
    sys.exit(main())
