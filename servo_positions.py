#!/usr/bin/env python3
"""Print the current position of each SO-101 servo using the Feetech SDK.

Install the dependencies first (pyserial and the ftservo-python-sdk package,
which provides the `scservo_sdk` module):

    pip install -r requirements.txt

Usage:

    python3 servo_positions.py                    # /dev/ttyACM0, IDs 1..6, one reading
    python3 servo_positions.py --loop             # refresh continuously (Ctrl+C to stop)
    python3 servo_positions.py --port /dev/ttyUSB0 --ids 1,2,3

Positions are raw encoder values, 0..4095 for one full turn (about 0.088 degrees
per step). Nothing is written to the servos.
"""

import argparse
import sys
import time

from so101_bus import (
    ARM_IDS,
    COMM_SUCCESS,
    DEFAULT_BAUDRATE,
    DEFAULT_PORT,
    STEPS_PER_TURN,
    joint_name,
    open_bus,
    parse_ids,
)


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
    args = parser.parse_args(argv)

    ids = parse_ids(args.ids)
    try:
        port_handler, packet_handler = open_bus(args.port, args.baud)
    except RuntimeError as exc:
        print(exc)
        return 1
    print(f"Connected to {args.port} @ {args.baud} bps\n")

    try:
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
