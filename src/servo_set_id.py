#!/usr/bin/env python3
"""Change the ID of a Feetech STS3215 servo.

Connect ONE servo at a time. The ID lives in the servo EEPROM, so every servo
that currently answers to the old ID would be renamed at once, and two servos
sharing an ID cannot be told apart afterwards.

    python3 src/servo_set_id.py --from 1 --to 3
    python3 src/servo_set_id.py --from 1 --to 3 --port /dev/ttyUSB0 --yes

The script refuses to run when more than one servo answers on the bus, or when
the target ID is already taken, unless you pass --force.

Typical use for the SO-101: servos arrive set to ID 1, so each joint is given
its own ID before assembly, following the mapping
1 Base, 2 Shoulder, 3 Elbow, 4 Wrist, 5 Wrist roll, 6 Gripper.
"""

import argparse
import sys
import time

from pathlib import Path

# Allow running this file directly (python3 <folder>/<file>.py) as well as with python3 -m
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from src.so101_bus import (
    COMM_SUCCESS,
    DEFAULT_BAUDRATE,
    DEFAULT_PORT,
    MAX_ID,
    REG_ID,
    describe,
    joint_name,
    open_bus,
)

# The servo needs a moment to store the new ID before it answers to it.
WRITE_SETTLE_SECONDS = 0.1


def scan(packet_handler, ids=range(1, MAX_ID + 1)):
    """Return the sorted list of IDs that answer a ping."""
    return sorted(servo_id for servo_id in ids if packet_handler.ping(servo_id)[1] == COMM_SUCCESS)


def set_servo_id(packet_handler, old_id, new_id, settle=WRITE_SETTLE_SECONDS):
    """Write a new ID to one servo. Returns (ok, message).

    The EEPROM is unlocked, the ID register is written and the EEPROM is locked
    again. The lock goes to the NEW id: the servo answers to it the moment the
    write lands. Success is confirmed by pinging the new ID.
    """
    packet_handler.unLockEprom(old_id)
    result, error = packet_handler.write1ByteTxRx(old_id, REG_ID, new_id)
    if result != COMM_SUCCESS:
        packet_handler.LockEprom(old_id)
        return False, f"Writing the ID failed: {describe(packet_handler, result, error)}"

    time.sleep(settle)
    packet_handler.LockEprom(new_id)

    if packet_handler.ping(new_id)[1] != COMM_SUCCESS:
        return False, f"Servo {new_id} does not answer after the change. Power-cycle the servo and scan the bus."
    if packet_handler.ping(old_id)[1] == COMM_SUCCESS:
        return False, f"Servo {old_id} still answers: the old ID was not replaced. Check that only one servo is connected."
    return True, f"Servo {old_id} is now ID {new_id} ({joint_name(new_id)})."


def check_bus(found, old_id, new_id):
    """Reasons not to change the ID, given the IDs that answered. Returns [str]."""
    problems = []
    if old_id == new_id:
        problems.append(f"The old and the new ID are the same ({old_id}).")
    if not found:
        problems.append("No servo answered. Check the 7.4V supply, the cable and the baud rate.")
        return problems
    if old_id not in found:
        problems.append(f"No servo answers to ID {old_id}. The bus has {found}.")
    if new_id in found:
        problems.append(f"ID {new_id} is already taken. Two servos must never share an ID.")
    if len(found) > 1:
        problems.append(f"{len(found)} servos answered {found}. Connect ONE servo at a time: all of them would be renamed.")
    return problems


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Change the ID of one Feetech STS3215 servo.",
        epilog="Connect a single servo to the bus adapter before running this.",
    )
    parser.add_argument("--from", dest="old_id", type=int, required=True, help="current servo ID")
    parser.add_argument("--to", dest="new_id", type=int, required=True, help="new servo ID (0..252)")
    parser.add_argument("--port", default=DEFAULT_PORT, help="serial device (default: %(default)s)")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUDRATE, help="baud rate (default: %(default)s)")
    parser.add_argument("--yes", action="store_true", help="do not ask for confirmation")
    parser.add_argument("--force", action="store_true",
                        help="change the ID even if several servos answer or the target ID is taken")
    args = parser.parse_args(argv)

    for value, label in ((args.old_id, "--from"), (args.new_id, "--to")):
        if not 0 <= value <= MAX_ID:
            print(f"{label} must be between 0 and {MAX_ID}.")
            return 1

    try:
        port_handler, packet_handler = open_bus(args.port, args.baud)
    except RuntimeError as exc:
        print(exc)
        return 1
    print(f"Connected to {args.port} @ {args.baud} bps")

    try:
        print("Scanning the bus ...")
        found = scan(packet_handler)
        print(f"Servos that answered: {found or 'none'}")

        problems = check_bus(found, args.old_id, args.new_id)
        if problems:
            for problem in problems:
                print(f"  ! {problem}")
            if not args.force:
                print("\nNothing changed. Fix the problems above, or pass --force if you know what you are doing.")
                return 1
            print("\n--force given: continuing anyway.")

        if not args.yes:
            print(f"\nServo {args.old_id} will become ID {args.new_id} ({joint_name(args.new_id)}).")
            print("This is written to the servo EEPROM and survives a power cycle.")
            if input("Type 'yes' to continue: ").strip().lower() != "yes":
                print("Cancelled. Nothing changed.")
                return 1

        ok, message = set_servo_id(packet_handler, args.old_id, args.new_id)
        print(message)
        if not ok:
            return 1
        print(f"Bus now: {scan(packet_handler)}")
    finally:
        port_handler.closePort()

    print("\nNext steps:")
    print("1. Repeat for every joint, one servo at a time.")
    print("2. Run python3 src/servo_scan.py --ids 1-6 with the whole arm connected.")
    print("3. Run python3 src/servo_config.py to check the configuration of each servo.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
