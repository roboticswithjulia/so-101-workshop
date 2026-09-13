#!/usr/bin/env python3
"""Show or clear the stored position offset of each SO-101 servo.

Every time a servo is told "take your current position as the middle", it adds
to the offset in its EEPROM rather than replacing it. After a few rounds of
re-zeroing, a joint can end up hundreds of steps away from the encoder's native
zero, which puts its working range near the 0/4095 seam. Crossing that seam
makes the servo travel the long way round, so a small commanded move turns into
most of a revolution.

    python3 src/servo_offsets.py                 # read only: show the offsets
    python3 src/servo_offsets.py --reset         # clear them, then re-zero once
    python3 src/servo_offsets.py --reset --ids 6 # only the gripper

Clearing an offset changes the position the servo reports, so each servo is
immediately commanded to hold its new reading. The torque is never turned off:
the arm must not go limp. Afterwards put the arm in its neutral pose and zero it
exactly once, with the app or with servo_positions.py --set-zero.
"""

import argparse
import sys
import time

from pathlib import Path

# Allow running this file directly (python3 <folder>/<file>.py) as well as with python3 -m
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from src.servo_positions import HOLD_ACC, HOLD_SPEED, POSITION_MIDDLE, read_positions
from src.so101_bus import (
    ARM_IDS,
    COMM_SUCCESS,
    DEFAULT_BAUDRATE,
    DEFAULT_PORT,
    REG_OFFSET,
    STEPS_PER_TURN,
    decode_offset,
    describe,
    joint_name,
    open_bus,
    parse_ids,
)

# A joint whose raw encoder position is within this many steps of 0 or 4095 can
# cross the seam during a normal move.
WRAP_MARGIN = 200
WRITE_SETTLE_SECONDS = 0.05


def raw_position(position, offset):
    """The encoder reading behind a reported position, wrapped into 0..4095."""
    return (position + offset) % STEPS_PER_TURN


def distance_to_wrap(raw):
    """How many steps the joint is from the 0/4095 seam."""
    return min(raw, STEPS_PER_TURN - raw)


def read_offsets(packet_handler, ids):
    """Return {id: {"offset", "position", "raw", "to_wrap"}} for the servos that answer."""
    report = {}
    readings = read_positions(packet_handler, ids)
    for servo_id, reading in readings.items():
        value, result, _ = packet_handler.read2ByteTxRx(servo_id, REG_OFFSET)
        if result != COMM_SUCCESS:
            continue
        offset = decode_offset(value)
        raw = raw_position(reading["position"], offset)
        report[servo_id] = {
            "offset": offset,
            "position": reading["position"],
            "raw": raw,
            "to_wrap": distance_to_wrap(raw),
        }
    return report


def clear_offset(packet_handler, servo_id, settle=WRITE_SETTLE_SECONDS):
    """Set one servo's offset to 0 and hold it at its new reading. Returns (ok, message).

    The torque is left on throughout, and the servo is commanded to the position
    it reports once the offset is gone, so it does not move.
    """
    name = joint_name(servo_id)
    packet_handler.unLockEprom(servo_id)
    result, error = packet_handler.write2ByteTxRx(servo_id, REG_OFFSET, 0)
    packet_handler.LockEprom(servo_id)
    if result != COMM_SUCCESS:
        return False, f"{name} (ID {servo_id}): clearing the offset failed. {describe(packet_handler, result, error)}"

    time.sleep(settle)
    position, result, _ = packet_handler.read2ByteTxRx(servo_id, 56)
    if result != COMM_SUCCESS:
        return False, f"{name} (ID {servo_id}): no answer after clearing the offset."
    # Hold the joint where it now reads, so a stale goal cannot drag it.
    packet_handler.WritePosEx(servo_id, int(position), HOLD_SPEED, HOLD_ACC)

    value, result, _ = packet_handler.read2ByteTxRx(servo_id, REG_OFFSET)
    if result == COMM_SUCCESS and decode_offset(value) != 0:
        return False, f"{name} (ID {servo_id}): the offset is still {decode_offset(value)}."
    return True, f"{name} (ID {servo_id}): offset cleared, now reading {position}."


def print_table(ids, report):
    print(f"{'ID':>3}  {'Joint':<11} {'Offset':>7} {'Degrees':>8} {'Reported':>9} {'Raw':>6} {'To seam':>8}  Note")
    for servo_id in ids:
        entry = report.get(servo_id)
        name = joint_name(servo_id)
        if entry is None:
            print(f"{servo_id:>3}  {name:<11} no answer")
            continue
        degrees = entry["offset"] * 360.0 / STEPS_PER_TURN
        note = ""
        if entry["to_wrap"] <= WRAP_MARGIN:
            note = "NEAR THE SEAM: a move here can travel the long way round"
        elif abs(entry["offset"]) > STEPS_PER_TURN / 8:
            note = "large offset"
        print(
            f"{servo_id:>3}  {name:<11} {entry['offset']:>7} {degrees:>7.0f}° "
            f"{entry['position']:>9} {entry['raw']:>6} {entry['to_wrap']:>8}  {note}"
        )


def main(argv=None):
    parser = argparse.ArgumentParser(description="Show or clear the stored position offsets.")
    parser.add_argument("--port", default=DEFAULT_PORT, help="serial device (default: %(default)s)")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUDRATE, help="baud rate (default: %(default)s)")
    parser.add_argument("--ids", default=ARM_IDS, help="servo IDs (default: %(default)s)")
    parser.add_argument("--reset", action="store_true", help="clear the offsets (writes the servo EEPROM)")
    parser.add_argument("--yes", action="store_true", help="do not ask for confirmation")
    args = parser.parse_args(argv)

    ids = parse_ids(args.ids)
    try:
        port_handler, packet_handler = open_bus(args.port, args.baud)
    except RuntimeError as exc:
        print(exc)
        return 1
    print(f"Connected to {args.port} @ {args.baud} bps\n")

    try:
        report = read_offsets(packet_handler, ids)
        print_table(ids, report)
        if not report:
            print("\nNo servo answered.")
            return 1

        if not args.reset:
            near = [servo_id for servo_id, entry in report.items() if entry["to_wrap"] <= WRAP_MARGIN]
            if near:
                print(f"\nJoints sitting at the seam: {sorted(near)}. Run again with --reset to clear the offsets.")
            else:
                print("\nNo joint is near the seam. Run again with --reset if you want to clear the offsets anyway.")
            return 0

        print("\nClearing the offsets changes the position each servo reports.")
        print("The torque stays on and each joint is held where it is, so the arm should not move.")
        print("Hold the arm anyway, then put it in its neutral pose and zero it once afterwards.")
        if not args.yes and input("Type 'yes' to continue: ").strip().lower() != "yes":
            print("Cancelled. Nothing changed.")
            return 1

        print()
        failed = []
        for servo_id in sorted(report):
            ok, message = clear_offset(packet_handler, servo_id)
            print(f"  {message}")
            if not ok:
                failed.append(servo_id)

        print()
        print_table(ids, read_offsets(packet_handler, ids))
        if failed:
            print(f"\nOffsets NOT cleared on {failed}.")
            return 1
    finally:
        port_handler.closePort()

    print(f"\nEvery offset is 0 and each joint reports its raw encoder position ({POSITION_MIDDLE} is the middle).")
    print("Next: put the arm in its neutral pose and zero it ONCE, with the app or")
    print("python3 src/servo_positions.py --set-zero. Do not repeat the zeroing.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
