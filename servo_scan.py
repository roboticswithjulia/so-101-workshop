#!/usr/bin/env python3
"""Find the serial port and the IDs of the Feetech STS3215 servos.

Uses the official Feetech SDK (`scservo_sdk`, installed from requirements.txt).
Each ID is pinged; a servo that answers also reports its model number.

Usage examples:

    python3 servo_scan.py                       # USB ports, IDs 1..253, 1M/500k/115200 bps
    python3 servo_scan.py --ids 1-6             # quick check of the arm (about 1 second)
    python3 servo_scan.py --port /dev/ttyACM0   # one port only
    python3 servo_scan.py --baud 115200         # servos configured at another speed
"""

import argparse
import sys

from so101_bus import COMM_SUCCESS, describe, discover_ports, open_bus, parse_ids

# The STS3215 factory default is 1,000,000 bps. The others are common values that
# a servo may have been reconfigured to.
DEFAULT_BAUDRATES = (1000000, 500000, 115200)


def scan_ids(packet_handler, ids, log=None):
    """Ping every ID. Returns {id: model_number} for the servos that answered."""
    found = {}
    for servo_id in ids:
        model, result, error = packet_handler.ping(servo_id)
        if result == COMM_SUCCESS:
            found[servo_id] = int(model)
            if log and error:
                log(f"    ID {servo_id} reports: {describe(packet_handler, result, error)}")
    return found


def scan_port(port, baudrate, ids, log=None):
    port_handler, packet_handler = open_bus(port, baudrate)
    try:
        return scan_ids(packet_handler, ids, log=log)
    finally:
        port_handler.closePort()


def main(argv=None):
    parser = argparse.ArgumentParser(description="Scan a Feetech STS3215 servo bus.")
    parser.add_argument("--port", help="serial device to scan (default: all USB serial ports)")
    parser.add_argument("--baud", type=int, help="baud rate (default: try %s)" % ", ".join(map(str, DEFAULT_BAUDRATES)))
    parser.add_argument("--ids", default="1-253", help="IDs to ping, e.g. '1-6' or '1,2,3' (default: 1-253)")
    args = parser.parse_args(argv)

    ports = [args.port] if args.port else discover_ports()
    baudrates = [args.baud] if args.baud else list(DEFAULT_BAUDRATES)
    ids = parse_ids(args.ids)

    print("Scanning for Feetech STS3215 servos...")
    if not ports:
        print("No serial port detected.")
        print("Check that the bus servo adapter is plugged into the Raspberry Pi USB port.")
        return 1
    print("Ports: " + ", ".join(ports))

    found = {}
    for port in ports:
        for baudrate in baudrates:
            print(f"  {port} @ {baudrate} bps, IDs {ids[0]}..{ids[-1]} ...", end="", flush=True)
            try:
                detected = scan_port(port, baudrate, ids, log=print)
            except RuntimeError as exc:
                print(f" failed: {exc}")
                break
            if detected:
                print(" found:")
                for servo_id, model in detected.items():
                    print(f"    ID {servo_id:<3} model {model}")
                found[(port, baudrate)] = detected
                break
            print(" nothing")

    if not found:
        print("\nNo STS3215 response detected on the serial bus.")
        print("Check, in this order:")
        print("1. The servo power supply is on (7.4V, the adapter LED / servo LEDs light up).")
        print("2. Only ONE cable path: adapter -> servo -> servo ... (no loops).")
        print("3. The 3-pin cable orientation on the adapter and on each servo.")
        print("4. Try a single servo directly on the adapter.")
        return 1

    print("\nRecommended next steps:")
    print("1. Verify each servo has a unique ID (1..6 for the SO-101).")
    print("2. Run python3 servo_config.py to review the configuration of each servo.")
    print("3. Run python3 servo_positions.py to watch the joint positions.")
    print("4. Keep an emergency stop ready while testing.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
