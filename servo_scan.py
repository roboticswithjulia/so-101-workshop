#!/usr/bin/env python3
"""Scan serial ports and ping Feetech STS3215 servos.

Feetech STS/SCS servos use the Dynamixel-style protocol:

    0xFF 0xFF <id> <length> <instruction> <params...> <checksum>

with checksum = ~(id + length + instruction + params) & 0xFF.
A ping (instruction 0x01) is answered with a status packet:

    0xFF 0xFF <id> 0x02 <error> <checksum>

Usage examples:

    python3 servo_scan.py                       # scan every port, IDs 1..253
    python3 servo_scan.py --port /dev/ttyACM0   # one port only
    python3 servo_scan.py --ids 1-6             # quick check of the arm
    python3 servo_scan.py --baud 115200         # servos configured at another speed
"""

import argparse
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover
    serial = None
    list_ports = None

HEADER = b"\xff\xff"
INSTRUCTION_PING = 0x01
BROADCAST_ID = 0xFE

# The STS3215 factory default is 1,000,000 bps. The others are common values that
# a servo may have been reconfigured to.
DEFAULT_BAUDRATES = (1000000, 500000, 115200)


def checksum(payload):
    """Feetech checksum over id, length, instruction and parameters."""
    return (~sum(payload)) & 0xFF


def build_packet(servo_id, instruction, params=()):
    body = [servo_id, len(params) + 2, instruction, *params]
    return HEADER + bytes(body) + bytes([checksum(body)])


def ping_packet(servo_id):
    """Build a Feetech ping packet for one servo."""
    return build_packet(servo_id, INSTRUCTION_PING)


def parse_status(data, servo_id):
    """Return True when `data` contains a valid status packet from `servo_id`.

    Half-duplex adapters often echo the bytes we transmitted, so the echoed ping
    packet is stripped before looking for the reply.
    """
    echo = ping_packet(servo_id)
    if data.startswith(echo):
        data = data[len(echo):]

    marker = HEADER + bytes([servo_id])
    start = data.find(marker)
    while start != -1:
        if len(data) >= start + 4:
            length = data[start + 3]
            end = start + 4 + length
            if len(data) >= end:
                body = data[start + 2:end - 1]
                if data[end - 1] == checksum(body):
                    return True
        start = data.find(marker, start + 1)
    return False


def discover_ports():
    if list_ports is None:
        return [
            "/dev/ttyUSB0",
            "/dev/ttyUSB1",
            "/dev/ttyACM0",
            "/dev/ttyACM1",
            "/dev/ttyAMA0",
            "/dev/serial0",
        ]

    detected = sorted({port.device for port in list_ports.comports()})
    # USB adapters (Waveshare bus servo adapter, Feetech URT-1, CH340/CP2102
    # dongles) are far more likely than the Pi's built-in UART, so try them first.
    usb = [p for p in detected if "USB" in p or "ACM" in p]
    other = [p for p in detected if p not in usb]
    return usb + other


def ping(connection, servo_id, wait=0.02):
    connection.reset_input_buffer()
    connection.write(ping_packet(servo_id))
    connection.flush()
    deadline = time.monotonic() + wait
    data = b""
    while time.monotonic() < deadline:
        chunk = connection.read(connection.in_waiting or 1)
        if chunk:
            data += chunk
            if parse_status(data, servo_id):
                return True
    return parse_status(data, servo_id)


def scan_bus(port, baudrate=1000000, ids=range(1, 254)):
    if serial is None:
        raise RuntimeError("pyserial is not installed. Run: pip install pyserial")

    found = []
    with serial.Serial(port, baudrate=baudrate, timeout=0.005) as connection:
        for servo_id in ids:
            if ping(connection, servo_id):
                found.append(servo_id)
    return found


def parse_ids(text):
    ids = set()
    for part in text.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            low, high = part.split("-", 1)
            ids.update(range(int(low), int(high) + 1))
        else:
            ids.add(int(part))
    return sorted(i for i in ids if 0 <= i < BROADCAST_ID)


def main(argv=None):
    parser = argparse.ArgumentParser(description="Scan a Feetech STS3215 servo bus.")
    parser.add_argument("--port", help="serial device to scan (default: all detected)")
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
                detected = scan_bus(port, baudrate=baudrate, ids=ids)
            except Exception as exc:  # pragma: no cover - hardware-dependent
                print(f" failed: {exc}")
                if "ermission" in str(exc):
                    print("    Add your user to the dialout group and log in again:")
                    print("    sudo usermod -aG dialout $USER")
                break
            if detected:
                print(f" found IDs {detected}")
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
    print("2. Use the port and baud rate above in the app's real robot mode.")
    print("3. Calibrate the zero positions and define the home pose.")
    print("4. Keep an emergency stop ready while testing.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
