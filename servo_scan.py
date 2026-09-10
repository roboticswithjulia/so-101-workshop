#!/usr/bin/env python3
"""Scan serial ports and ping Feetech STS3215 servos.

This helper follows the common Feetech serial protocol and is intended for
beginner-focused workshop setups. It is conservative and will fail cleanly if no
hardware is connected.
"""

import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover
    serial = None
    list_ports = None


def ping_packet(servo_id):
    """Build a minimal Feetech ping packet."""
    packet = [0x55, 0x55, servo_id, 0x02, 0x01]
    checksum = (~(servo_id + 0x02 + 0x01)) & 0xFF
    packet.append(checksum)
    return bytes(packet)


def discover_ports():
    if list_ports is None:
        return [
            "/dev/ttyUSB0",
            "/dev/ttyUSB1",
            "/dev/ttyAMA0",
            "/dev/serial0",
        ]

    detected = []
    for port in list_ports.comports():
        detected.append(port.device)
    return sorted(set(detected))


def scan_bus(port, baudrate=1000000):
    if serial is None:
        raise RuntimeError("pyserial is not installed. Run: pip install pyserial")

    found = []
    with serial.Serial(port, baudrate=baudrate, timeout=0.05) as connection:
        for servo_id in range(1, 254):
            connection.write(ping_packet(servo_id))
            time.sleep(0.01)
            response = connection.read(8)
            if len(response) >= 6 and response[:2] == b"\x55\x55" and response[2] == servo_id:
                found.append(servo_id)
    return found


def main():
    print("Scanning serial ports for Feetech STS3215 bus...")
    ports = discover_ports()
    if not ports:
        print("No serial port detected.")
        print("Check: power, ground, and the controller connection.")
        return 1

    found = []
    for port in ports:
        try:
            detected = scan_bus(port)
            if detected:
                print(f"Detected servo IDs on {port}: {detected}")
                found.extend(detected)
        except Exception as exc:  # pragma: no cover - hardware-dependent
            print(f"Port {port} check failed: {exc}")

    if not found:
        print("No STS3215 response detected on the serial bus.")
        print("Check the servo power, common ground, and bus connection.")
        return 1

    print("\nRecommended next steps:")
    print("1. Verify each servo has a unique ID.")
    print("2. Confirm the controller uses the Feetech protocol.")
    print("3. Set the position mode for each servo.")
    print("4. Calibrate the zero positions and define the home pose.")
    print("5. Keep an emergency stop ready while testing.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
