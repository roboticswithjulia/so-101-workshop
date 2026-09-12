#!/usr/bin/env python3
"""Example helper for checking a Feetech STS3215 serial servo bus.

This script is intentionally simple and workshop-friendly. It does not assume a
particular controller library; it focuses on the setup steps and validation that
matter when working with serial smart servos.
"""

import sys

try:
    import serial
except ImportError:  # pragma: no cover
    serial = None


def list_serial_ports():
    if serial is None:
        print("pyserial is not installed. Install it with: pip install pyserial")
        return []

    ports = []
    for port in range(0, 20):
        try:
            candidate = f"/dev/ttyACM{port}"
            with serial.Serial(candidate, timeout=0.2):
                ports.append(candidate)
        except Exception:
            pass

    try:
        candidate = "/dev/ttyAMA0"
        with serial.Serial(candidate, timeout=0.2):
            ports.append(candidate)
    except Exception:
        pass

    return sorted(set(ports))


def print_setup_instructions():
    print("Feetech STS3215 configuration checklist")
    print("- Use a serial bus, not PWM")
    print("- Use a stable 7.4V power source for the motors")
    print("- Share GND between the controller and all servos")
    print("- Set each servo to a unique ID")
    print("- Use the Feetech/STSC protocol")
    print("- Set baud rate to 1,000,000 bps if supported")
    print("- Configure position mode and disable unsafe motion before calibration")
    print("- Define a safe home pose before running workshop tasks")


def main():
    print_setup_instructions()
    ports = list_serial_ports()
    if not ports:
        print("\nNo serial device detected. Check the wiring and power supply.")
        return 1

    print("\nDetected serial ports:")
    for port in ports:
        print(f"- {port}")

    print("\nNext steps:")
    print("1. Connect one servo at a time to verify communication")
    print("2. Read the servo ID and confirm the bus protocol")
    print("3. Set unique IDs for each joint")
    print("4. Calibrate zero positions and save home pose")
    return 0


if __name__ == "__main__":
    sys.exit(main())
