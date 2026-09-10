#!/usr/bin/env python3
"""Calibration guide and helper for STS3215 servos.

This script explains the calibration process and uses the same serial port
inspection method as the bus scanner so the workflow is consistent across the
project.
"""

from servo_scan import discover_ports, scan_bus

CALIBRATION_STEPS = """
STS3215 calibration procedure for SO-101
=======================================

1. Power the servos from a stable 7.4V source.
2. Verify all grounds are common between the controller and all servos.
3. Confirm that the serial bus is connected correctly.
4. Run the bus scan and verify that each servo is reachable.
5. Set each servo to a unique ID.
6. Set the bus speed to the supported Feetech protocol speed (typically 1,000,000 bps).
7. Put each servo in position mode.
8. Move the joints to their mechanical zero / home pose.
9. Record the zero angle for each servo.
10. Save the zero offset in the software.
11. Test each joint individually at slow speed.
12. Define the final home position for the robot arm.
13. Validate the group of servos before running bigger tasks.

Safety check
------------
- Keep hands away from moving joints.
- Use an emergency stop during testing.
- Do not run large motions before all joints are calibrated.
- Stop immediately if a servo stalls or overheats.

Recommended servo mapping
-------------------------
  ID 1 -> Base
  ID 2 -> Shoulder
  ID 3 -> Elbow
  ID 4 -> Wrist
  ID 5 -> Wrist roll
  ID 6 -> Gripper

"""


def main():
    print(CALIBRATION_STEPS)
    ports = discover_ports()
    if not ports:
        print("No serial ports detected. Connect the controller before calibration.")
        return 1

    print("Attempting a bus scan before calibration...\n")
    for port in ports:
        try:
            ids = scan_bus(port)
            if ids:
                print(f"Bus scan on {port}: detected IDs {ids}")
                break
        except Exception as exc:  # pragma: no cover - hardware dependent
            print(f"Failed to scan {port}: {exc}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
