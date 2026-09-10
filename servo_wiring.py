#!/usr/bin/env python3
"""Print a simple SO-101 / STS3215 wiring guide for workshop use.

This script does not connect to hardware. It is intended to help beginners and
instructors review the safe wiring pattern before powering the robotic arm.
"""

WIRING_GUIDE = """
LEERobot SO-101 + STS3215 wiring guide
=====================================

Power side
----------
  7.4V supply  ---->  Servo VCC (all servos)
  GND         ---->  Servo GND (all servos)
  GND         ---->  Controller GND

Signal side
-----------
  Controller TX ----> Servo RX / serial bus line
  Controller RX ----> Servo TX / serial bus line
  All servos share the same serial data line in daisy chain

Important safety notes
----------------------
- Do not power the servos directly from the Raspberry Pi GPIO pins.
- Keep the power supply current rating above the combined servo load.
- Use a common ground between the controller and all servos.
- Connect one servo at a time for testing before wiring the full arm.
- Set a unique ID for each servo before using the arm.
- Keep the arm in a safe, clear workspace before calibration.

Typical joint mapping
---------------------
  ID 1  -> Base rotation
  ID 2  -> Shoulder
  ID 3  -> Elbow
  ID 4  -> Wrist
  ID 5  -> Wrist roll
  ID 6  -> Gripper

Suggested bus setup
-------------------
- Protocol: Feetech/STSC serial servo protocol
- Baud rate: 1,000,000 bps (when supported)
- Mode: position control
- Home position: define before workshop use
- Emergency stop: always available before motion

Example chain layout
--------------------
  +7.4V ---- Servo1 VCC ---- Servo2 VCC ---- Servo3 VCC ---- Servo4 VCC ---- Servo5 VCC ---- Servo6 VCC
      |                                                           |
      +-------------------------- GND common ----------------------+

  Controller TX/RX <----> Serial bus line <----> Servo1 <----> Servo2 <----> Servo3 <----> Servo4 <----> Servo5 <----> Servo6

"""


def main():
    print(WIRING_GUIDE)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
