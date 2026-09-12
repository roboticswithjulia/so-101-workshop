# LEERobot SO-101 Workshop App

A beginner-friendly desktop application for the LEERobot SO-101 workshop. It is designed for non-expert users and runs well on a Raspberry Pi 3 B without requiring a browser-based UI.

## Project goal

This project supports a 4-hour robotics workshop where newcomers learn to:

- assemble a SO-101 arm
- connect the Feetech STS3215 motors
- understand basic motion and positioning
- use a safe, guided control interface
- run basic tasks without programming knowledge

## Included features

- desktop app built with Python and Tkinter
- beginner-safe controls with large buttons
- emergency stop and reset flow
- demo mode for training and testing
- real robot mode adapter for future hardware integration
- bilingual interface: Catalan and English
- instructor-only advanced controls panel
- task screens for pick-and-place and robotic Jenga

## Project structure

- `app.py` – main desktop UI
- `robot_controller.py` – safe controller logic
- `mock_robot.py` – simulation backend for demos and tests
- `real_robot.py` – adapter for a real robot connection
- `servo_config.py` – example servo configuration helper
- `servo_scan.py` – scan serial ports and check bus availability
- `servo_calibration.py` – step-by-step calibration guide
- `servo_wiring.py` – wiring and safety diagram
- `tests/test_robot_controller.py` – validation tests

## Requirements

### Raspberry Pi 3 B

- Raspberry Pi OS (recommended)
- Python 3
- Tkinter
- optionally: `pyserial` for serial bus work

### Hardware

- LEERobot SO-101 arm
- Feetech STS3215 servos
- 7.4V power source for the motors
- proper serial bus wiring
- common ground between controller and servos

## Run the app

From the project folder:

```bash
python3 app.py
```

## Run the tests

```bash
python3 -m unittest discover -s tests -v
```

## STS3215 servo configuration notes

The STS3215 is a smart serial servo, not a normal PWM hobby servo. For a SO-101 arm, the expected setup is:

- servo bus: serial TTL
- common ground: controller + all servos
- power: 7.4V supply suitable for the whole arm
- each servo must have a unique ID
- baud rate typically 1,000,000 bps
- mode: position control
- home pose and zero calibration must be defined before use

### Safe configuration checklist

1. Power the servos from a proper 7.4V source
2. Connect all grounds together
3. Connect the serial bus correctly
4. Verify communication with each servo
5. Set unique IDs
6. Configure position mode
7. Calibrate each joint to zero
8. Define home position
9. Test movement slowly
10. Keep emergency stop ready

## Example servo wiring concept

- Power rail: 7.4V to all servo VCC pins
- Ground: GND shared with controller
- Signal bus: daisy chained through all servos
- Motor IDs: 1..6 (or your chosen mapping)

## Hardware helper scripts

The project includes several educational helper scripts for the actual SO-101 hardware setup.

### 1. Servo scan

Use the bus scanner to inspect available serial ports and confirm the controller is seeing the communication bus.

```bash
python3 servo_scan.py
```

This helper uses a minimal Feetech ping packet and checks multiple serial ports for a response from STS3215 servos. It is intended as a practical workshop tool to confirm that the bus is live before calibration.

This is useful for:

- confirming the correct serial device is connected
- checking the wiring before powering the arm
- identifying the communication port before calibration

### 2. Servo calibration

Run the calibration guide to follow the safe zeroing procedure before any movement test.

```bash
python3 servo_calibration.py
```

This script explains:

- how to wire the servos safely
- how to set unique IDs
- how to move to the neutral position
- how to save zero offsets
- how to test each servo slowly

It also attempts a bus scan before calibration so the workflow is aligned with the hardware check.

### 3. Servo wiring guide

Print the wiring reference for the arm and the serial bus layout.

```bash
python3 servo_wiring.py
```

This tool shows:

- power and ground layout
- serial bus chain
- recommended joint mapping
- safety rules for beginner users

## Real serial protocol notes

The scripts use the Feetech STS/SCS serial protocol (Dynamixel-style) used by STS3215 smart servos:

- packet header: `0xFF 0xFF` (note: `0x55 0x55` is the Hiwonder/LewanSoul protocol and STS3215 servos ignore it)
- servo ID field
- length field (number of parameters + 2)
- instruction field (`0x01` ping, `0x02` read, `0x03` write)
- parameter bytes
- checksum byte: `~(id + length + instruction + params) & 0xFF`

A ping to ID 1 is `FF FF 01 02 01 FB` and the servo answers `FF FF 01 02 00 FC`. Many USB bus adapters are half-duplex and echo the transmitted bytes back, so the scanner strips its own packet before looking for the reply.

Useful scanner options:

```bash
python3 servo_scan.py --ids 1-6              # quick check of the six arm servos
python3 servo_scan.py --port /dev/ttyACM0    # scan one port only
python3 servo_scan.py --baud 115200          # servos configured at another speed
```

If the port check fails with "Permission denied", add your user to the `dialout` group and log in again:

```bash
sudo usermod -aG dialout $USER
```

The live adapter in `real_robot.py` now performs:

- serial port discovery
- connection attempts to likely Pi serial ports
- a ping scan across the servo IDs
- safe servo position commands for a first-stage workshop integration

> This implementation is intentionally conservative for a beginner workshop environment. It is designed to work as a practical base layer for the SO-101, but the exact servo model, adapter board, and firmware may still require a small adjustment depending on the final hardware setup.

## Raspberry Pi live setup

To use the real robot path on the Pi:

1. Connect the STS3215 bus to the Pi serial adapter or USB serial converter.
2. Make sure all servo grounds share a common ground with the controller.
3. Power the servos from a stable 7.4V supply.
4. Run:

```bash
python3 servo_scan.py
```

5. If a servo responds, run the app in real mode from the desktop interface.
6. Use the emergency stop before each motion test.

This is the recommended live workflow before any broader arm movement.

## Typical SO-101 servo mapping

- ID 1 -> Base rotation
- ID 2 -> Shoulder
- ID 3 -> Elbow
- ID 4 -> Wrist
- ID 5 -> Wrist roll
- ID 6 -> Gripper

## Safe usage workflow

1. Power the arm from a stable 7.4V source
2. Verify the ground is common for all servos and controller
3. Run `servo_scan.py` to identify the serial port
4. Check each servo one by one on the bus
5. Assign the correct IDs and configure the bus speed
6. Run `servo_calibration.py` and set the zero positions
7. Confirm the home pose is safe
8. Only then test motion in the workshop UI

## Troubleshooting

### No serial device detected

- check the power source
- verify the bus wiring and shared ground
- test one servo at a time
- confirm the controller board is connected to the serial port

### Servos jitter or lose position

- improve the 7.4V power supply quality
- verify GND continuity
- reduce simultaneous motion after startup
- confirm baud rate and protocol match the servo type

### Arm does not move after calibration

- check the servo IDs are unique
- verify the calibration step was completed correctly
- confirm the software is sending the expected commands
- ensure emergency stop is reset before movement

## Safety rules

- never power the arm from the Pi directly
- keep hands away from moving joints
- check the workspace before each motion
- always stop the robot if a joint stalls or overheats
- calibration should normally be done by instructors only

## Instructor mode

The app includes a hidden instructor panel for advanced actions such as:

- reset emergency stop
- diagnostics
- calibration reminders

This keeps the main beginner interface simple and safe.

## Future hardware integration

The `RealRobotAdapter` class is included as a clean abstraction layer. This makes it easier to connect the desktop app to the real SO-101 once the serial setup is ready.

## Troubleshooting

### App opens but robot does not move

- check whether demo mode is active
- verify the real robot adapter is connected
- confirm the servo bus is powered and grounded
- validate the serial connection and servo IDs

### Servos jitter or reset

- check the power supply rating
- verify the shared ground
- reduce the number of simultaneous motions
- confirm the bus speed matches the servo protocol

## License

This project is intended for workshop and educational use.
