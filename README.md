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

- `app/` – desktop applications
  - `app/app.py` – main desktop UI
  - `app/app_v2.py` – one-button app: store the current pose as the zero pose (same as `servo_positions.py --set-zero`)
- `src/` – source code: robot logic and servo tools
  - `src/robot_controller.py` – safe controller logic
  - `src/mock_robot.py` – simulation backend for demos and tests
  - `src/real_robot.py` – adapter for the real arm on top of the Feetech SDK
  - `src/so101_bus.py` – shared helpers on top of the Feetech SDK (open the bus, parse IDs, register map)
  - `src/servo_scan.py` – find the serial port and the servo IDs (ping)
  - `src/servo_config.py` – read and check the configuration of each servo
  - `src/servo_positions.py` – show the current position of each joint
  - `src/servo_wiring.py` – wiring and safety diagram
- `tests/` – validation tests
- `docs/` – documentation: `FTSERVO_SDK.md` (Feetech SDK reference), `context.md` (workshop context) and the workshop proposal PDF

All servo communication goes through the official Feetech SDK (`ftservo-python-sdk`, imported as `scservo_sdk`). Run every script from the project folder; the apps and tools can be run directly (`python3 app/app.py`) or as modules (`python3 -m app.app`, `python3 -m src.servo_scan`).

## Requirements

### Raspberry Pi 3 B

- Raspberry Pi OS (recommended)
- Python 3
- Tkinter
- the Python packages in `requirements.txt`: `pyserial` and the official Feetech SDK `ftservo-python-sdk` (imported as `scservo_sdk`)

Install them from the project folder:

```bash
pip install -r requirements.txt
```

On Raspberry Pi OS Bookworm, pip refuses to install into the system Python. Use a virtual environment:

```bash
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install -r requirements.txt
```

The `--system-site-packages` flag keeps the system Tkinter visible inside the virtual environment.

### Hardware

- LEERobot SO-101 arm
- Feetech STS3215 servos
- 7.4V power source for the motors
- proper serial bus wiring
- common ground between controller and servos

## Run the app

From the project folder:

```bash
python3 app/app.py
```

### App v2: zero pose only

A minimal window (Catalan interface) with a single button for instructors. Put the arm in its neutral pose with the gripper open, press "Fixar la posició actual com a zero", confirm, and every servo stores its current position as its zero (2048). The result per servo is shown in the window.

If the app reports "permís denegat" on the serial port, your user must be in the `dialout` group. Run `sudo usermod -aG dialout $USER` once, then **log out and log back in** (or reboot). The new group only applies to sessions started after the change, so a terminal or VS Code window that was already open keeps failing. To run once without logging out: `sg dialout -c "python3 app/app_v2.py"`.

```bash
python3 app/app_v2.py                          # /dev/ttyACM0, IDs 1..6
python3 app/app_v2.py --port /dev/ttyUSB0 --ids 1,2,3
```

## Run the tests

```bash
python3 -m unittest discover -s tests -t . -v
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
python3 src/servo_scan.py
```

It pings every ID with the SDK's `ping` call on each USB serial port, at 1,000,000 bps first and then at 500,000 and 115,200 bps, and prints the model number of every servo that answers.

```bash
python3 src/servo_scan.py --ids 1-6              # quick check of the six arm servos (about 1 s)
python3 src/servo_scan.py --port /dev/ttyACM0    # scan one port only
python3 src/servo_scan.py --baud 115200          # servos configured at another speed
```

This is useful for:

- confirming the correct serial device is connected
- checking the wiring before powering the arm
- identifying the communication port and the servo IDs

### 2. Servo configuration check

Reads the configuration registers of each servo and compares them with the workshop setup. Nothing is written.

```bash
python3 src/servo_config.py                      # /dev/ttyACM0, IDs 1..6
python3 src/servo_config.py --port /dev/ttyUSB0 --ids 1,2,3
```

The table shows, per servo: model, baud rate, mode, angle limits, offset, torque state, position, voltage and temperature. Warnings are printed for a baud rate other than 1,000,000 bps, a mode other than position, torque off, voltage outside 6.0 to 8.4 V, high temperature, and IDs that do not answer.

### 3. Servo positions

Shows the raw position (0..4095), degrees, speed and moving state of each joint.

```bash
python3 src/servo_positions.py                   # one reading
python3 src/servo_positions.py --loop            # refresh every 0.5 s until Ctrl+C
python3 src/servo_positions.py --set-zero        # take the current pose as the zero pose
```

`--set-zero` first prints the current positions and asks for confirmation. It then tells each servo to treat its current position as the middle of its range, so every joint reads 2048 in that pose. The servo stores the offset in its EEPROM, so it survives power cycles and replaces any previous zero. Put the arm in its neutral pose with the gripper open before running it, and use it only from the instructor account.

### 4. Servo wiring guide

Print the wiring reference for the arm and the serial bus layout.

```bash
python3 src/servo_wiring.py
```

This tool shows:

- power and ground layout
- serial bus chain
- recommended joint mapping
- safety rules for beginner users

## Serial protocol notes

The scripts do not build packets themselves. They use the official Feetech SDK (`scservo_sdk`): `PortHandler` opens the serial port and `sms_sts` implements the STS/SMS protocol (ping, read and write registers, sync read/write). `so101_bus.py` wraps the two in `open_bus()` and holds the STS3215 register addresses the tools read. See `docs/FTSERVO_SDK.md` for the SDK reference.

For reference, the wire format is Dynamixel-style: header `0xFF 0xFF`, servo ID, length, instruction, parameters, checksum. A ping to ID 1 is `FF FF 01 02 01 FB` and the servo answers `FF FF 01 02 00 FC`. The `0x55 0x55` header used by Hiwonder/LewanSoul servos is a different protocol and STS3215 servos ignore it.

If a tool fails with "Permission denied" on the serial port, add your user to the `dialout` group and log in again:

```bash
sudo usermod -aG dialout $USER
```

## Raspberry Pi live setup

To use the real robot path on the Pi:

1. Connect the STS3215 bus to the Pi serial adapter or USB serial converter.
2. Make sure all servo grounds share a common ground with the controller.
3. Power the servos from a stable 7.4V supply.
4. Run:

```bash
python3 src/servo_scan.py
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
3. Run `servo_scan.py` to identify the serial port and the servo IDs
4. Check each servo one by one on the bus
5. Assign the correct IDs and configure the bus speed
6. Run `servo_config.py` and fix any warning it prints
7. Run `servo_positions.py` and confirm the home pose is safe
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
- **Set current pose as zero**: a check box that stores the current position of every motor as its zero position (2048). Ticking it opens a confirmation dialog; on "yes" the app writes the zero to the servos (the same operation as `servo_positions.py --set-zero`) and reports the result in the status area. The box clears itself afterwards. In demo mode the operation is simulated.

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
