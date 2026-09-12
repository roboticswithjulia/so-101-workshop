# LEERobot SO-101 Workshop App

Tools and a small desktop application for the LEERobot SO-101 workshop. Everything runs on a Raspberry Pi 3 B without a browser and talks to the Feetech STS3215 servos through the official Feetech SDK.

## Project goal

This project supports a 4-hour robotics workshop where newcomers learn to:

- assemble a SO-101 arm
- connect the Feetech STS3215 motors
- understand basic motion and positioning
- use a safe, guided control interface
- run basic tasks without programming knowledge

## Included features

- one-button desktop app (Python and Tkinter, Catalan interface) that stores the current pose of the arm as its zero pose
- command line tools to find the servos, check their configuration and watch their positions
- a real robot adapter and a simulated robot with the same interface, ready for a future workshop UI
- all servo communication through the official Feetech SDK
- unit tests that run without hardware

## Project structure

- `app/` – desktop application
  - `app/app.py` – one-button app: store the current pose as the zero pose (same as `servo_positions.py --set-zero`)
- `src/` – source code: robot logic and servo tools
  - `src/robot_controller.py` – safe controller logic (used by the tests and by a future workshop UI)
  - `src/mock_robot.py` – simulated robot with the same interface as the real one
  - `src/real_robot.py` – adapter for the real arm on top of the Feetech SDK
  - `src/so101_bus.py` – shared helpers on top of the Feetech SDK (open the bus, parse IDs, register map)
  - `src/servo_scan.py` – find the serial port and the servo IDs (ping)
  - `src/servo_config.py` – read and check the configuration of each servo
  - `src/servo_positions.py` – show the current position of each joint
  - `src/servo_wiring.py` – wiring and safety diagram
- `tests/` – validation tests
- `docs/` – documentation: `FTSERVO_SDK.md` (Feetech SDK reference), `context.md` (workshop context) and the workshop proposal PDF

All servo communication goes through the official Feetech SDK (`ftservo-python-sdk`, imported as `scservo_sdk`). Run every script from the project folder; the app and the tools can be run directly (`python3 app/app.py`) or as modules (`python3 -m app.app`, `python3 -m src.servo_scan`).

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

A minimal window (Catalan interface) with a single button for instructors. Put the arm in its neutral pose with the gripper open, press "Fixar la posició actual com a zero", confirm with "Sí", and every servo stores its current position as its zero (2048). The result per servo is shown in the window.

From the project folder:

```bash
python3 app/app.py                             # /dev/ttyACM0, IDs 1..6
python3 app/app.py --port /dev/ttyUSB0 --ids 1,2,3
```

If the app reports "permís denegat" on the serial port, your user must be in the `dialout` group. Run `sudo usermod -aG dialout $USER` once, then **log out and log back in** (or reboot). The new group only applies to sessions started after the change, so a terminal or VS Code window that was already open keeps failing. To run once without logging out: `sg dialout -c "python3 app/app.py"`.

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

5. If the six servos respond, put the arm in its neutral pose and run `python3 app/app.py` to store the zero pose.
6. Check with `python3 src/servo_positions.py` that every joint reads 2048 in that pose.

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
7. Put the arm in its neutral pose and store the zero pose with `app/app.py`
8. Run `servo_positions.py` and confirm every joint reads 2048 in the neutral pose

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

## Robot adapters for a future workshop UI

`src/real_robot.py` (real arm, on the Feetech SDK) and `src/mock_robot.py` (simulation) share one interface: `move_to`, `open_gripper`, `close_gripper`, `home`, `emergency_stop`, `reset_emergency_stop`, `read_positions` and `set_zero`. `src/robot_controller.py` wraps them with range checks and emergency-stop handling. Motion commands are offsets from the zero pose stored by the app, so a workshop UI can be built on top without touching the servo code.

### App reports an error

- "permís denegat": add your user to `dialout` and log out and back in (see "Run the app")
- "Cap motor ha respost": check the 7.4V supply, the cable chain and the servo IDs with `src/servo_scan.py`
- "NO aplicat" on a servo: the servo ignored the zero command; check its firmware or use the Feetech tool

## License

This project is intended for workshop and educational use.
