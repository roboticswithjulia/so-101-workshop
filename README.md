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

- desktop app (Python and Tkinter, Catalan interface) with two tabs: joint sliders with the live angle of each servo, and a code editor where teams complete the coordinates of ready-made movement functions
- a task player with Executar, Pas a pas, Pausa and Aturar that runs the team's program
- command line tools to find the servos, check their configuration and watch their positions
- a real robot adapter and a simulated robot with the same interface, ready for a future workshop UI
- all servo communication through the official Feetech SDK
- unit tests that run without hardware

## Project structure

- `app/` – desktop application
  - `app/app.py` – the whole app: Control tab with the joint sliders, and Programa tab where the task is written and run
- `src/` – source code: robot logic and servo tools
  - `src/robot_controller.py` – safe controller logic (used by the tests and by a future workshop UI)
  - `src/mock_robot.py` – simulated robot with the same interface as the real one
  - `src/real_robot.py` – adapter for the real arm on top of the Feetech SDK
  - `src/so101_bus.py` – shared helpers on top of the Feetech SDK (open the bus, parse IDs, register map)
  - `src/servo_scan.py` – find the serial port and the servo IDs (ping)
  - `src/servo_set_id.py` – change the ID of one servo (EEPROM)
  - `src/servo_offsets.py` – show or clear the stored position offsets
  - `src/servo_config.py` – read and check the configuration of each servo
  - `src/servo_positions.py` – show the current position of each joint
  - `src/servo_wiring.py` – wiring and safety diagram
- `logo/` – the JMRobotics logo: `Logo_JMR.svg` is the source, `Logo_JMR.png` is what the app shows
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

A Catalan window for instructors, on a dark grey background with the two colours of the JMRobotics logo as the accent: `#6CDAE7` for the sliders, headings and focus, and `#008B8B` for the troughs, borders and buttons. The program editor is light grey with dark text, since that is where you read and edit at length, and the log box below the status line is dark grey, one step lighter than the window. Buttons, joint names, panel titles and field labels are bold, and the buttons have rounded corners. The palette is the block of constants at the top of `app/app.py`, and `apply_dark_theme()` builds the ttk style from it. ttk has no border radius, so the buttons are drawn from a rounded image that Tk stretches; that needs Pillow, and without it the buttons are simply square. It connects to the arm when it opens (or with the "Connectar" button), holds every servo where it is and enables its torque.

**Joint sliders.** One slider per joint moves that servo as an offset in degrees from the zero pose:

| ID | Joint (Catalan) | Limits [min, max] | Direction |
|----|-----------------|-------------------|-----------|
| 1 | Base | -180° to +180° | normal |
| 2 | Espatlla | -180° to +180° | normal |
| 3 | Colze | -180° to +180° | inverted (value × -1) |
| 4 | Canell | -180° to +180° | normal |
| 5 | Gir del canell | -180° to +180° | normal |
| 6 | Pinça | -180° to +180° | normal |

> The defaults span the whole travel the servo can reach, so they stop nothing. Narrow the min and max of a joint, in the app or in the `JOINTS` table, to keep it off its mechanical stops.

Each slider runs between the limits of its joint, and the direction comes from the last column of the `JOINTS` table in `app/app.py` (+1 normal, -1 inverted); Colze is inverted. A move is sent when the slider is released, as one smooth motion. See the Control tab section below for what each column does.

From the project folder:

```bash
python3 app/app.py                             # /dev/ttyACM0, IDs 1..6
python3 app/app.py --port /dev/ttyUSB0 --ids 1,2,3
```

If the app reports "permís denegat" on the serial port, your user must be in the `dialout` group. Run `sudo usermod -aG dialout $USER` once, then **log out and log back in** (or reboot). The new group only applies to sessions started after the change, so a terminal or VS Code window that was already open keeps failing. To run once without logging out: `sg dialout -c "python3 app/app.py"`.

### Control tab

The top row has three buttons of the same size, **Connectar**, **Fixar la posició zero** and **Tornar tot a zero**, and the **Motors lliures** check box.

Each joint has a slider between its limits, the **live angle the servo actually reports**, a box to type a target angle, and its min and max limits. The live angle is re-read from the arm several times a second, so it stays right even while the motors are free and you are posing the arm by hand, which is exactly when the sliders cannot know where the arm is.

**Moviment** holds the speed and acceleration used by every move. Speed is in encoder steps per second and acceleration in units of 100 steps/s². The defaults are 750 and 50, so a 90° move takes about 1.5 seconds. A higher speed and a steeper ramp draw more current: if the servos start tripping their overload protection, or `src/servo_config.py` shows the supply voltage sagging below 7 V under load, lower both. Below 100 steps/s the servo creeps and judders, so that is the accepted minimum. Type the two values and press **Aplicar**, or Enter in either box; the status line then reports how long a 90° move will take.

The second row of the panel holds the **step durations**: how long the program waits after a move and after a gripper step, before starting the next line. There is no feedback from the servos, so a step must outlast the move it starts, but a wait longer than the move is dead time. If the arm visibly stops between moves, lower **Espera moviment**; if the gripper is still moving when the next line begins, raise **Espera pinça**. Both default to 1 s, which at the default speed covers a move of about 50°. A `segons=` argument on a line always wins over these.

**Motors lliures** turns the torque off on every servo so the arm can be posed by hand. It asks for confirmation first, because the arm drops under its own weight the moment the torque goes: hold it, and take any object out of the gripper. While the motors are free the sliders are disabled, since a goal position is stored but not obeyed. Unticking the box commands every servo the position it is currently in and only then restores the torque, so the arm holds where your hand left it instead of jumping back. Starting a program re-enables the torque automatically.

**Fixar la posició zero** stores the current pose of every servo as its zero (2048). It is written to the motor memory and replaces the previous zero, which invalidates every coordinate already written in a program, so capture the poses again after using it.

### Programa tab

This is where the task is written and run. The editor opens with the Repte 1 template: the functions are written already and teams only complete the coordinates.

```python
inici()
anar_a(base=-3, espatlla=48, colze=14, canell=-25, gir=-6)
tancar_pinca(-40)
esperar(0.5)

anar_a(base=-3, espatlla=27, colze=15, canell=-7, gir=-6)
anar_a(base=14, espatlla=22, colze=14, canell=-3, gir=-6)

anar_a(base=32, espatlla=66, colze=45, canell=-16, gir=-6)
obrir_pinca(10)

anar_a(base=31, espatlla=36, colze=18, canell=-16, gir=-6)

inici()
```

| Function | What it does |
|---|---|
| `anar_a(base, espatlla, colze, canell, gir)` | moves the arm; any joint left out stays where it is |
| `obrir_pinca(graus)` / `tancar_pinca(graus)` | moves the gripper; without an angle it uses the default open and closed values |
| `aixecar()` | goes to the intermediate carry pose |
| `inici()` | goes to the home position |
| `esperar(segons)` | waits without moving |

All of them accept `segons=` to override the default duration, and `for` loops work. Every move is written out step by step, so a team can see the whole sequence in the editor with nothing hidden inside a helper.

**Afegir la posició actual** reads the arm and writes an `anar_a(base=…, espatlla=…, colze=…, canell=…, gir=…)` line straight into the editor, below the cursor. Tick **Motors lliures** on the Control tab, move the arm by hand to where you want it, press the button, and the line appears with all five joints filled in. This is the fastest way for a team to build a sequence.

**Desar posició inici** takes the current pose as the start position, so every later `inici()` goes there instead of to the zero pose.

**Executar el programa** compiles the code, turns it into a step list and plays it. The whole task is validated before the first servo moves, so a pose outside a joint's current limits is reported in Catalan and nothing moves. Timing is open loop: each step waits a fixed number of seconds, so give a move enough time to finish, about 2.5 seconds at the default speed.

- **Pas a pas** runs one step and stops there, reporting what finished and what comes next. It is the button to use while teaching. After the last step it rewinds.
- **Pausa** freezes the arm where it is and remembers the step; Executar resumes that step from its beginning.
- **Aturar** freezes the arm and rewinds to the first step. It is live at all times, so it doubles as a hold button.

Freezing commands every servo the position it currently reads, with the torque left on, so the arm never goes limp. While a program runs, the editor, the sliders and the other buttons are disabled.

Errors are reported in Catalan: a syntax error gives the line number, an unknown function gives the name. A program is capped at 200 steps, so a runaway `while True` is caught rather than run, and the code runs with a restricted set of builtins, so imports, `open` and `eval` are unreachable. **Desar** and **Carregar** write and read the program as a `.py` file, and **Exemple** restores the template.

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

### 4. Change a servo ID

Servos arrive set to ID 1, so each joint needs its own ID before assembly. **Connect one servo at a time**: the ID lives in the servo EEPROM, and every servo answering to the old ID would be renamed at once.

```bash
python3 src/servo_set_id.py --from 1 --to 3
python3 src/servo_set_id.py --from 1 --to 3 --port /dev/ttyUSB0 --yes
```

The script scans the bus first and refuses to write when nothing answers, when the old ID is missing, when the target ID is already taken, or when more than one servo is connected. `--force` overrides those checks. It then unlocks the EEPROM, writes the ID register, locks it again on the new ID, and confirms by checking that the servo answers to the new ID and no longer to the old one. Use the SO-101 mapping: 1 base, 2 shoulder, 3 elbow, 4 wrist, 5 wrist roll, 6 gripper.

### 5. Position offsets

Each time a servo is told "take your current position as the middle", it **adds** to the offset stored in its EEPROM instead of replacing it. After a few rounds of re-zeroing, a joint drifts hundreds of steps from the encoder's native zero, and its working range can end up sitting on the 0/4095 seam where the encoder wraps. A joint there behaves erratically: a small commanded move can be interpreted as travel the long way round, or the joint can stop short because it has run out of electrical range.

```bash
python3 src/servo_offsets.py            # read only: show the offsets
python3 src/servo_offsets.py --reset    # clear them, then re-zero once
```

The table shows each joint's offset, the raw encoder reading behind its reported position, and how many steps it is from the seam. Anything within 200 steps is flagged. `--reset` clears the offsets and immediately commands each joint to hold its new reading, with the torque left on so the arm never goes limp. Afterwards put the arm in its neutral pose and zero it **once**.

### 6. Servo wiring guide

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

- "fora dels límits": a coordinate in the program is outside that joint's min/max. Change the coordinate, or widen the limit on the Control tab.
- "permís denegat": add your user to `dialout` and log out and back in (see "Run the app")
- "Cap motor ha respost": check the 7.4V supply, the cable chain and the servo IDs with `src/servo_scan.py`
- "NO aplicat" on a servo: the servo ignored the zero command; check its firmware or use the Feetech tool

## License

This project is intended for workshop and educational use.
