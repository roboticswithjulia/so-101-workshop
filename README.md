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

- desktop app (Python and Tkinter, Catalan interface) with two tabs: joint sliders plus a zero-pose button, and a code editor where teams complete the coordinates of ready-made `agafar()` / `deixar()` functions
- a task player with Play, Pause and Stop that runs either the task block or the team's program
- command line tools to find the servos, check their configuration and watch their positions
- a real robot adapter and a simulated robot with the same interface, ready for a future workshop UI
- all servo communication through the official Feetech SDK
- unit tests that run without hardware

## Project structure

- `app/` – desktop application
  - `app/app.py` – the whole app: joint sliders, zero pose, task player and the program tab; the default task is the `TASCA` block at the top of the file
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

A Catalan window for instructors. It connects to the arm when it opens (or with the "Connectar" button), holds every servo where it is and enables its torque.

**Joint sliders.** One slider per joint moves that servo as an offset in degrees from the zero pose:

| ID | Joint (Catalan) | Limits [min, max] | Direction |
|----|-----------------|-------------------|-----------|
| 1 | Base | -75° to +75° | normal |
| 2 | Espatlla | -75° to +60° | normal |
| 3 | Colze | -25° to +75° | inverted (value × -1) |
| 4 | Canell | -70° to +18° | normal |
| 5 | Gir del canell | -75° to +75° | normal |
| 6 | Pinça | -25° to +45° | normal |

Each slider runs between the limits of its joint. To the right of each slider there are three boxes: the target position in degrees (type a value such as `45` or `-12.5` and press Enter and the joint moves there; a value outside the limits is refused), and the minimum and maximum limits (type a new value and press Enter: the slider is rescaled and later moves are clamped to the new limits). The defaults are the table above. The last column of the `JOINTS` table in `app/app.py` sets the direction of each joint (+1 normal, -1 inverted); Colze is inverted. "Tornar tot a zero" moves every joint back to the zero pose. "Desar posicions" asks for a file and writes the current position of every servo to it as JSON: raw position, degrees from the zero pose, whether the joint is inverted, and its limits, plus the port and a timestamp. A move is sent when the slider is released (one smooth motion at about 9 rpm). The ranges are the `JOINTS` table and the speed the `SAFE_SPEED` / `SAFE_ACC` constants at the top of `app/app.py`; if a joint judders, raise the speed, and if it moves too fast, lower it.

**Zero pose.** Put the arm in its neutral pose with the gripper open, press "Fixar la posició actual com a zero", confirm with "Sí", and every servo stores its current position as its zero (2048). The result per servo is shown in the window and the sliders reset to 0°.

From the project folder:

```bash
python3 app/app.py                             # /dev/ttyACM0, IDs 1..6
python3 app/app.py --port /dev/ttyUSB0 --ids 1,2,3
```

If the app reports "permís denegat" on the serial port, your user must be in the `dialout` group. Run `sudo usermod -aG dialout $USER` once, then **log out and log back in** (or reboot). The new group only applies to sessions started after the change, so a terminal or VS Code window that was already open keeps failing. To run once without logging out: `sg dialout -c "python3 app/app.py"`.

### Pick-and-place task

The workshop challenge, picking an object at a start position and placing it at an end position, is programmed by editing one block at the top of `app/app.py`, marked `# ===== TASCA: EDITA AQUÍ =====`. Teams change four poses (`POSICIO_ZERO`, `POSICIO_AGAFAR`, `POSICIO_INTERMITJA`, `POSICIO_DEIXAR`), the two gripper values (`PINCA_OBERTA`, `PINCA_TANCADA`) and the list of steps in `TASCA`:

```python
POSICIO_AGAFAR = {1: 5, 2: 43, 3: 27, 4: 0, 5: 0}

TASCA = [
    ("Inici",             POSICIO_ZERO,       2.0),
    ("Obrir pinça",       PINCA_OBERTA,       1.0),
    ("Anar a l'objecte",  POSICIO_AGAFAR,     2.5),
    ("Tancar pinça",      PINCA_TANCADA,      1.0),
    ("Aixecar l'objecte", POSICIO_INTERMITJA, 2.5),
    ("Anar al destí",     POSICIO_DEIXAR,     2.5),
    ("Obrir pinça",       PINCA_OBERTA,       1.0),
    ("Tornar a l'inici",  POSICIO_ZERO,       2.0),
]
```

`POSICIO_INTERMITJA` is the lift-and-carry pose between picking and placing: the arm raised with the object held, high enough not to drag it across the table. Capture it with the object in the gripper.

A pose is `{servo id: degrees from the zero pose}`, and a joint left out of a pose stays where it is. A step is `("name", pose, seconds)`, and the pose may be a bare number, which moves the gripper (servo 6) only, so an arm move never reopens the gripper and drops the object. Restart the app after editing the block.

The "Tasca: agafar i deixar un objecte" panel has three buttons:

- **Executar** validates the whole task first. A pose outside the current limits of a joint is reported in Catalan and nothing moves, so a wrong number is never silently clamped. It then runs the steps in order, waiting the fixed number of seconds each step declares. There is no feedback from the servos, so give a step enough time for the arm to arrive: about 2 s for a 90° move at the default speed.
- **Pas a pas** runs one step and stops there, reporting which step just finished and which comes next. Press it again for the following step, or Executar to run the rest without stopping. It is the button to use while teaching: the class sees exactly what each line of the program does. After the last step it rewinds, so pressing it again starts the task from the beginning.
- **Pausa** freezes the arm where it is and remembers the step. Pressing Executar again re-sends that step from its beginning with its full duration.
- **Aturar** freezes the arm and rewinds to the first step. It is live at all times, so it doubles as a hold button.

Freezing commands every servo the position it currently reads, with the torque left on, so the arm never goes limp. While a task is playing or paused the joint sliders, the limit boxes and the zero and save buttons are disabled, so nothing else writes to the servo bus.

**Moviment** holds the speed and acceleration used by every move: the sliders, the task and the program. Speed is in encoder steps per second and acceleration in units of 100 steps/s². The defaults are 300 and 15, so a 90° move takes about 3.6 seconds. A higher speed and a steeper ramp draw more current: if the servos start tripping their overload protection, or `src/servo_config.py` shows the supply voltage sagging below 7 V under load, halve both values. Type a value and press Enter; the status line reports how long a 90° move will then take. Below 100 steps/s the servo creeps and judders instead of turning smoothly, so that is the accepted minimum. If a task step ends before the arm arrives, either give the step more seconds or raise the speed.

**Carregar posicions** reads a file written by "Desar posicions" and moves the arm to the pose it holds. It lists the angles and asks for confirmation before moving, and refuses if any of them falls outside the joint's current limits, so a file saved with different limits cannot drive a joint past them. The two files in `config/` are a neutral pose and a pose reaching a block, captured during setup. The file stores both the raw position and the angle, and the angle is what gets replayed, so it goes through the same limit and direction handling as every other move and lands on the same step it was saved at.

**Motors lliures** is a check box in the top row that turns the torque off on every servo, so the arm can be posed by hand. It asks for confirmation first, because the arm drops under its own weight the moment the torque goes: hold it, and take any object out of the gripper. While the motors are free the joint sliders are disabled, since a goal position is stored but not obeyed. Unticking the box commands every servo the position it is currently in and only then restores the torque, so the arm holds where your hand left it instead of jumping back. Starting a task or a program re-enables the torque automatically.

**Copiar la posició actual** reads the arm and prints a ready-to-paste Python literal such as `{1: 6, 2: 44, 3: 27, 4: 0, 5: 0}` in the log box, and copies it to the clipboard. Move the arm by hand or with the sliders to the pick or place pose, press the button, and paste the line into `POSICIO_AGAFAR` or `POSICIO_DEIXAR`. The gripper is reported on its own line, for `PINCA_OBERTA` and `PINCA_TANCADA`. The numbers match the `degrees` field written by "Desar posicions", so `config/positions_block.json` can also be used as a source.

> Pressing "Fixar la posició actual com a zero" moves the zero pose, which invalidates every number in `TASCA`. Capture the poses again after re-zeroing.

### Programming tab

The "Programa" tab is the Repte 1 template from the workshop proposal: the functions are written already and teams only complete the coordinates. The editor opens with an example and the code can be edited, saved and loaded from inside the app.

```python
obrir_pinca()
inici()

# Baixar a l'objecte i agafar-lo
anar_a(base=-7, espatlla=30, colze=17)
tancar_pinca()

# Aixecar, girar cap al destí i baixar
anar_a(base=-7, espatlla=17, colze=15)
anar_a(base=49, espatlla=17, colze=15)
anar_a(base=49, espatlla=29, colze=15)

# Deixar l'objecte i retirar-se
obrir_pinca()
anar_a(base=49, espatlla=17, colze=15)

inici()
```

Available functions, all taking degrees relative to the zero pose:

| Function | What it does |
|---|---|
| `agafar(base, espatlla, colze)` | opens the gripper, moves to the object, closes the gripper and lifts |
| `deixar(base, espatlla, colze)` | moves to the destination, opens the gripper and lifts |
| `anar_a(base, espatlla, colze)` | moves the arm only |
| `obrir_pinca()` / `tancar_pinca()` | moves the gripper only |
| `aixecar()` | goes to the intermediate carry pose |
| `inici()` | returns to the zero pose |
| `esperar(segons)` | waits without moving |

All of them also accept `canell=` and `gir=`, and a `segons=` to override the default duration. Joints that are not named stay where they are. `for` loops work, which is what the second challenge needs.

**Executar el programa** compiles the code, turns it into the same kind of step list as the `TASCA` block and plays it with the same Pas a pas, Pause and Stop buttons. The **Pas a pas** button on this tab compiles the program and runs it one step at a time, which is the quickest way for a team to see which line moves which joint. Nothing reaches the servos until the whole program has been read and validated, so a mistake stops before the arm moves. Errors are reported in Catalan with the line number for a syntax error, and the name for an unknown function. A program is capped at 200 steps, so a runaway `while True` is caught rather than run. The code runs with a restricted set of builtins: imports, `open` and `eval` are not reachable.

**Desar** and **Carregar** write and read the program as a `.py` file, so an instructor can prepare one file per team, and **Exemple** restores the starting template. While a program runs, the editor and its buttons are disabled.

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

- "fora dels límits": a pose in the `TASCA` block is outside that joint's min/max. Change the pose, or widen the limit in the joint panel.
- "permís denegat": add your user to `dialout` and log out and back in (see "Run the app")
- "Cap motor ha respost": check the 7.4V supply, the cable chain and the servo IDs with `src/servo_scan.py`
- "NO aplicat" on a servo: the servo ignored the zero command; check its firmware or use the Feetech tool

## License

This project is intended for workshop and educational use.
