# FTServo_Python SDK overview

Reference notes on the official Feetech Python SDK checked out in [FTServo_Python/](FTServo_Python/).
Source: https://github.com/ftservo/FTServo_Python (MIT license, also on PyPI as `ftservo-python-sdk`).
Requires Python 3 and `pyserial`.

The SDK talks to Feetech smart serial servos (SCS, SMS/STS and HLS families) over a
half-duplex TTL bus through a USB adapter such as the URT-1 or the Waveshare bus servo
adapter. The SO-101 uses STS3215 servos, so the relevant class is `sms_sts`.

## Folder layout

| Path | Content |
|------|---------|
| `scservo_sdk/` | The library itself (one package, no dependencies besides pyserial) |
| `sms_sts/` | Examples for STS/SMS servos (the STS3215 family) |
| `scscl/` | Examples for the older SCS servos (SCS15, SCS09, ...) |
| `hls/` | Examples for the HLS high-torque family, plus offset calibration and factory reset |

Every example is a standalone script that opens `/dev/ttyUSB0` at 1,000,000 bps and
loops forever. Edit the port at the top of the script before running it.

## Architecture

```
PortHandler                 pyserial wrapper: open/close, baud, byte timing, timeouts
   └── protocol_packet_handler   packet build/parse, ping, read, write, reg write, sync, reset
          ├── sms_sts            STS/SMS register map + position/speed helpers   <- STS3215
          ├── scscl              SCS register map + helpers
          └── hls                HLS register map + helpers (adds torque limit)
GroupSyncWrite              collect per-servo data, send one SYNC WRITE packet
GroupSyncRead               send one SYNC READ, parse one reply per servo
```

Usage pattern common to every script:

```python
from scservo_sdk import *

portHandler = PortHandler("/dev/ttyACM0")
packetHandler = sms_sts(portHandler)
portHandler.openPort()
portHandler.setBaudRate(1000000)

model, result, error = packetHandler.ping(1)
if result != COMM_SUCCESS:
    print(packetHandler.getTxRxResult(result))
if error != 0:
    print(packetHandler.getRxPacketError(error))

portHandler.closePort()
```

Almost every call returns `(..., comm_result, error)`. `comm_result` is about the
serial exchange (`COMM_SUCCESS`, `COMM_RX_TIMEOUT`, ...). `error` is the status byte the
servo itself reports (voltage, angle sensor, overheat, over-current, overload bits).

## Protocol definitions (`scservo_def.py`)

Packet format: `0xFF 0xFF <id> <length> <instruction> <params...> <checksum>` with
`checksum = ~(id + length + instruction + params) & 0xFF`.

| Constant | Value | Meaning |
|----------|-------|---------|
| `BROADCAST_ID` | 0xFE (254) | All servos, no reply expected |
| `MAX_ID` | 0xFC (252) | Highest usable servo ID |
| `INST_PING` | 0x01 | Is the servo there? |
| `INST_READ` | 0x02 | Read N bytes from an address |
| `INST_WRITE` | 0x03 | Write bytes at an address, applied immediately |
| `INST_REG_WRITE` | 0x04 | Write bytes, but hold them until ACTION |
| `INST_ACTION` | 0x05 | Apply all pending REG_WRITEs at once |
| `INST_SYNC_WRITE` | 0x83 | One packet writing the same registers on many servos |
| `INST_SYNC_READ` | 0x82 | One packet reading the same registers from many servos |
| `INST_RESET` | 0x0A | Factory reset |
| `INST_OFSCAL` | 0x0B | Offset calibration: make the current position read as a given value |

Communication results: `COMM_SUCCESS` (0), `COMM_PORT_BUSY`, `COMM_TX_FAIL`,
`COMM_RX_FAIL`, `COMM_TX_ERROR`, `COMM_RX_WAITING`, `COMM_RX_TIMEOUT`,
`COMM_RX_CORRUPT`, `COMM_NOT_AVAILABLE`.

Servo error bits: `ERRBIT_VOLTAGE` (1), `ERRBIT_ANGLE` (2), `ERRBIT_OVERHEAT` (4),
`ERRBIT_OVERELE` (8, over-current), `ERRBIT_OVERLOAD` (32).

## `PortHandler` (`port_handler.py`)

| Method | Purpose |
|--------|---------|
| `PortHandler(port_name)` | Create the handler; nothing is opened yet |
| `openPort()` | Open at the current baud rate (default 1,000,000) |
| `closePort()` | Close the serial port |
| `setBaudRate(baud)` | Reopen at one of 4800 ... 1000000; returns False for unsupported values |
| `getBaudRate()` / `setPortName()` / `getPortName()` | Accessors |
| `clearPort()` | Flush the transmit buffer |
| `readPort(n)` / `writePort(bytes)` | Raw I/O |
| `getBytesAvailable()` | Bytes waiting in the input buffer |
| `setPacketTimeout(packet_length)` | Arm a reply timeout: byte time × length + 50 ms latency |
| `setPacketTimeoutMillis(ms)` / `isPacketTimeout()` | Manual timeout control |

The port is opened with `timeout=0` (non-blocking); all waiting is done by the
packet handler polling `isPacketTimeout()`.

## `protocol_packet_handler` (`protocol_packet_handler.py`)

Generic layer shared by all servo families. `protocol_end` selects byte order:
0 for STS/SMS/HLS (little-endian), 1 for SCS (big-endian).

| Method | Purpose |
|--------|---------|
| `ping(id)` | Ping, then read the model number at address 3. Returns `(model, result, error)` |
| `read1ByteTxRx / read2ByteTxRx / read4ByteTxRx(id, address)` | Read a register, returns `(value, result, error)` |
| `readTxRx(id, address, length)` | Read raw bytes |
| `write1ByteTxRx / write2ByteTxRx / write4ByteTxRx(id, address, value)` | Write a register and wait for the status reply |
| `write*TxOnly(...)` | Same, without waiting for a reply |
| `writeTxRx(id, address, length, data)` | Write a block of bytes |
| `regWriteTxRx / regWriteTxOnly(...)` | Deferred write (REG_WRITE), applied on `action()` |
| `action(id)` | Trigger pending REG_WRITEs (use `BROADCAST_ID` for all) |
| `syncWriteTxOnly(start, length, params, n)` | Low-level SYNC WRITE, used by `GroupSyncWrite` |
| `syncReadTx / syncReadRx(...)` | Low-level SYNC READ, used by `GroupSyncRead` |
| `reOfsCal(id, position)` | OFSCAL: the servo's current physical position becomes `position` (EEPROM) |
| `reSet(id)` | Factory reset of the servo |
| `scs_tohost(value, bit)` / `scs_toscs(value, bit)` | Convert between sign-magnitude (bit 15 = sign) and Python ints |
| `scs_makeword / scs_lobyte / scs_hibyte / ...` | Byte packing helpers honouring the family byte order |
| `getTxRxResult(result)` | Human-readable text for a comm result |
| `getRxPacketError(error)` | Human-readable text for a servo error byte |

Notes on the reply parser (`rxPacket`): it scans for the `0xFF 0xFF` header, checks ID,
length and checksum, and retries until the reply ID matches the request. It does not
strip a transmit echo, so the adapters Feetech tests with do not echo.

## `sms_sts` (STS/SMS family, includes STS3215)

Register map (`SMS_STS_*` constants, addresses in decimal):

| Address | Name | Area | Notes |
|---------|------|------|-------|
| 3-4 | MODEL | EEPROM, read-only | Model number (STS3215 reports 1540 in the README example) |
| 5 | ID | EEPROM | Servo ID, 0-252 |
| 6 | BAUD_RATE | EEPROM | Index 0..7: 1M, 500k, 250k, 128k, 115200, 76800, 57600, 38400 |
| 9-10 / 11-12 | MIN / MAX_ANGLE_LIMIT | EEPROM | Position limits |
| 26 / 27 | CW_DEAD / CCW_DEAD | EEPROM | Dead band |
| 31-32 | OFS | EEPROM | Position offset (what OFSCAL writes) |
| 33 | MODE | EEPROM | 0 position (servo) mode, 1 wheel (continuous) mode |
| 40 | TORQUE_ENABLE | SRAM | 0 off, 1 on |
| 41 | ACC | SRAM | Acceleration, units of 8.7 °/s² |
| 42-43 | GOAL_POSITION | SRAM | 0-4095 = one turn (4096 steps) |
| 44-45 | GOAL_TIME | SRAM | Unused for STS position moves (written as 0) |
| 46-47 | GOAL_SPEED | SRAM | Speed, units of 0.732 rpm (≈ 50 steps/s) |
| 55 | LOCK | SRAM | 1 = EEPROM locked (default), 0 = writable |
| 56-57 | PRESENT_POSITION | read-only | Current position |
| 58-59 | PRESENT_SPEED | read-only | Current speed, sign-magnitude |
| 60-61 | PRESENT_LOAD | read-only | Load |
| 62 | PRESENT_VOLTAGE | read-only | Volts × 10 |
| 63 | PRESENT_TEMPERATURE | read-only | °C |
| 66 | MOVING | read-only | 1 while moving to the goal |
| 69-70 | PRESENT_CURRENT | read-only | Current |

Helper methods:

| Method | What it does |
|--------|--------------|
| `WritePosEx(id, position, speed, acc)` | One WRITE at address 41: acc, goal position, time=0, speed. The normal "move to" call |
| `RegWritePosEx(id, position, speed, acc)` | Same, deferred until `RegAction()` so several servos start together |
| `SyncWritePosEx(id, position, speed, acc)` | Queue the same data into `groupSyncWrite`; send with `groupSyncWrite.txPacket()` |
| `RegAction()` | Broadcast ACTION |
| `ReadPos(id)` | Present position |
| `ReadSpeed(id)` | Present speed (signed) |
| `ReadPosSpeed(id)` | Both in a single 4-byte read |
| `ReadMoving(id)` | 1 while the servo has not reached its goal |
| `WheelMode(id)` | Set MODE = 1 (continuous rotation) |
| `WriteSpec(id, speed, acc)` | In wheel mode: run at signed speed with the given acceleration |
| `unLockEprom(id)` / `LockEprom(id)` | Write LOCK 0 / 1 around any EEPROM change (ID, baud, limits, offset, mode) |

Rule of thumb from the examples for the time a move takes:
`(|P1 - P0| / (V * 50)) + ((V * 50) / (A * 100)) + 0.05` seconds, with V and A the
values passed to `WritePosEx`.

## `scscl` (SCS family)

Same structure with the SCS register map (LOCK at 48, no ACC register, big-endian words).
`WritePos(id, position, time, speed)` writes goal position, time and speed at address 42.
Adds `PWMMode(id)` and `WritePWM(id, value)` for open-loop PWM control. Not used by the SO-101.

## `hls` (HLS family)

Same as `sms_sts` but every position call takes an extra `torque` limit
(`WritePosEx(id, position, speed, acc, torque)`, `WriteSpec(id, speed, acc, torque)`),
written to registers 44-45 instead of the time field. Not used by the SO-101, but its
examples show the OFSCAL and RESET instructions, which the base class also exposes for STS.

## `GroupSyncWrite` / `GroupSyncRead`

`GroupSyncWrite(handler, start_address, data_length)`: `addParam(id, bytes)`,
`changeParam`, `removeParam`, `clearParam`, `txPacket()`. One packet moves every servo in
the same instant, without individual replies. `sms_sts` creates one at address 41 with
length 7, matching `WritePosEx`.

`GroupSyncRead(handler, start_address, data_length)`: `addParam(id)`, `txRxPacket()`,
then `isAvailable(id, address, length)` and `getData(id, address, length)`. One request,
one reply per servo, useful to read all six joint positions of an arm in one round trip.

## Example applications

| Script | Demonstrates |
|--------|--------------|
| `sms_sts/ping.py` | Detect a servo and print its model number |
| `sms_sts/read.py` | Poll position and speed of ID 1 once per second |
| `sms_sts/write.py` | Move ID 1 between 0 and 4095 with speed 60 and acceleration 50 |
| `sms_sts/read_write.py` | Same move, but poll `ReadMoving` until the servo arrives instead of sleeping |
| `sms_sts/reg_write.py` | Queue a move on IDs 1-10 with REG_WRITE, then fire them together with ACTION |
| `sms_sts/sync_write.py` | Move IDs 1-10 with a single SYNC WRITE packet |
| `sms_sts/sync_read.py` | Read position and speed of IDs 1-10 with a single SYNC READ |
| `sms_sts/wheel.py` | Switch ID 1 to wheel mode and spin forward, stop, reverse, stop |
| `hls/ofscal.py` | Offset calibration: make the current position read as 1024 (EEPROM) |
| `hls/reset.py` | Factory reset of a servo |
| `scscl/*` | The same set for SCS servos |

## PyPI package vs GitHub repository

The project installs the SDK from PyPI (`ftservo-python-sdk==2.0.0` in `requirements.txt`).
That build is not identical to the GitHub repository described above:

- it contains `port_handler`, `protocol_packet_handler`, `scservo_def`, `sms_sts`, `scscl`
  and the two group sync classes, but **no `hls.py`**;
- the packet handler has **no `reOfsCal` and no `reSet`** methods (the OFSCAL and RESET
  instructions), so in-servo offset calibration must be done with a register write instead
  (writing 128 to the torque register 40 makes the STS3215 take its current position as 2048).

Everything the workshop tools use (`PortHandler`, `sms_sts.ping`, `ReadPos`, `ReadPosSpeed`,
`ReadMoving`, `read1ByteTxRx`, `read2ByteTxRx`, `write1ByteTxRx`, `WritePosEx`) exists in both.

## Relationship to this project

The workshop tools import the SDK through [so101_bus.py](so101_bus.py), which provides
`open_bus()` (PortHandler + sms_sts), ID parsing, USB port discovery and the STS3215
register addresses:

| Tool | SDK calls used |
|------|----------------|
| `servo_scan.py` | `ping` on each ID and baud rate |
| `servo_config.py` | `ping`, `read1ByteTxRx`, `read2ByteTxRx` on the configuration registers |
| `servo_positions.py` | `ReadPosSpeed`, `ReadMoving`, `getRxPacketError` |

Things the SDK has that could be useful next:

- **`WritePosEx(id, position, speed, acc)`** for motion with an explicit slow speed and
  acceleration, which keeps workshop motions gentle and predictable.
- **`ReadMoving`** to wait for a move to finish instead of a fixed sleep.
- **`GroupSyncWrite` / `GroupSyncRead`** to move or read all six joints in one packet.
- **`unLockEprom` + write ID / baud + `LockEprom`** to set servo IDs from Python instead of
  the Feetech desktop tool.
- **Load and current registers** for a diagnostics panel in instructor mode.

Caveats when using the SDK directly:

- Examples hard-code `/dev/ttyUSB0`; the Waveshare adapter appears as `/dev/ttyACM0`.
- Comments in the register tables are in Chinese; the names are self-explanatory.
- `setBaudRate` accepts only the listed standard rates and returns False otherwise.
- `openPort` lets pyserial exceptions through (missing port, permission denied) instead of
  returning False; `so101_bus.open_bus()` converts them into a readable message.
- No handling of a transmit echo. If an adapter echoes, the echo of a ping is accepted as
  the reply, so a scan would report every ID as present.
- Each failed ping waits about 50 ms, so a full scan of IDs 1..253 takes around 13 s per
  port and baud rate. Use `--ids 1-6` for a quick check.
