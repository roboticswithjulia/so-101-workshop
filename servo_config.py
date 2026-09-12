#!/usr/bin/env python3
"""Show the configuration of each SO-101 servo and check it against the workshop setup.

Uses the official Feetech SDK (`scservo_sdk`, installed from requirements.txt).
Everything is read-only: nothing is written to the servos.

Usage:

    python3 servo_config.py                       # /dev/ttyACM0, IDs 1..6
    python3 servo_config.py --port /dev/ttyUSB0 --ids 1,2,3
"""

import argparse
import sys

from so101_bus import (
    ARM_IDS,
    BAUD_RATES,
    COMM_SUCCESS,
    DEFAULT_BAUDRATE,
    DEFAULT_PORT,
    MODES,
    REG_BAUD_RATE,
    REG_MAX_ANGLE_LIMIT,
    REG_MIN_ANGLE_LIMIT,
    REG_MODE,
    REG_OFFSET,
    REG_PRESENT_POSITION,
    REG_PRESENT_TEMPERATURE,
    REG_PRESENT_VOLTAGE,
    REG_TORQUE_ENABLE,
    decode_offset,
    joint_name,
    open_bus,
    parse_ids,
)

CHECKLIST = """
Feetech STS3215 configuration checklist
- Use a serial bus, not PWM
- Use a stable 7.4V power source for the motors
- Share GND between the controller and all servos
- Set each servo to a unique ID (1..6 for the SO-101)
- Baud rate 1,000,000 bps
- Position mode, torque enabled
- Define a safe home pose before running workshop tasks
"""


def read_config(packet_handler, servo_id):
    """Read the relevant registers of one servo. Returns a dict, or None if it does not answer."""
    model, result, _ = packet_handler.ping(servo_id)
    if result != COMM_SUCCESS:
        return None

    def read1(address):
        value, result, _ = packet_handler.read1ByteTxRx(servo_id, address)
        return int(value) if result == COMM_SUCCESS else None

    def read2(address):
        value, result, _ = packet_handler.read2ByteTxRx(servo_id, address)
        return int(value) if result == COMM_SUCCESS else None

    baud_index = read1(REG_BAUD_RATE)
    offset_raw = read2(REG_OFFSET)
    mode = read1(REG_MODE)
    voltage = read1(REG_PRESENT_VOLTAGE)
    return {
        "model": int(model),
        "baudrate": BAUD_RATES[baud_index] if baud_index is not None and baud_index < len(BAUD_RATES) else None,
        "min_limit": read2(REG_MIN_ANGLE_LIMIT),
        "max_limit": read2(REG_MAX_ANGLE_LIMIT),
        "offset": decode_offset(offset_raw) if offset_raw is not None else None,
        "mode": MODES.get(mode, str(mode)) if mode is not None else None,
        "torque": read1(REG_TORQUE_ENABLE),
        "position": read2(REG_PRESENT_POSITION),
        "voltage": voltage / 10.0 if voltage is not None else None,
        "temperature": read1(REG_PRESENT_TEMPERATURE),
    }


def check_config(config):
    """Return a list of warnings for a servo configuration."""
    warnings = []
    if config["baudrate"] not in (None, DEFAULT_BAUDRATE):
        warnings.append(f"baud rate is {config['baudrate']}, expected {DEFAULT_BAUDRATE}")
    if config["mode"] not in (None, "position"):
        warnings.append(f"mode is {config['mode']}, expected position")
    if config["torque"] == 0:
        warnings.append("torque is off")
    if config["voltage"] is not None and not 6.0 <= config["voltage"] <= 8.4:
        warnings.append(f"voltage {config['voltage']:.1f} V outside 6.0-8.4 V")
    if config["temperature"] is not None and config["temperature"] >= 60:
        warnings.append(f"temperature {config['temperature']} C is high")
    if config["min_limit"] is not None and config["max_limit"] is not None and config["min_limit"] >= config["max_limit"]:
        warnings.append("angle limits are inverted or disabled")
    return warnings


def fmt(value, template="{}"):
    return "--" if value is None else template.format(value)


def print_table(ids, configs):
    print(f"{'ID':>3}  {'Joint':<11} {'Model':>5} {'Baud':>8} {'Mode':<8} {'Limits':<11} {'Offset':>6} {'Torque':>6} {'Pos':>5} {'Volt':>5} {'Temp':>4}")
    for servo_id in ids:
        name = joint_name(servo_id)
        config = configs.get(servo_id)
        if config is None:
            print(f"{servo_id:>3}  {name:<11} no answer")
            continue
        limits = f"{fmt(config['min_limit'])}..{fmt(config['max_limit'])}"
        torque = {1: "on", 0: "off"}.get(config["torque"], fmt(config["torque"]))
        print(
            f"{servo_id:>3}  {name:<11} {fmt(config['model']):>5} {fmt(config['baudrate']):>8} "
            f"{fmt(config['mode']):<8} {limits:<11} {fmt(config['offset']):>6} {torque:>6} "
            f"{fmt(config['position']):>5} {fmt(config['voltage'], '{:.1f}'):>5} {fmt(config['temperature']):>4}"
        )


def main(argv=None):
    parser = argparse.ArgumentParser(description="Show the configuration of each SO-101 servo.")
    parser.add_argument("--port", default=DEFAULT_PORT, help="serial device (default: %(default)s)")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUDRATE, help="baud rate (default: %(default)s)")
    parser.add_argument("--ids", default=ARM_IDS, help="servo IDs, e.g. '1-6' or '1,2,3' (default: %(default)s)")
    args = parser.parse_args(argv)

    print(CHECKLIST)
    ids = parse_ids(args.ids)
    try:
        port_handler, packet_handler = open_bus(args.port, args.baud)
    except RuntimeError as exc:
        print(exc)
        return 1
    print(f"Connected to {args.port} @ {args.baud} bps\n")

    try:
        configs = {servo_id: read_config(packet_handler, servo_id) for servo_id in ids}
    finally:
        port_handler.closePort()

    print_table(ids, configs)
    answered = {servo_id: config for servo_id, config in configs.items() if config is not None}
    if not answered:
        print("\nNo servo answered. Run servo_scan.py to find the port and the IDs.")
        return 1

    problems = 0
    for servo_id, config in answered.items():
        for warning in check_config(config):
            problems += 1
            print(f"  ! ID {servo_id} {joint_name(servo_id)}: {warning}")
    missing = [servo_id for servo_id in ids if servo_id not in answered]
    if missing:
        problems += 1
        print(f"  ! no answer from IDs {missing}")
    print("\nAll checks passed." if not problems else "\nReview the warnings above before the workshop.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
