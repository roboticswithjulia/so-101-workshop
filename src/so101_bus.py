"""Shared helpers for the SO-101 command line tools built on the Feetech SDK.

The SDK is the `ftservo-python-sdk` package from requirements.txt. It installs
the `scservo_sdk` module used here (PortHandler + sms_sts for STS3215 servos).
"""

try:
    from scservo_sdk import COMM_SUCCESS, PortHandler, sms_sts
except ImportError:  # pragma: no cover - only hardware access needs the SDK
    PortHandler = sms_sts = None
    COMM_SUCCESS = 0

try:
    from serial.tools import list_ports
except ImportError:  # pragma: no cover
    list_ports = None

INSTALL_HINT = "The Feetech SDK (ftservo-python-sdk) is not installed. Run: pip install -r requirements.txt"

DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_BAUDRATE = 1000000
ARM_IDS = "1-6"
MAX_ID = 252
STEPS_PER_TURN = 4096

# STS3215 register addresses (see docs/FTSERVO_SDK.md for the full table).
REG_MODEL = 3
REG_ID = 5
REG_BAUD_RATE = 6
REG_MIN_ANGLE_LIMIT = 9
REG_MAX_ANGLE_LIMIT = 11
REG_OFFSET = 31
REG_MODE = 33
REG_TORQUE_ENABLE = 40
REG_PRESENT_POSITION = 56
REG_PRESENT_VOLTAGE = 62
REG_PRESENT_TEMPERATURE = 63
REG_MOVING = 66

# Value of the baud rate register -> bits per second.
BAUD_RATES = [1000000, 500000, 250000, 128000, 115200, 76800, 57600, 38400]
MODES = {0: "position", 1: "wheel", 2: "pwm", 3: "step"}

JOINT_NAMES = {
    1: "Base",
    2: "Shoulder",
    3: "Elbow",
    4: "Wrist",
    5: "Wrist roll",
    6: "Gripper",
}


def joint_name(servo_id):
    return JOINT_NAMES.get(servo_id, f"Servo {servo_id}")


def parse_ids(text):
    """'1-6' or '1,2,3' -> sorted list of servo IDs."""
    ids = set()
    for part in text.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            low, high = part.split("-", 1)
            ids.update(range(int(low), int(high) + 1))
        else:
            ids.add(int(part))
    return sorted(i for i in ids if 0 <= i <= MAX_ID)


def discover_ports():
    """Serial ports that belong to a USB device (the bus servo adapter).

    Falls back to every port pyserial knows about when no USB port is found.
    """
    if list_ports is None:
        return [DEFAULT_PORT, "/dev/ttyUSB0"]
    ports = list(list_ports.comports())
    usb = sorted(p.device for p in ports if p.vid is not None)
    if usb:
        return usb
    return sorted(p.device for p in ports)


def open_bus(port, baudrate=DEFAULT_BAUDRATE):
    """Open the servo bus. Returns (port_handler, packet_handler)."""
    if PortHandler is None:
        raise RuntimeError(INSTALL_HINT)
    port_handler = PortHandler(port)
    try:
        opened = port_handler.openPort() and port_handler.setBaudRate(baudrate)
    except Exception as exc:  # the SDK lets pyserial errors through
        hint = ""
        if "ermission" in str(exc):
            hint = " Add your user to the dialout group: sudo usermod -aG dialout $USER"
        raise RuntimeError(f"Could not open {port}: {exc}{hint}") from None
    if not opened:
        raise RuntimeError(f"Could not open {port} at {baudrate} bps.")
    return port_handler, sms_sts(port_handler)


def describe(packet_handler, result, error):
    """Human-readable text for an SDK (result, error) pair."""
    text = []
    if result != COMM_SUCCESS:
        text.append(packet_handler.getTxRxResult(result))
    if error:
        text.append(packet_handler.getRxPacketError(error))
    return " ".join(text)


def decode_offset(raw):
    """Offset register uses sign-magnitude with the sign in bit 11."""
    value = raw & 0x7FF
    return -value if raw & 0x800 else value
