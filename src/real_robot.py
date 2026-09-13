"""Adapter for a real SO-101 arm, built on the Feetech SDK (scservo_sdk).

Motion commands are offsets from the zero pose. The zero pose is the position
the servos report as 2048 (the middle of their range), which is what
`servo_positions.py --set-zero` or the instructor panel stores in the servos.
"""

import time

from src.servo_positions import POSITION_MIDDLE, read_positions, set_zero_positions
from src.so101_bus import COMM_SUCCESS, DEFAULT_BAUDRATE, DEFAULT_PORT, describe, discover_ports, open_bus

POSITION_MAX = 4095


class RealRobotAdapter:
    """Same interface as MockRobot, talking to the servos through the SDK."""

    # One workshop coordinate unit is 10 encoder steps (about 0.9 degrees), so the
    # controller's +-100 range maps to +-1000 steps around the zero pose.
    STEPS_PER_UNIT = 10
    # Offset from the gripper zero (recorded open) to the closed position.
    GRIPPER_CLOSE_STEPS = 500
    # STS3215 units: speed in encoder steps per second (50 steps/s = 0.732 rpm),
    # acceleration in units of 100 steps/s^2. About 11 rpm.
    SAFE_SPEED = 750
    SAFE_ACC = 50

    def __init__(self, port=None, baudrate=DEFAULT_BAUDRATE):
        self.position = {"x": 0, "y": 0, "z": 0}
        self.gripper_open = True
        self.emergency_stop_active = False
        self.connected = False
        self.port = port
        self.baudrate = baudrate
        self.port_handler = None
        self.packet_handler = None
        self.servo_ids = [1, 2, 3, 4, 5, 6]
        self.responding_ids = []

    # -- connection ----------------------------------------------------------

    def connect(self, port=None, baudrate=None):
        """Open the first port where at least one arm servo answers a ping."""
        baudrate = baudrate or self.baudrate
        candidates = [port or self.port] if (port or self.port) else (discover_ports() or [DEFAULT_PORT])
        errors = []
        for candidate in candidates:
            try:
                port_handler, packet_handler = open_bus(candidate, baudrate)
            except RuntimeError as exc:
                errors.append(str(exc))
                continue
            responding = [
                servo_id for servo_id in self.servo_ids
                if packet_handler.ping(servo_id)[1] == COMM_SUCCESS
            ]
            if responding:
                self.port_handler = port_handler
                self.packet_handler = packet_handler
                self.port = candidate
                self.baudrate = baudrate
                self.responding_ids = responding
                self.connected = True
                return True
            port_handler.closePort()
            errors.append(f"{candidate}: no STS3215 servo answered")
        raise RuntimeError("Could not connect to the SO-101 bus. " + "; ".join(errors))

    def disconnect(self):
        if self.port_handler is not None:
            self.port_handler.closePort()
        self.port_handler = None
        self.packet_handler = None
        self.connected = False
        self.responding_ids = []

    # -- helpers -------------------------------------------------------------

    def _require_connected(self):
        if not self.connected or self.packet_handler is None:
            raise RuntimeError("Robot is not connected. Use demo mode or reconnect the bus.")

    def _require_ready(self):
        if self.emergency_stop_active:
            raise RuntimeError("Robot is in emergency stop mode.")
        self._require_connected()

    @staticmethod
    def _target(delta):
        """Absolute servo position for an offset from the zero pose."""
        return int(max(0, min(POSITION_MAX, POSITION_MIDDLE + round(delta))))

    def _write_position(self, servo_id, position):
        result, error = self.packet_handler.WritePosEx(servo_id, int(position), self.SAFE_SPEED, self.SAFE_ACC)
        if result != COMM_SUCCESS:
            raise RuntimeError(f"Servo {servo_id}: {describe(self.packet_handler, result, error)}")

    # -- motion --------------------------------------------------------------

    def move_to(self, x, y, z):
        self._require_ready()
        self.position = {"x": x, "y": y, "z": z}
        # A safe first-stage integration: map the x/y/z coordinates to the first 3
        # joints as offsets from the zero pose. Adjust for the real arm geometry.
        for servo_id, value in ((1, x), (2, y), (3, z)):
            self._write_position(servo_id, self._target(value * self.STEPS_PER_UNIT))
            time.sleep(0.02)
        return {"x": x, "y": y, "z": z}

    def open_gripper(self):
        self._require_ready()
        self.gripper_open = True
        self._write_position(6, self._target(0))
        return True

    def close_gripper(self):
        self._require_ready()
        self.gripper_open = False
        self._write_position(6, self._target(self.GRIPPER_CLOSE_STEPS))
        return False

    def home(self):
        self._require_ready()
        self.position = {"x": 0, "y": 0, "z": 0}
        self.gripper_open = True
        for servo_id in self.servo_ids:
            self._write_position(servo_id, POSITION_MIDDLE)
            time.sleep(0.02)
        return {"x": 0, "y": 0, "z": 0}

    def emergency_stop(self):
        """Stop every joint where it is by commanding its current position."""
        self.emergency_stop_active = True
        if self.connected and self.packet_handler is not None:
            try:
                for servo_id, reading in read_positions(self.packet_handler, self.servo_ids).items():
                    self.packet_handler.WritePosEx(servo_id, reading["position"], self.SAFE_SPEED, self.SAFE_ACC)
            except Exception:
                pass
        return True

    def reset_emergency_stop(self):
        self.emergency_stop_active = False
        return True

    # -- calibration and diagnostics ----------------------------------------

    def read_positions(self, ids=None):
        self._require_connected()
        return read_positions(self.packet_handler, ids or self.servo_ids)

    def set_zero(self, ids=None, log=None):
        """Store the current pose as the zero pose in the servos (EEPROM)."""
        self._require_connected()
        return set_zero_positions(self.packet_handler, ids or self.servo_ids, log=log or (lambda *_: None))
