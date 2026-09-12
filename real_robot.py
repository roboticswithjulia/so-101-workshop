import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover
    serial = None
    list_ports = None


class RealRobotAdapter:
    """Adapter for a real SO-101 robot over a Feetech serial bus.

    This implementation keeps the workshop experience safe, while providing a real
    serial connectivity layer for the Raspberry Pi. The exact servo protocol is
    Feetech-compatible and the adapter is intentionally conservative.
    """

    DEFAULT_BAUDRATE = 1000000
    PORT_CANDIDATES = [
        "/dev/ttyUSB0",
        "/dev/ttyUSB1",
        "/dev/ttyUSB2",
        "/dev/ttyAMA0",
        "/dev/serial0",
        "/dev/ttyS0",
        "/dev/ttyS1",
    ]

    def __init__(self):
        self.position = {"x": 0, "y": 0, "z": 0}
        self.gripper_open = True
        self.emergency_stop_active = False
        self.connected = False
        self.port = None
        self.connection = None
        self.servo_ids = [1, 2, 3, 4, 5, 6]

    # Feetech STS/SCS protocol constants.
    HEADER = b"\xff\xff"
    INSTRUCTION_PING = 0x01
    INSTRUCTION_WRITE = 0x03
    REGISTER_GOAL_POSITION = 0x2A

    def _checksum(self, body):
        # Sum of id, length, instruction and parameters (everything after the header).
        return (~sum(body)) & 0xFF

    def _build_packet(self, servo_id, instruction, params):
        body = [servo_id, len(params) + 2, instruction, *params]
        return self.HEADER + bytes(body) + bytes([self._checksum(body)])

    def _goal_position_packet(self, servo_id, position):
        position = int(max(0, min(4095, position)))
        params = [self.REGISTER_GOAL_POSITION, position & 0xFF, (position >> 8) & 0xFF]
        return self._build_packet(servo_id, self.INSTRUCTION_WRITE, params)

    def _ping(self, servo_id):
        if self.connection is None:
            return False
        packet = self._build_packet(servo_id, self.INSTRUCTION_PING, [])
        try:
            self.connection.reset_input_buffer()
            self.connection.write(packet)
            time.sleep(0.02)
            response = self.connection.read(self.connection.in_waiting or 6)
            # Half-duplex adapters may echo our own packet before the reply.
            if response.startswith(packet):
                response = response[len(packet):]
            return len(response) >= 6 and response[:2] == self.HEADER and response[2] == servo_id
        except Exception:
            return False

    def _discover_port(self):
        if serial is None:
            raise RuntimeError("pyserial is not installed. Run: pip install pyserial")

        if list_ports is not None:
            ports = [p.device for p in list_ports.comports()]
            if ports:
                return sorted(set(ports))[0]

        for port in self.PORT_CANDIDATES:
            try:
                with serial.Serial(port, timeout=0.1):
                    return port
            except Exception:
                continue
        raise RuntimeError("No serial port available for the SO-101 bus.")

    def connect(self, port=None, baudrate=None):
        if serial is None:
            raise RuntimeError("pyserial is not installed. Run: pip install pyserial")

        port = port or self._discover_port()
        baudrate = baudrate or self.DEFAULT_BAUDRATE
        self.connection = serial.Serial(port, baudrate=baudrate, timeout=0.05)
        self.port = port

        for servo_id in self.servo_ids:
            if self._ping(servo_id):
                self.connected = True
                return True

        self.connection.close()
        self.connection = None
        raise RuntimeError("No STS3215 servo responded on the serial bus.")

    def move_to(self, x, y, z):
        if self.emergency_stop_active:
            raise RuntimeError("Robot is in emergency stop mode.")
        if not self.connected or self.connection is None:
            raise RuntimeError("Robot is not connected. Use demo mode or reconnect the bus.")

        self.position = {"x": x, "y": y, "z": z}

        # A safe first-stage integration: map the x/y/z coordinates to the first 3
        # servos in a linear and bounded way. This is intentionally conservative for
        # workshop use and should be adjusted for the exact arm geometry.
        target_map = {
            1: int(max(0, min(1000, (x + 50) * 10))),
            2: int(max(0, min(1000, (y + 50) * 10))),
            3: int(max(0, min(1000, (z + 50) * 10))),
        }

        for servo_id, angle in target_map.items():
            self.connection.write(self._goal_position_packet(servo_id, angle))
            time.sleep(0.02)

        return {"x": x, "y": y, "z": z}

    def open_gripper(self):
        if self.emergency_stop_active:
            raise RuntimeError("Robot is in emergency stop mode.")
        if not self.connected or self.connection is None:
            raise RuntimeError("Robot is not connected. Use demo mode or reconnect the bus.")

        self.gripper_open = True
        self.connection.write(self._goal_position_packet(6, 0))
        return True

    def close_gripper(self):
        if self.emergency_stop_active:
            raise RuntimeError("Robot is in emergency stop mode.")
        if not self.connected or self.connection is None:
            raise RuntimeError("Robot is not connected. Use demo mode or reconnect the bus.")

        self.gripper_open = False
        self.connection.write(self._goal_position_packet(6, 1023))
        return False

    def home(self):
        self.position = {"x": 0, "y": 0, "z": 0}
        self.gripper_open = True
        if self.connected and self.connection is not None:
            for servo_id in self.servo_ids:
                self.connection.write(self._goal_position_packet(servo_id, 0))
                time.sleep(0.02)
        return {"x": 0, "y": 0, "z": 0}

    def emergency_stop(self):
        self.emergency_stop_active = True
        if self.connected and self.connection is not None:
            try:
                self.connection.write(b"\x00")
            except Exception:
                pass
        return True

    def reset_emergency_stop(self):
        self.emergency_stop_active = False
        return True
