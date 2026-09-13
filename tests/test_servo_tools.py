import unittest

import src.so101_bus as so101_bus
from src.servo_config import check_config, read_config
from src.servo_positions import read_positions, set_zero_positions
from src.servo_scan import scan_ids
from src.so101_bus import COMM_SUCCESS, decode_offset, parse_ids

COMM_RX_TIMEOUT = -6


class FakePacketHandler:
    """Stand-in for scservo_sdk.sms_sts backed by a per-servo register dict."""

    def __init__(self, servos):
        self.servos = servos  # id -> {address: value}, 2-byte values stored at the low address

    def ping(self, servo_id):
        if servo_id not in self.servos:
            return 0, COMM_RX_TIMEOUT, 0
        return self.servos[servo_id].get(so101_bus.REG_MODEL, 1540), COMM_SUCCESS, 0

    def read1ByteTxRx(self, servo_id, address):
        if servo_id not in self.servos or address not in self.servos[servo_id]:
            return 0, COMM_RX_TIMEOUT, 0
        return self.servos[servo_id][address], COMM_SUCCESS, 0

    read2ByteTxRx = read1ByteTxRx

    def ReadPosSpeed(self, servo_id):
        if servo_id not in self.servos:
            return 0, 0, COMM_RX_TIMEOUT, 0
        regs = self.servos[servo_id]
        return regs.get(so101_bus.REG_PRESENT_POSITION, 0), regs.get("speed", 0), COMM_SUCCESS, regs.get("error", 0)

    def ReadMoving(self, servo_id):
        return self.servos[servo_id].get(so101_bus.REG_MOVING, 0), COMM_SUCCESS, 0

    def write2ByteTxRx(self, servo_id, address, value):
        if servo_id not in self.servos:
            return COMM_RX_TIMEOUT, 0
        self.servos[servo_id][address] = value
        return COMM_SUCCESS, 0

    def write1ByteTxRx(self, servo_id, address, value):
        if servo_id not in self.servos:
            return COMM_RX_TIMEOUT, 0
        regs = self.servos[servo_id]
        if address == so101_bus.REG_TORQUE_ENABLE and value == 128:
            # The servo takes its current position as the middle of the range.
            regs[so101_bus.REG_PRESENT_POSITION] = 2048
            regs["set_middle_calls"] = regs.get("set_middle_calls", 0) + 1
        else:
            regs[address] = value
        return COMM_SUCCESS, 0

    def WritePosEx(self, servo_id, position, speed, acc):
        if servo_id not in self.servos:
            return COMM_RX_TIMEOUT, 0
        regs = self.servos[servo_id]
        regs["goal"] = (position, speed, acc)
        regs[so101_bus.REG_PRESENT_POSITION] = position
        return COMM_SUCCESS, 0

    def unLockEprom(self, servo_id):
        if servo_id not in self.servos:
            return COMM_RX_TIMEOUT, 0
        self.servos[servo_id]["lock"] = 0
        return COMM_SUCCESS, 0

    def LockEprom(self, servo_id):
        if servo_id not in self.servos:
            return COMM_RX_TIMEOUT, 0
        self.servos[servo_id]["lock"] = 1
        return COMM_SUCCESS, 0

    def getTxRxResult(self, result):
        return f"result {result}"

    def getRxPacketError(self, error):
        return "[ServoStatus] Overload error!" if error & 32 else ""


def healthy_servo(position=2048, regs=None, **extra):
    """Register dict of a well-configured servo; `regs` overrides register addresses."""
    defaults = {
        so101_bus.REG_MODEL: 1540,
        so101_bus.REG_BAUD_RATE: 0,
        so101_bus.REG_MIN_ANGLE_LIMIT: 0,
        so101_bus.REG_MAX_ANGLE_LIMIT: 4095,
        so101_bus.REG_OFFSET: 0,
        so101_bus.REG_MODE: 0,
        so101_bus.REG_TORQUE_ENABLE: 1,
        so101_bus.REG_PRESENT_POSITION: position,
        so101_bus.REG_PRESENT_VOLTAGE: 74,
        so101_bus.REG_PRESENT_TEMPERATURE: 35,
        so101_bus.REG_MOVING: 0,
    }
    defaults.update(regs or {})
    defaults.update(extra)
    return defaults


class BusHelperTests(unittest.TestCase):
    def test_parse_ids(self):
        self.assertEqual(parse_ids("1-6"), [1, 2, 3, 4, 5, 6])
        self.assertEqual(parse_ids("3,1, 2"), [1, 2, 3])
        self.assertEqual(parse_ids("250-260"), [250, 251, 252])

    def test_decode_offset_sign_magnitude(self):
        self.assertEqual(decode_offset(0), 0)
        self.assertEqual(decode_offset(100), 100)
        self.assertEqual(decode_offset(0x800 | 100), -100)


class ScanTests(unittest.TestCase):
    def test_scan_ids_returns_models_of_answering_servos(self):
        handler = FakePacketHandler({1: healthy_servo(), 3: healthy_servo(regs={so101_bus.REG_MODEL: 777})})

        self.assertEqual(scan_ids(handler, [1, 2, 3, 4]), {1: 1540, 3: 777})


class PositionTests(unittest.TestCase):
    def test_read_positions_reports_position_speed_moving_and_errors(self):
        handler = FakePacketHandler({
            1: healthy_servo(2011, speed=-5),
            2: healthy_servo(1500, regs={so101_bus.REG_MOVING: 1}, error=32),
        })

        readings = read_positions(handler, [1, 2, 3])

        self.assertEqual(sorted(readings), [1, 2])
        self.assertEqual(readings[1], {"position": 2011, "speed": -5, "moving": False, "error": ""})
        self.assertEqual(readings[2]["moving"], True)
        self.assertIn("Overload", readings[2]["error"])

    def test_set_zero_positions_makes_servos_read_middle(self):
        handler = FakePacketHandler({
            1: healthy_servo(2011),
            2: healthy_servo(1500, regs={so101_bus.REG_TORQUE_ENABLE: 0}),
        })
        logs = []

        results = set_zero_positions(handler, [1, 2, 3], log=logs.append)

        self.assertEqual(results, {
            1: {"before": 2011, "after": 2048, "ok": True},
            2: {"before": 1500, "after": 2048, "ok": True},
        })
        self.assertEqual(handler.servos[1]["set_middle_calls"], 1)
        # Torque state is restored: on for servo 1, left off for servo 2.
        self.assertEqual(handler.servos[1][so101_bus.REG_TORQUE_ENABLE], 1)
        self.assertEqual(handler.servos[2][so101_bus.REG_TORQUE_ENABLE], 0)
        # EEPROM is locked again afterwards.
        self.assertEqual(handler.servos[1]["lock"], 1)
        self.assertTrue(any("ID 3" in line and "skipped" in line for line in logs))


class ConfigTests(unittest.TestCase):
    def test_read_config_decodes_registers(self):
        handler = FakePacketHandler({1: healthy_servo(2011, regs={so101_bus.REG_OFFSET: 0x800 | 40})})

        config = read_config(handler, 1)

        self.assertEqual(config["model"], 1540)
        self.assertEqual(config["baudrate"], 1000000)
        self.assertEqual(config["mode"], "position")
        self.assertEqual(config["offset"], -40)
        self.assertEqual(config["torque"], 1)
        self.assertEqual(config["position"], 2011)
        self.assertEqual(config["voltage"], 7.4)
        self.assertEqual(config["temperature"], 35)
        self.assertEqual(check_config(config), [])

    def test_read_config_returns_none_for_missing_servo(self):
        self.assertIsNone(read_config(FakePacketHandler({}), 1))

    def test_check_config_warns_about_workshop_problems(self):
        handler = FakePacketHandler({
            2: healthy_servo(regs={
                so101_bus.REG_BAUD_RATE: 4,
                so101_bus.REG_MODE: 1,
                so101_bus.REG_TORQUE_ENABLE: 0,
                so101_bus.REG_PRESENT_VOLTAGE: 50,
            })
        })

        warnings = check_config(read_config(handler, 2))

        self.assertEqual(len(warnings), 4)
        self.assertTrue(any("115200" in w for w in warnings))
        self.assertTrue(any("wheel" in w for w in warnings))
        self.assertTrue(any("torque" in w for w in warnings))
        self.assertTrue(any("5.0 V" in w for w in warnings))


if __name__ == "__main__":
    unittest.main()
