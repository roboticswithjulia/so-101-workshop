import unittest

import real_robot
from real_robot import RealRobotAdapter
from robot_controller import RobotController
from test_servo_tools import FakePacketHandler, healthy_servo


class FakePortHandler:
    def __init__(self):
        self.closed = False

    def closePort(self):
        self.closed = True


def connected_adapter(servos):
    adapter = RealRobotAdapter()
    adapter.packet_handler = FakePacketHandler(servos)
    adapter.port_handler = FakePortHandler()
    adapter.connected = True
    adapter.responding_ids = sorted(servos)
    return adapter


def full_arm(positions=None):
    positions = positions or {}
    return {servo_id: healthy_servo(positions.get(servo_id, 2048)) for servo_id in range(1, 7)}


class RealRobotMotionTests(unittest.TestCase):
    def test_move_to_is_an_offset_from_the_zero_pose(self):
        adapter = connected_adapter(full_arm())

        adapter.move_to(10, 0, -5)

        goals = {servo_id: adapter.packet_handler.servos[servo_id]["goal"] for servo_id in (1, 2, 3)}
        self.assertEqual(goals[1], (2148, RealRobotAdapter.SAFE_SPEED, RealRobotAdapter.SAFE_ACC))
        self.assertEqual(goals[2][0], 2048)
        self.assertEqual(goals[3][0], 1998)

    def test_move_is_clamped_to_the_servo_range(self):
        adapter = connected_adapter(full_arm())
        adapter.STEPS_PER_UNIT = 100

        adapter.move_to(100, -100, 0)

        self.assertEqual(adapter.packet_handler.servos[1]["goal"][0], 4095)
        self.assertEqual(adapter.packet_handler.servos[2]["goal"][0], 0)

    def test_home_sends_every_joint_to_the_zero_pose(self):
        adapter = connected_adapter(full_arm({1: 1500, 6: 3000}))

        adapter.home()

        for servo_id in range(1, 7):
            self.assertEqual(adapter.packet_handler.servos[servo_id]["goal"][0], 2048)

    def test_gripper_open_is_zero_and_close_is_offset(self):
        adapter = connected_adapter(full_arm())

        adapter.close_gripper()
        self.assertEqual(adapter.packet_handler.servos[6]["goal"][0], 2048 + RealRobotAdapter.GRIPPER_CLOSE_STEPS)
        adapter.open_gripper()
        self.assertEqual(adapter.packet_handler.servos[6]["goal"][0], 2048)

    def test_emergency_stop_holds_current_positions_and_blocks_motion(self):
        adapter = connected_adapter(full_arm({2: 1700}))

        adapter.emergency_stop()

        self.assertEqual(adapter.packet_handler.servos[2]["goal"][0], 1700)
        with self.assertRaises(RuntimeError):
            adapter.move_to(1, 1, 1)
        adapter.reset_emergency_stop()
        adapter.move_to(1, 1, 1)

    def test_motion_requires_connection(self):
        with self.assertRaises(RuntimeError):
            RealRobotAdapter().home()


class RealRobotCalibrationTests(unittest.TestCase):
    def test_set_zero_stores_the_current_pose_in_every_servo(self):
        adapter = connected_adapter(full_arm({1: 2011, 2: 1500}))

        results = adapter.set_zero()

        self.assertEqual(sorted(results), [1, 2, 3, 4, 5, 6])
        self.assertEqual(results[1], {"before": 2011, "after": 2048, "ok": True})
        self.assertTrue(all(r["ok"] for r in results.values()))
        self.assertEqual(adapter.read_positions()[2]["position"], 2048)

    def test_controller_reports_set_zero_outcome(self):
        controller = RobotController(connected_adapter(full_arm()))

        result = controller.set_zero()

        self.assertTrue(result["success"])
        self.assertIn("[1, 2, 3, 4, 5, 6]", result["message"])

    def test_controller_refuses_set_zero_during_emergency_stop(self):
        adapter = connected_adapter(full_arm())
        adapter.emergency_stop()

        result = RobotController(adapter).set_zero()

        self.assertFalse(result["success"])
        self.assertIn("mergency", result["message"])


class RealRobotConnectTests(unittest.TestCase):
    def setUp(self):
        self.original_open_bus = real_robot.open_bus
        self.original_discover = real_robot.discover_ports

    def tearDown(self):
        real_robot.open_bus = self.original_open_bus
        real_robot.discover_ports = self.original_discover

    def test_connect_picks_the_port_where_servos_answer(self):
        buses = {"/dev/empty": FakePacketHandler({}), "/dev/arm": FakePacketHandler(full_arm())}
        handlers = {}

        def fake_open_bus(port, baudrate):
            handlers[port] = FakePortHandler()
            return handlers[port], buses[port]

        real_robot.open_bus = fake_open_bus
        real_robot.discover_ports = lambda: ["/dev/empty", "/dev/arm"]
        adapter = RealRobotAdapter()

        self.assertTrue(adapter.connect())

        self.assertEqual(adapter.port, "/dev/arm")
        self.assertEqual(adapter.responding_ids, [1, 2, 3, 4, 5, 6])
        self.assertTrue(handlers["/dev/empty"].closed)
        self.assertFalse(handlers["/dev/arm"].closed)

    def test_connect_fails_when_nothing_answers(self):
        real_robot.open_bus = lambda port, baudrate: (FakePortHandler(), FakePacketHandler({}))
        adapter = RealRobotAdapter(port="/dev/ttyACM0")

        with self.assertRaises(RuntimeError):
            adapter.connect()
        self.assertFalse(adapter.connected)


if __name__ == "__main__":
    unittest.main()
