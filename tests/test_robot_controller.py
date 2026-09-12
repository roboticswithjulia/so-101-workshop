import unittest

from robot_controller import RobotController
from mock_robot import MockRobot


class RobotControllerTests(unittest.TestCase):
    def setUp(self):
        self.robot = MockRobot()
        self.controller = RobotController(self.robot)

    def test_home_resets_position_and_gripper(self):
        self.robot.position = {"x": 12, "y": 8, "z": 3}
        self.robot.gripper_open = False

        result = self.controller.home()

        self.assertTrue(result["success"])
        self.assertEqual(self.robot.position, {"x": 0, "y": 0, "z": 0})
        self.assertTrue(self.robot.gripper_open)

    def test_safe_move_rejects_risky_coordinates(self):
        result = self.controller.safe_move({"x": 999, "y": 0, "z": 0})

        self.assertFalse(result["success"])
        self.assertIn("out of range", result["message"].lower())

    def test_open_and_close_gripper(self):
        self.controller.open_gripper()
        self.assertTrue(self.robot.gripper_open)

        self.controller.close_gripper()
        self.assertFalse(self.robot.gripper_open)

    def test_emergency_stop_blocks_further_motion(self):
        self.controller.emergency_stop()
        result = self.controller.safe_move({"x": 10, "y": 0, "z": 0})

        self.assertFalse(result["success"])
        self.assertIn("emergency", result["message"].lower())

    def test_set_zero_stores_current_pose_in_demo_mode(self):
        result = self.controller.set_zero()

        self.assertTrue(result["success"])
        self.assertTrue(all(p == MockRobot.ZERO for p in self.robot.servo_positions.values()))

    def test_set_zero_is_refused_during_emergency_stop(self):
        self.controller.emergency_stop()

        result = self.controller.set_zero()

        self.assertFalse(result["success"])

    def test_reset_emergency_stop_allows_motion_again(self):
        self.controller.emergency_stop()
        self.controller.reset_emergency_stop()

        result = self.controller.safe_move({"x": 10, "y": 0, "z": 0})

        self.assertTrue(result["success"])


if __name__ == "__main__":
    unittest.main()
