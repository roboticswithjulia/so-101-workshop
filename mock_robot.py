class MockRobot:
    """Simple robot backend used for demos, tests, and workshop onboarding."""

    ZERO = 2048

    def __init__(self):
        self.position = {"x": 0, "y": 0, "z": 0}
        self.gripper_open = True
        self.emergency_stop_active = False
        self.servo_ids = [1, 2, 3, 4, 5, 6]
        # Simulated raw servo positions, deliberately away from the zero pose.
        self.servo_positions = {servo_id: self.ZERO - 200 + 60 * servo_id for servo_id in self.servo_ids}

    def move_to(self, x, y, z):
        if self.emergency_stop_active:
            raise RuntimeError("Robot is in emergency stop mode.")
        self.position = {"x": x, "y": y, "z": z}
        return {"x": x, "y": y, "z": z}

    def open_gripper(self):
        if self.emergency_stop_active:
            raise RuntimeError("Robot is in emergency stop mode.")
        self.gripper_open = True
        return True

    def close_gripper(self):
        if self.emergency_stop_active:
            raise RuntimeError("Robot is in emergency stop mode.")
        self.gripper_open = False
        return False

    def emergency_stop(self):
        self.emergency_stop_active = True
        return True

    def reset_emergency_stop(self):
        self.emergency_stop_active = False
        return True

    def home(self):
        self.position = {"x": 0, "y": 0, "z": 0}
        self.gripper_open = True
        return {"x": 0, "y": 0, "z": 0}

    def read_positions(self, ids=None):
        return {
            servo_id: {"position": self.servo_positions[servo_id], "speed": 0, "moving": False, "error": ""}
            for servo_id in (ids or self.servo_ids)
        }

    def set_zero(self, ids=None, log=None):
        """Simulate storing the current pose as the zero pose (same result shape as the real robot)."""
        results = {}
        for servo_id in ids or self.servo_ids:
            before = self.servo_positions[servo_id]
            self.servo_positions[servo_id] = self.ZERO
            results[servo_id] = {"before": before, "after": self.ZERO, "ok": True}
        return results
