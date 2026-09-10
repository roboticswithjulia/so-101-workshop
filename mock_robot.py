class MockRobot:
    """Simple robot backend used for demos, tests, and workshop onboarding."""

    def __init__(self):
        self.position = {"x": 0, "y": 0, "z": 0}
        self.gripper_open = True
        self.emergency_stop_active = False

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
