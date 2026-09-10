class RobotController:
    """Safe controller for beginner-friendly robot commands."""

    MAX_RANGE = 100

    def __init__(self, robot):
        self.robot = robot

    def _is_safe_position(self, position):
        for value in (position.get("x", 0), position.get("y", 0), position.get("z", 0)):
            if abs(value) > self.MAX_RANGE:
                return False
        return True

    def home(self):
        try:
            self.robot.home()
            return {"success": True, "message": "Robot returned to home position."}
        except Exception as exc:  # pragma: no cover - protective fallback
            return {"success": False, "message": f"Home failed: {exc}"}

    def safe_move(self, position):
        if self.robot.emergency_stop_active:
            return {"success": False, "message": "Emergency stop is active. Reset the robot before moving."}
        if not self._is_safe_position(position):
            return {"success": False, "message": "Requested position is out of range for a safe workshop move."}
        try:
            self.robot.move_to(position["x"], position["y"], position["z"])
            return {"success": True, "message": "Robot moved successfully."}
        except Exception as exc:  # pragma: no cover - protective fallback
            return {"success": False, "message": f"Movement failed: {exc}"}

    def open_gripper(self):
        try:
            self.robot.open_gripper()
            return {"success": True, "message": "Gripper opened."}
        except Exception as exc:  # pragma: no cover - protective fallback
            return {"success": False, "message": f"Open gripper failed: {exc}"}

    def close_gripper(self):
        try:
            self.robot.close_gripper()
            return {"success": True, "message": "Gripper closed."}
        except Exception as exc:  # pragma: no cover - protective fallback
            return {"success": False, "message": f"Close gripper failed: {exc}"}

    def emergency_stop(self):
        self.robot.emergency_stop()
        return {"success": True, "message": "Emergency stop activated."}

    def reset_emergency_stop(self):
        self.robot.reset_emergency_stop()
        return {"success": True, "message": "Emergency stop reset."}
