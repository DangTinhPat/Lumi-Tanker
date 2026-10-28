"""Controller GUI contracts after removing the camera/follower path."""

import builtins
import importlib
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))

from tap_gui.command_router import RobotCommandRouter
from tap_gui.motion_api import MotionAPI


class FakeBackend:
    def __init__(self):
        self.sent = []

    def command_drive(self, linear, yaw):
        self.sent.append(("DRIVE", linear, yaw))
        return True

    def command_stop(self):
        self.sent.append(("STOP",))
        return True


class ControllerOnlyTest(unittest.TestCase):
    def setUp(self):
        self.backend = FakeBackend()
        self.ready = True
        self.router = RobotCommandRouter(
            MotionAPI(self.backend), can_control=lambda: self.ready,
            send_console=lambda line: self.backend.sent.append(line),
            emergency_stop=lambda: self.backend.sent.append("ESTOP"))

    def test_interfaces_import_without_camera_dependencies(self):
        original = builtins.__import__

        def without_camera(name, *args, **kwargs):
            if name.split(".")[0] in {"cv2", "onnxruntime", "numpy"} or "vision" in name:
                raise ImportError("camera dependency removed: " + name)
            return original(name, *args, **kwargs)

        with patch("builtins.__import__", side_effect=without_camera):
            hardware = importlib.import_module("tap_serial_control_gui")
            native = importlib.import_module("tap_gui.native_dashboard")
        self.assertFalse(hasattr(hardware.TapSerialControlApp, "toggle_vision"))
        self.assertFalse(hasattr(native.NativeDashboard, "toggle_vision"))

    def test_removed_follow_commands_cannot_send_motion(self):
        result = self.router.execute({
            "command_id": "old-follow", "source": "luvox",
            "action": "follow", "args": {"enabled": True}})
        self.assertEqual(result.status, "rejected")
        self.assertFalse(self.router.command_velocity("tracking", .1, .1))
        self.assertEqual(self.backend.sent, [])

    def test_manual_drive_stop_and_balance_gate_remain(self):
        self.assertTrue(self.router.command_velocity("manual_ui", .1, .2))
        self.assertEqual(self.backend.sent[-1], ("DRIVE", .1, .2))
        self.router.stop_source("manual_ui")
        self.assertEqual(self.backend.sent[-1], ("STOP",))
        self.ready = False
        count = len(self.backend.sent)
        self.assertFalse(self.router.command_velocity("manual_ui", .1, 0))
        self.assertEqual(len(self.backend.sent), count)

    def test_estop_and_height_release_work_without_balance(self):
        self.ready = False
        self.assertTrue(self.router.height_hold(0))
        self.assertEqual(self.backend.sent[-1], "HMOVE 0")
        result = self.router.execute({
            "command_id": "safety", "source": "system",
            "action": "emergency_stop", "args": {}})
        self.assertEqual(result.status, "completed")
        self.assertEqual(self.backend.sent[-1], "ESTOP")


if __name__ == "__main__":
    unittest.main()
