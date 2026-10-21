"""MotionAPI finite moves: handed to the firmware (MOVE/TURN + MOT lines) when it has them."""
import os
import sys
import time
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from gui.motion_api import ActionStatus, MotionAPI, MotionLimits  # noqa: E402


class FakeBackend:
    running = True

    def __init__(self, firmware_motion: bool):
        self.sent: list[str] = []
        self.telemetry = {"mode": "Balancing", "wheel_position": "0.0", "wheel_left_position": "0.0",
                          "wheel_right_position": "0.0", "drive_ready": "1"}
        if firmware_motion:
            self.telemetry["fw_motion"] = "1"
        self.telemetry_received_monotonic_s = time.monotonic()

    def touch(self):
        self.telemetry_received_monotonic_s = time.monotonic()

    def command_drive(self, v, w):
        self.sent.append(f"DRIVE {v:.3f} {w:.3f}")
        return True

    def command_stop(self):
        self.sent.append("STOP")
        return True

    def command_move(self, d, v):
        self.sent.append(f"MOVE {d:.3f} {v:.3f}")
        return True

    def command_turn(self, a, w):
        self.sent.append(f"TURN {a:.3f} {w:.3f}")
        return True

    def mot(self, state):
        self.telemetry["motion_state"] = state
        self.telemetry["motion_seq"] = str(int(self.telemetry.get("motion_seq", "0")) + 1)


class MotionFirmwareApiTest(unittest.TestCase):
    def api(self, firmware_motion=True):
        backend = FakeBackend(firmware_motion)
        return backend, MotionAPI(backend, MotionLimits(max_linear_m_s=0.5, max_yaw_rad_s=1.0))

    def test_distance_goes_to_firmware_and_completes(self):
        backend, api = self.api()
        self.assertTrue(api.forward(0.3, distance_m=0.5))
        self.assertEqual(backend.sent[-1], "MOVE 0.500 0.300")
        backend.touch()
        self.assertEqual(api.update_action(), ActionStatus.RUNNING)   # no MOT answer yet: no PC drive
        self.assertNotIn("DRIVE", " ".join(backend.sent))
        backend.mot("running")
        backend.touch()
        self.assertEqual(api.update_action(), ActionStatus.RUNNING)
        backend.mot("done")
        backend.touch()
        self.assertEqual(api.update_action(), ActionStatus.COMPLETED)

    def test_turn_and_backward_signs(self):
        backend, api = self.api()
        self.assertTrue(api.backward(0.2, distance_m=0.4))
        self.assertEqual(backend.sent[-1], "MOVE -0.400 0.200")
        api.cancel_action()
        self.assertTrue(api.rotate(0.5, angle_rad=-1.0))
        self.assertEqual(backend.sent[-1], "TURN -1.000 0.500")

    def test_firmware_refusal_is_rejected(self):
        backend, api = self.api()
        api.forward(0.3, distance_m=0.5)
        backend.mot("refused")
        backend.touch()
        self.assertEqual(api.update_action(), ActionStatus.REJECTED)

    def test_without_firmware_support_the_pc_loop_is_used(self):
        backend, api = self.api(firmware_motion=False)
        self.assertTrue(api.forward(0.3, distance_m=0.5))
        backend.touch()
        api.update_action()
        self.assertTrue(any(c.startswith("DRIVE") for c in backend.sent))
        self.assertFalse(any(c.startswith("MOVE") for c in backend.sent))


if __name__ == "__main__":
    unittest.main()
