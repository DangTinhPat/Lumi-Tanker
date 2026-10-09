import unittest
import io
import os
import time
from types import SimpleNamespace

from gui.native_backend import NativeBackend
from gui.native_dashboard import NativeDashboard
from gui.motion_api import ActionStatus, MotionAPI
from gui.native_viewer import latest_view


class NativeBackendTest(unittest.TestCase):
    def test_viewer_discards_stale_frames_and_preserves_partial_line(self):
        latest, partial = latest_view(b"VIEW|t=1\nVIEW|t=2\nVIEW|t=3")
        self.assertEqual(latest, b"VIEW|t=2")
        self.assertEqual(partial, b"VIEW|t=3")
        latest, partial = latest_view(partial+b"\n")
        self.assertEqual((latest, partial), (b"VIEW|t=3", b""))
        self.assertEqual(latest_view(b"invalid\n"), (None, b""))
        self.assertEqual(latest_view(b"x"*65537), (None, b""))

    def test_button_immediate_command_respects_readiness_and_estop(self):
        for enabled, estop, running, ready in ((True, False, True, "0"),
                                               (False, False, True, "1"),
                                               (True, True, True, "1"),
                                               (True, False, False, "1")):
            sent = []
            dashboard = SimpleNamespace(
                enabled=enabled, estop=estop, vision_process=SimpleNamespace(running=False),
                backend=SimpleNamespace(running=running, telemetry={"drive_ready": ready}, log=[]),
                _cancel_pending_posture=lambda: None, stop_skill=lambda: None,
                speed=SimpleNamespace(get=lambda: .3), yaw=SimpleNamespace(get=lambda: .3),
                drive=(.3, 0), _drive_stop_latched=False,
                motion=SimpleNamespace(command_velocity=lambda v, w: sent.append((v, w))))
            dashboard.set_drive = lambda f, y: NativeDashboard.set_drive(dashboard, f, y)
            NativeDashboard._drive_press(dashboard, 1, 0)
            self.assertEqual(sent, [])

    def test_reader_keeps_bounded_recent_telemetry(self):
        backend = NativeBackend()
        backend.process = SimpleNamespace(stdout=io.StringIO(
            "".join(f"STATE|t={i}\n" for i in range(1000))))
        backend._read_output()
        self.assertEqual(backend.lines.qsize(), 256)
        backend.drain()
        self.assertEqual(backend.telemetry["t"], "999")

    def test_drain_sends_only_latest_view(self):
        backend = NativeBackend()
        sent = []
        backend._sync_mujoco_viewer = lambda: sent.append(backend.view.copy())
        for i in range(100):
            backend.lines.put_nowait(f"VIEW|t={i}|x={i}")
        self.assertEqual(backend.drain(), 100)
        self.assertEqual(sent, [{"t": "99", "x": "99"}])

    def test_busy_viewer_cannot_block_gui(self):
        read_fd, write_fd = os.pipe()
        os.set_blocking(write_fd, False)
        try:
            while True:
                try:
                    os.write(write_fd, b"x"*4096)
                except BlockingIOError:
                    break
            backend = NativeBackend()
            backend._viewer_process = SimpleNamespace(
                poll=lambda: None, stdin=SimpleNamespace(fileno=lambda: write_fd))
            backend.view = {"t": "1", "x": "0"}
            start = time.monotonic()
            backend._sync_mujoco_viewer()
            self.assertLess(time.monotonic()-start, .05)
        finally:
            os.close(read_fd)
            os.close(write_fd)

    def test_direction_button_release_stops_once(self):
        for direction in (1, -1):
            commands = []
            dashboard = SimpleNamespace(
                enabled=True, estop=False, vision_process=SimpleNamespace(running=False),
                backend=SimpleNamespace(running=True, telemetry={"drive_ready": "1"}, log=[]),
                _cancel_pending_posture=lambda: None, stop_skill=lambda: None,
                speed=SimpleNamespace(get=lambda: .3), yaw=SimpleNamespace(get=lambda: .3),
                drive=(0, 0), _drive_stop_latched=True, _drive_button_active=False,
                motion=SimpleNamespace(stop=lambda: commands.append("STOP"),
                                       command_velocity=lambda v, w: commands.append(("DRIVE", v, w))))
            dashboard.set_drive = lambda f, y: NativeDashboard.set_drive(dashboard, f, y)
            dashboard.stop_drive = lambda: NativeDashboard.stop_drive(dashboard)
            NativeDashboard._drive_press(dashboard, direction, 0)
            self.assertEqual(dashboard.drive, (direction*.3, 0))
            self.assertFalse(dashboard._drive_stop_latched)
            self.assertEqual(commands, [("DRIVE", direction*.3, 0)])
            NativeDashboard._drive_release(dashboard)
            NativeDashboard._drive_release(dashboard)
            self.assertEqual(dashboard.drive, (0, 0))
            self.assertTrue(dashboard._drive_stop_latched)
            self.assertEqual(commands, [("DRIVE", direction*.3, 0), "STOP"])

    def test_speed_slider_updates_active_drive_without_starting_motion(self):
        for initial, expected in (((.3, .2), (.5, .2)),
                                  ((-.3, 0), (-.5, 0)),
                                  ((0, .2), (0, .2)), ((0, 0), (0, 0))):
            displayed = []
            dashboard = SimpleNamespace(
                drive=initial, speed=SimpleNamespace(get=lambda: .5),
                speed_readout=SimpleNamespace(set=displayed.append))
            NativeDashboard._speed_preview(dashboard)
            self.assertEqual(dashboard.drive, expected)
            self.assertEqual(displayed, ["0.50 m/s"])

    def test_parse_is_key_value_only(self):
        fields = NativeBackend._parse("STATE|mode=Balance|pitch=0.1|ground_truth_to_controller=0")
        self.assertEqual(fields["mode"], "Balance")
        self.assertEqual(fields["ground_truth_to_controller"], "0")

    def test_runner_is_source_tree_native_binary(self):
        backend = NativeBackend()
        runner = backend.runner_path()
        self.assertTrue(runner.name == "tap_mujoco_gim_v3")
        self.assertTrue(runner.is_file())

    def test_v3_finite_motion_uses_drive_ready(self):
        backend = SimpleNamespace(
            running=True, telemetry_received_monotonic_s=time.monotonic(),
            telemetry={"mode": "Balancing", "drive_ready": "1", "wheel_position": "0",
                       "wheel_left_position": "0", "wheel_right_position": "0"},
            command_stop=lambda: None, command_drive=lambda v, w: None)
        motion = MotionAPI(backend)
        self.assertTrue(motion.forward(.1, distance_m=.2))
        self.assertEqual(motion.update_action(), ActionStatus.RUNNING)
        backend.telemetry["drive_ready"] = "0"
        self.assertFalse(motion.forward(.1, distance_m=.2))

    def test_dashboard_accepts_v3_balance_and_blocks_fixture(self):
        for ready in ("0", "1"):
            dashboard = SimpleNamespace(
                enabled=True, estop=False, vision_process=SimpleNamespace(running=False),
                backend=SimpleNamespace(telemetry={"mode": "Balancing", "drive_ready": ready}, log=[]),
                _cancel_pending_posture=lambda: None, stop_skill=lambda: None,
                speed=SimpleNamespace(get=lambda: .2), yaw=SimpleNamespace(get=lambda: .3),
                drive=(0, 0), _drive_stop_latched=True)
            NativeDashboard.set_drive(dashboard, 1, 0)
            self.assertEqual(dashboard.drive, (.2, 0) if ready == "1" else (0, 0))


if __name__ == "__main__":
    unittest.main()
