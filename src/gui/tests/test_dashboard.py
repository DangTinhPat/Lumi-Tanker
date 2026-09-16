"""Callback regressions without a Tk window or ROS node."""
from pathlib import Path
import json
import math
import os
import sys
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'tap_teleop'))
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from gui.dashboard import ALL_JOINTS, Dashboard, parse_options
from tap_teleop.pose_curves import JOINTS


class DashboardStopTest(unittest.TestCase):
    def setUp(self):
        self.gui = Dashboard.__new__(Dashboard)
        self.gui.refresh_controls = Mock()
        self.gui.updating_controls = False
        self.gui.enabled = True
        self.gui.jump_locked = False
        self.gui.experimental_jump = False
        self.gui.pose_commands_blocked = False
        self.gui.estopped = False
        self.gui.drive = (.1, .2)
        self.gui.feedback_grace_until = 0.
        self.gui.last_mode = 0.
        self.gui.show = None
        self.gui.show_text = 'Show idle'
        self.gui.show_status = Mock()
        self.gui.workflow = Value('Native joints')
        self.gui.deep_text = ''
        self.gui.deep_trajectory = None
        self.gui.deep_returning = False
        self.gui.pub = {name: Mock() for name in
                        ('cmd_vel', 'enable', 'emergency_stop', 'reset_controller',
                         'stand_up', 'target_pose')}
        self.gui.node = Mock()
        self.gui.root = Mock()
        self.gui.status = Mock()
        self.gui.motors = Mock()
        self.gui.telemetry = Mock()
        self.gui.mode_text = 'Balance'
        self.gui.motor_text = ''
        self.gui.telemetry_text = ''
        self.gui.target = Mock(return_value=[0., 0., 0., 0.])
        self.gui.experimental = Mock()
        self.gui.experimental.get.return_value = True
        self.gui.sim_process = None
        self.gui.reset_standup_stage = None
        self.gui.recording = Value(False)
        self.gui.csv_writer = None
        self.gui.pitch_history = []

    def feedback(self, text):
        self.gui.on_mode(String(data=text))

    def tick_at(self, now, elapsed=.05):
        self.gui.last_tick = now - elapsed
        with patch('gui.dashboard.rclpy.ok', return_value=True), \
             patch('gui.dashboard.rclpy.spin_once'), \
             patch('gui.dashboard.time.monotonic', return_value=now):
            self.gui.tick()

    def test_explicit_activation_has_bounded_first_feedback_grace(self):
        for activate in (self.gui.enable, self.gui.stand_up):
            with self.subTest(activate=activate.__name__):
                with patch('gui.dashboard.time.monotonic', return_value=10.):
                    activate()
                self.assertEqual(self.gui.last_mode, 0.)
                self.gui.pub['enable'].reset_mock()
                for now in (10.05, 10.7, 11.99):
                    self.tick_at(now)
                    self.assertTrue(self.gui.enabled)
                    self.gui.pub['enable'].publish.assert_not_called()
                self.tick_at(12.01)
                self.assertFalse(self.gui.enabled)
                self.gui.pub['enable'].publish.assert_called_once()

    def test_feedback_drain_is_bounded_and_ui_tick_is_20ms(self):
        self.gui.last_tick = 99.98
        self.gui.last_mode = 100.
        with patch('gui.dashboard.rclpy.ok', return_value=True), \
             patch('gui.dashboard.rclpy.spin_once') as spin, \
             patch('gui.dashboard.time.monotonic', return_value=100.):
            self.gui.tick()
        self.assertEqual(spin.call_count, 32)
        self.gui.root.after.assert_called_with(20, self.gui.tick)

    def test_first_feedback_ends_grace_and_normal_timeout_resumes(self):
        with patch('gui.dashboard.time.monotonic', return_value=10.):
            self.gui.enable()
        with patch('gui.dashboard.time.monotonic', return_value=10.1):
            self.feedback('Balance|estop=0')
        self.assertEqual(self.gui.feedback_grace_until, 0.)
        self.assertEqual(self.gui.last_mode, 10.1)
        self.tick_at(11.)
        self.assertTrue(self.gui.enabled)
        self.tick_at(11.11)
        self.assertFalse(self.gui.enabled)

    def test_ui_stall_still_disables_during_feedback_grace(self):
        with patch('gui.dashboard.time.monotonic', return_value=10.):
            self.gui.enable()
        self.tick_at(10.5, elapsed=.5)
        self.assertFalse(self.gui.enabled)

    def test_stall_watchdog_only_disables_armed_drive(self):
        self.gui.node = Mock()
        self.gui.root = Mock()
        self.gui.status = Mock()
        self.gui.motors = Mock()
        self.gui.telemetry = Mock()
        self.gui.mode_text = 'Balance'
        self.gui.motor_text = ''
        self.gui.telemetry_text = ''
        for armed in (False, True):
            with self.subTest(armed=armed):
                self.gui.enabled = armed
                self.gui.last_tick = 1.
                self.gui.last_mode = 0.
                self.gui.pub['enable'].reset_mock()
                with patch('gui.dashboard.rclpy.ok', return_value=True), \
                     patch('gui.dashboard.rclpy.spin_once'), \
                     patch('gui.dashboard.time.monotonic', return_value=3.):
                    self.gui.tick()
                if armed:
                    self.gui.pub['enable'].publish.assert_called_once()
                    self.assertFalse(self.gui.pub['enable'].publish.call_args.args[0].data)
                else:
                    self.gui.pub['enable'].publish.assert_not_called()
                self.assertFalse(self.gui.enabled)

    def test_clear_then_emergency_mode_does_not_relatch(self):
        self.gui.estop()
        self.gui.clear_estop()
        for mode in ('EmergencyStop', 'EStop'):
            self.feedback(mode + '|estop=0')
            self.assertFalse(self.gui.estopped)
            self.assertFalse(self.gui.enabled)
            self.assertEqual(self.gui.drive, (0., 0.))
        self.gui.enable()
        self.assertTrue(self.gui.enabled)
        self.assertTrue(self.gui.pub['enable'].publish.call_args.args[0].data)
        self.gui.reset()
        self.assertFalse(self.gui.enabled)
        self.assertTrue(self.gui.pub['reset_controller'].publish.call_args.args[0].data)

    def test_manual_latch_requires_explicit_clear(self):
        self.gui.estop()
        for mode in ('EmergencyStop', 'Disabled', 'Balance'):
            self.feedback(mode + '|estop=0')
            self.assertTrue(self.gui.estopped)
        self.gui.enable()
        self.assertFalse(self.gui.enabled)
        self.gui.clear_estop()
        self.assertFalse(self.gui.estopped)
        self.assertFalse(self.gui.enabled)

    def test_actual_estop_flag_latches_in_any_mode(self):
        self.feedback('Balance|estop=1')
        self.assertTrue(self.gui.estopped)
        self.assertFalse(self.gui.enabled)
        self.assertEqual(self.gui.drive, (0., 0.))

    def test_stop_modes_only_disarm_without_actual_estop(self):
        for mode in ('Fallen', 'Disabled', 'EmergencyStop', 'EStop'):
            self.gui.enabled = True
            self.gui.drive = (.1, .2)
            self.feedback(mode + '|estop=0')
            self.assertFalse(self.gui.enabled)
            self.assertFalse(self.gui.estopped)
            self.assertEqual(self.gui.drive, (0., 0.))


class OnModeLastPitchTest(unittest.TestCase):
    def setUp(self):
        self.gui = Dashboard.__new__(Dashboard)
        self.gui.mode_text = ''
        self.gui.last_mode = 0.
        self.gui.feedback_grace_until = 0.
        self.gui.deep_geometry_ready = False
        self.gui.controller_pose_bounds = None
        self.gui.operator_zero = False
        self.gui.station_ready = False
        self.gui.enabled = False
        self.gui.estopped = False
        self.gui.drive = (0., 0.)
        self.gui.deep_trajectory = None
        self.gui.stop_show = Mock()
        self.gui.stop_drive = Mock()
        self.gui.recording = Value(False)
        self.gui.csv_writer = None
        self.gui.pitch_history = []

    def test_valid_pitch_and_pose_z_are_retained(self):
        self.gui.on_mode(String(data='Balance|fresh=1|estop=0|pitch=.05|pose_z=.24'))
        self.assertAlmostEqual(self.gui.last_pitch, .05)
        self.assertAlmostEqual(self.gui.last_pose_z, .24)

    def test_missing_or_malformed_fields_clear_both(self):
        self.gui.on_mode(String(data='Balance|fresh=1|estop=0|pitch=.05|pose_z=.24'))
        self.gui.on_mode(String(data='Balance|fresh=1|estop=0'))
        self.assertIsNone(self.gui.last_pitch)
        self.assertIsNone(self.gui.last_pose_z)
        self.gui.on_mode(String(data='Balance|fresh=1|estop=0|pitch=.05|pose_z=.24'))
        self.gui.on_mode(String(data='Balance|fresh=1|estop=0|pitch=nan|pose_z=.24'))
        # pitch=nan parses as a float, so it is retained (not a KeyError/ValueError);
        # deep_geometry_ready is what turns non-finite values into "not ready".
        self.assertTrue(math.isnan(self.gui.last_pitch))
        self.assertFalse(self.gui.deep_geometry_ready)


class LaunchStopSimTest(unittest.TestCase):
    def setUp(self):
        self.gui = Dashboard.__new__(Dashboard)
        self.gui.sim_process = None
        self.gui.sim_stream = None
        self.gui.sim_log_path = None
        self.gui.sim_button = Mock()
        self.gui.sim_status = Mock()
        self.gui.sim_terrain_menu = Mock()
        self.gui.sim_terrain = Value('none')
        self.gui.sim_profile_menu = Mock()
        self.gui.sim_profile = Value('Default')
        self.gui.spawn_x = Value('0')
        self.gui.spawn_y = Value('0')
        self.gui.spawn_z = Value('0.2372')
        self.gui.node = Mock()
        self.gui.node.count_publishers.return_value = 0
        self.gui.enabled = False
        self.gui.estopped = True
        self.gui.feedback_grace_until = 0.
        self.gui.sim_started = 0.
        self.gui.csv_file = None

    def test_refuses_when_a_real_sim_is_already_publishing_clock(self):
        # Regression test: an earlier version checked
        # get_topic_names_and_types() for '/clock', which always "found" it
        # because this node's own /clock SUBSCRIPTION (created in __init__)
        # makes the topic appear in the graph listing regardless of whether
        # any publisher exists -- that permanently blocked Launch sim on
        # every single attempt. count_publishers is the correct check.
        self.gui.node.count_publishers.return_value = 1
        with patch('gui.dashboard.sim_process.launch_sim') as launch:
            self.gui.launch_sim()
        launch.assert_not_called()
        self.assertIsNone(self.gui.sim_process)
        self.gui.node.count_publishers.assert_called_with('/clock')

    def test_launches_when_no_clock_publisher_exists(self):
        with patch('gui.dashboard.sim_process.launch_sim',
                  return_value=(Mock(), '/tmp/sim.log', Mock())) as launch:
            self.gui.launch_sim()
        launch.assert_called_once_with(
            ['auto_enable:=true', 'spawn_x:=0', 'spawn_y:=0', 'spawn_z:=0.2372'])
        self.assertIsNotNone(self.gui.sim_process)
        self.gui.sim_button.configure.assert_called_with(text='Stop sim')
        self.gui.sim_terrain_menu.configure.assert_called_with(state='disabled')

    def test_passes_terrain_choice_as_a_launch_arg(self):
        self.gui.sim_terrain.set('rough')
        with patch('gui.dashboard.sim_process.launch_sim',
                  return_value=(Mock(), '/tmp/sim.log', Mock())) as launch:
            self.gui.launch_sim()
        launch.assert_called_once_with(
            ['auto_enable:=true', 'terrain:=rough', 'spawn_x:=0', 'spawn_y:=0', 'spawn_z:=0.2372'])

    def test_deep_squat_profile_passes_controller_config_launch_arg(self):
        self.gui.sim_profile.set('Deep squat')
        with patch('gui.dashboard.sim_process.launch_sim',
                  return_value=(Mock(), '/tmp/sim.log', Mock())) as launch:
            self.gui.launch_sim()
        args = launch.call_args.args[0]
        matching = [a for a in args if a.startswith('controller_config:=')]
        self.assertEqual(len(matching), 1)
        self.assertTrue(matching[0].endswith('config/deep_squat.yaml'))

    def test_default_profile_passes_no_controller_config_launch_arg(self):
        self.gui.sim_profile.set('Default')
        with patch('gui.dashboard.sim_process.launch_sim',
                  return_value=(Mock(), '/tmp/sim.log', Mock())) as launch:
            self.gui.launch_sim()
        args = launch.call_args.args[0]
        self.assertFalse(any(a.startswith('controller_config:=') for a in args))

    def test_passes_current_spawn_field_values_as_launch_args(self):
        self.gui.spawn_x.set('-4.6')
        self.gui.spawn_y.set('1.5')
        self.gui.spawn_z.set('0.4372')
        with patch('gui.dashboard.sim_process.launch_sim',
                  return_value=(Mock(), '/tmp/sim.log', Mock())) as launch:
            self.gui.launch_sim()
        launch.assert_called_once_with(
            ['auto_enable:=true', 'spawn_x:=-4.6', 'spawn_y:=1.5', 'spawn_z:=0.4372'])

    def test_launch_blocked_when_spawn_field_is_not_a_number(self):
        self.gui.spawn_x.set('not-a-number')
        with patch('gui.dashboard.sim_process.launch_sim') as launch:
            self.gui.launch_sim()
        launch.assert_not_called()
        self.assertIsNone(self.gui.sim_process)
        self.gui.sim_status.set.assert_called_with(
            'Launch blocked: spawn X is not a number')

    def test_idempotent_when_already_launched(self):
        self.gui.sim_process = Mock()
        with patch('gui.dashboard.sim_process.launch_sim') as launch:
            self.gui.launch_sim()
        launch.assert_not_called()

    def test_launch_failure_is_reported_not_raised(self):
        with patch('gui.dashboard.sim_process.launch_sim', side_effect=RuntimeError('boom')):
            self.gui.launch_sim()  # must not raise
        self.assertIsNone(self.gui.sim_process)
        self.gui.sim_status.set.assert_called_with('Launch failed: boom')

    def test_stop_sim_is_a_noop_without_a_launched_process(self):
        with patch('gui.dashboard.sim_process.stop_sim') as stop:
            self.gui.stop_sim()
        stop.assert_not_called()

    def test_stop_sim_calls_sim_process_and_resets_state(self):
        self.gui.sim_process, self.gui.sim_stream = Mock(), Mock()
        with patch('gui.dashboard.sim_process.stop_sim') as stop:
            self.gui.stop_sim()
        stop.assert_called_once()
        self.assertIsNone(self.gui.sim_process)
        self.gui.sim_button.configure.assert_called_with(text='Launch sim')
        self.gui.sim_terrain_menu.configure.assert_called_with(state='readonly')

    def test_tick_sim_syncs_enabled_once_running_milestone_is_observed(self):
        # Launch sim always requests auto_enable:=true, so
        # activate_controllers enables the real controller itself; the GUI's
        # own self.enabled bookkeeping must follow that, or Drive and the
        # other enabled-gated controls silently do nothing after a Launch
        # sim click even though the robot really is enabled.
        self.gui.sim_process = Mock(poll=Mock(return_value=None))
        self.gui.sim_log_path = '/tmp/sim.log'
        with patch('gui.dashboard.sim_process.sim_stage', return_value='Running (5s)'):
            self.gui._tick_sim(100.)
        self.assertTrue(self.gui.enabled)
        self.assertFalse(self.gui.estopped)

    def test_tick_sim_does_not_sync_enabled_before_running_milestone(self):
        self.gui.sim_process = Mock(poll=Mock(return_value=None))
        self.gui.sim_log_path = '/tmp/sim.log'
        with patch('gui.dashboard.sim_process.sim_stage',
                  return_value='Controllers ready, waiting for Enable (2s)'):
            self.gui._tick_sim(100.)
        self.assertFalse(self.gui.enabled)

    def test_close_stops_only_a_sim_launched_via_the_button(self):
        self.gui.jump_locked = False
        self.gui.disable = Mock()
        self.gui.stop_sim = Mock()
        self.gui.root = Mock()
        self.gui.close()
        self.gui.stop_sim.assert_not_called()
        self.gui.sim_process = Mock()
        self.gui.close()
        self.gui.stop_sim.assert_called_once()


class ResetStandupGateTest(unittest.TestCase):
    def setUp(self):
        self.gui = Dashboard.__new__(Dashboard)
        self.gui.experimental = Value(True)
        self.gui.jump_locked = False
        self.gui.deep_trajectory = None
        self.gui.show = None
        self.gui.reset_standup_stage = None
        self.gui.reset_standup_status = Mock()
        self.gui.sim_time = 10.
        self.gui.clock_wall = 100.
        self.gui.stop_drive = Mock()
        self.gui.stop_show = Mock()
        self.gui.stop_deep = Mock()
        self.gui.spawn_x = Value('0')
        self.gui.spawn_y = Value('0')
        self.gui.spawn_z = Value('0.2372')
        self.gui._reset_standup_target = (0.0, 0.0, 0.2372)

    def start(self, now=100.):
        with patch('gui.dashboard.time.monotonic', return_value=now), \
             patch('gui.dashboard.threading.Thread') as thread_cls:
            self.gui.reset_standup()
        return thread_cls

    def test_blocked_when_not_experimental_or_busy_or_stale_clock(self):
        cases = [
            dict(experimental=False),
            dict(jump_locked=True),
            dict(deep_trajectory=object()),
            dict(show=object()),
            dict(reset_standup_stage='teleporting'),
            dict(sim_time=None),
            dict(clock_wall=0.),
        ]
        for overrides in cases:
            with self.subTest(overrides=overrides):
                self.setUp()
                if 'experimental' in overrides:
                    self.gui.experimental.set(overrides['experimental'])
                for key, value in overrides.items():
                    if key != 'experimental':
                        setattr(self.gui, key, value)
                thread_cls = self.start()
                thread_cls.assert_not_called()
                self.gui.stop_drive.assert_not_called()

    def test_happy_path_starts_background_thread(self):
        thread_cls = self.start()
        thread_cls.assert_called_once()
        self.assertEqual(self.gui.reset_standup_stage, 'teleporting')
        self.gui.stop_drive.assert_called_once()
        thread_cls.return_value.start.assert_called_once()

    def test_reads_current_spawn_field_values_as_the_teleport_target(self):
        self.gui.spawn_x.set('-4.6')
        self.gui.spawn_y.set('0.3')
        self.gui.spawn_z.set('0.4372')
        self.start()
        self.assertEqual(self.gui._reset_standup_target, (-4.6, 0.3, 0.4372))

    def test_blocked_when_a_spawn_field_is_not_a_number(self):
        self.gui.spawn_z.set('nope')
        thread_cls = self.start()
        thread_cls.assert_not_called()
        self.gui.reset_standup_status.set.assert_called_with(
            'Reset upright blocked: spawn Z is not a number')


class ResetStandupHandoffTest(unittest.TestCase):
    def setUp(self):
        self.gui = Dashboard.__new__(Dashboard)
        self.gui.reset_standup_status = Mock()
        self.gui.reset_standup_error = ''
        self.gui.reset_standup_confirm_start = None
        self.gui.reset_standup_confirm_deadline = 10000.
        self.gui.pose_commands_blocked = False
        self.gui.mode_text = 'Fallen'
        self.gui.last_pitch = None
        self.gui.last_mode = 0.
        self.gui.enabled = False
        self.gui.estopped = True
        self.gui.feedback_grace_until = 0.

    def advance(self, stage, now=100.):
        self.gui.reset_standup_stage = stage
        with patch('gui.gz_world.pause') as pause:
            self.gui.pause_mock = pause
            self.gui._advance_reset_standup(now)

    def test_confirming_sync_updates_bookkeeping_without_replaying_reset_or_estop(self):
        # The background gz phase (_reset_standup_gz_phase) already published
        # reset_controller / cleared estop / enabled directly, before
        # unpausing. This stage must only sync the GUI's own self.enabled/
        # self.estopped bookkeeping to match -- calling reset()/clear_estop()
        # again here would call disable() internally (-> /enable=false ->
        # command_.request_disable=true -> immediate EmergencyStop),
        # undoing the enable the background phase just established. Assert
        # no gz_world.pause call here either: unpausing already happened in
        # the background phase, not in this stage.
        self.advance('confirming_sync', now=100.)
        self.assertTrue(self.gui.enabled)
        self.assertFalse(self.gui.estopped)
        self.gui.pause_mock.assert_not_called()
        self.assertEqual(self.gui.reset_standup_stage, 'confirming')
        self.assertEqual(self.gui.reset_standup_confirm_deadline, 115.)
        self.assertIsNone(self.gui.reset_standup_confirm_start)

    def test_confirming_requires_two_continuous_seconds_and_resets_on_interruption(self):
        self.gui.mode_text = 'Balance'
        self.gui.last_pitch = .01
        self.gui.last_mode = 100.
        self.advance('confirming', now=100.)
        self.assertEqual(self.gui.reset_standup_confirm_start, 100.)
        self.assertEqual(self.gui.reset_standup_stage, 'confirming')
        self.gui.mode_text = 'Drive'  # interruption resets the timer
        self.gui.last_mode = 101.
        self.advance('confirming', now=101.)
        self.assertIsNone(self.gui.reset_standup_confirm_start)
        self.gui.mode_text = 'Balance'
        self.gui.last_mode = 102.
        self.advance('confirming', now=102.)
        self.gui.last_mode = 104.1
        self.advance('confirming', now=104.1)
        self.assertEqual(self.gui.reset_standup_stage, 'done')

    def test_confirming_times_out_if_never_upright(self):
        self.gui.mode_text = 'EmergencyStop'
        self.gui.reset_standup_confirm_deadline = 100.
        self.advance('confirming', now=100.1)
        self.assertEqual(self.gui.reset_standup_stage, 'failed')
        self.assertIn('EmergencyStop', self.gui.reset_standup_error)

    def test_failed_stage_surfaces_error_and_resets_to_none(self):
        self.gui.reset_standup_error = 'boom'
        self.advance('failed', now=100.)
        self.assertIsNone(self.gui.reset_standup_stage)
        self.gui.reset_standup_status.set.assert_called_once_with(
            '[SIM-ONLY] reset upright FAILED: boom')


class Value:
    def __init__(self, value):
        self.value = value

    def get(self):
        return self.value

    def set(self, value):
        self.value = value


@unittest.skipUnless(os.environ.get('DISPLAY'), 'Tk needs an X display')
class DashboardWidgetTest(unittest.TestCase):
    def test_jump_is_visible_but_locked_by_default_and_observes_ownership(self):
        import tkinter as tk
        root = tk.Tk()
        root.withdraw()
        self.addCleanup(root.destroy)
        curves = json.loads((Path(__file__).resolve().parents[2] / 'tap_teleop'
                             / 'config/pose_curves.json').read_text())
        node = Mock()
        g = Dashboard(root, node, curves)
        self.assertEqual(g.jump_button.cget('text'), 'Jump once')
        self.assertEqual(str(g.jump_button.cget('state')), 'disabled')
        self.assertFalse(g.jump_opt_in.get())
        topics = [call.args[1] for call in node.create_subscription.call_args_list]
        self.assertIn('/jump_status', topics)
        g.on_jump_status(String(data='Flight|phase=Flight|active=1|ready=0|enabled=1'))
        self.assertTrue(g.jump_locked)
        self.assertTrue(g.pose_commands_blocked)
        g.refresh_controls(100.)
        for scale in g.joint_scales:
            self.assertEqual(str(scale.cget('state')), 'disabled')

    def test_wheel_leg_style_tabs_and_new_widgets_are_built(self):
        import tkinter as tk
        root = tk.Tk()
        root.withdraw()
        self.addCleanup(root.destroy)
        curves = json.loads((Path(__file__).resolve().parents[2] / 'tap_teleop'
                             / 'config/pose_curves.json').read_text())
        g = Dashboard(root, Mock(), curves)
        tabs = [g.nb.tab(t, 'text') for t in g.nb.tabs()]
        self.assertEqual(tabs, ['Control', 'Pose', 'Telemetry', 'Log'])
        # New Control-tab widgets.
        self.assertEqual(g.mode_pill.cget('text'), ' offline ')
        # New Log-tab widgets.
        self.assertFalse(g.recording.get())
        self.assertEqual(str(g.logbox.cget('state')), 'disabled')
        # New Telemetry-tab widgets.
        self.assertEqual(set(g.jtree.get_children()), set(ALL_JOINTS))
        self.assertEqual(g.pitch_history, [])
        self.assertEqual(set(g.joint_history), set(ALL_JOINTS))

    def test_shove_is_a_noop_and_logs_when_no_sim_is_running(self):
        import tkinter as tk
        root = tk.Tk()
        root.withdraw()
        self.addCleanup(root.destroy)
        curves = json.loads((Path(__file__).resolve().parents[2] / 'tap_teleop'
                             / 'config/pose_curves.json').read_text())
        g = Dashboard(root, Mock(), curves)
        self.assertIsNone(g.sim_process)
        with patch('gui.dashboard.threading.Thread') as thread:
            g._shove(1., 0.)
            thread.assert_not_called()
        self.assertIn('shove ignored', g.logbox.get('1.0', 'end'))

    def test_toggle_record_writes_header_then_rows_and_closes(self):
        import tempfile
        import tkinter as tk
        root = tk.Tk()
        root.withdraw()
        self.addCleanup(root.destroy)
        curves = json.loads((Path(__file__).resolve().parents[2] / 'tap_teleop'
                             / 'config/pose_curves.json').read_text())
        g = Dashboard(root, Mock(), curves)
        with tempfile.NamedTemporaryFile(suffix='.csv', delete=False) as f:
            path = f.name
        self.addCleanup(os.unlink, path)
        g.csv_var.set(path)
        g.toggle_record()
        self.assertTrue(g.recording.get())
        g.on_mode(String(data='Balance|fresh=1|estop=0|pitch=.05|pitch_rate=.01|'
                          'cmd_v=0|cmd_w=0|wheel_vel=0|contact=1|pose_z=.2'))
        g.toggle_record()
        self.assertFalse(g.recording.get())
        with open(path) as f:
            rows = f.read().splitlines()
        self.assertEqual(rows[0].split(',')[:3], ['wall_time', 'mode', 'pitch'])
        self.assertEqual(len(rows), 2)
        self.assertIn('Balance', rows[1])


class DashboardShowTest(unittest.TestCase):
    def setUp(self):
        self.clock_patch = patch('gui.dashboard.time.monotonic', return_value=100.)
        self.wall = self.clock_patch.start()
        self.addCleanup(self.clock_patch.stop)
        g = self.gui = Dashboard.__new__(Dashboard)
        g.refresh_controls = Mock()
        g.updating_controls = False
        package = Path(__file__).resolve().parents[2] / 'tap_teleop'
        g.curves = json.loads((package / 'config/pose_curves.json').read_text())
        g.pose = list(g.curves['rest_pose'])
        g.jvars = [Value(q) for q in g.pose]
        g.workflow = Value('Native joints')
        g.depth = Value(0.)
        g.experimental = Value(True)
        g.speed, g.yaw = Value(.1), Value(.3)
        g.enabled, g.estopped = True, False
        g.jump_locked = False
        g.experimental_jump = False
        g.pose_commands_blocked = False
        g.drive = (0., 0.)
        g.show = None
        g.show_text = 'Show idle'
        g.sim_time, g.clock_wall = 10., 100.
        g.last_mode = 100.
        g.station_ready = True
        g.measured_pose = list(g.pose)
        g.measured_wall = 100.
        g.show_tracking_since = None
        g.deep_trajectory = None
        g.deep_returning = False
        g.deep_text = ''
        g.deep_mode = 'Deep squat'
        g.deep_curves = None
        g.stand_curves = None
        g.recording = Value(False)
        g.csv_writer = None
        g.csv_file = None
        g.pitch_history = []
        g.joint_history = {name: [] for name in ALL_JOINTS}
        g.reset_standup_stage = None
        g.sim_process = None
        g.pub = {name: Mock() for name in
                 ('cmd_vel', 'enable', 'emergency_stop', 'reset_controller')}

    def set_sim_time(self, seconds):
        msg = Clock()
        msg.clock.sec = int(seconds)
        msg.clock.nanosec = round((seconds - int(seconds)) * 1e9)
        self.gui.on_clock(msg)

    def advance_clock(self, seconds):
        while seconds - self.gui.sim_time > .1:
            self.set_sim_time(self.gui.sim_time + .1)
        self.set_sim_time(seconds)

    def test_start_gates(self):
        for field, value in [('enabled', False), ('estopped', True),
                             ('station_ready', False), ('last_mode', 98.),
                             ('clock_wall', 98.), ('sim_time', None), ('drive', (.1, 0.))]:
            with self.subTest(field=field):
                old = getattr(self.gui, field)
                setattr(self.gui, field, value)
                self.gui.start_show()
                self.assertIsNone(self.gui.show)
                setattr(self.gui, field, old)
        self.gui.experimental.set(False)
        self.gui.start_show()
        self.assertIsNone(self.gui.show)
        self.gui.experimental.set(True)
        self.gui.start_show()
        self.assertEqual(self.gui.show.amplitude, .25)

    def test_station_correcting_modes_and_realistic_sag(self):
        for mode in ('Balance', 'Drive', 'PoseChange'):
            self.gui.on_mode(String(data=mode + '|station=1|fresh=1|estop=0|cmd_v=0|cmd_w=0'))
            self.gui.on_joint_states(JointState(name=list(JOINTS), position=[.115, -.041, .115, -.041]))
            self.gui.start_show()
            self.assertIsNotNone(self.gui.show, mode)
            self.gui.on_mode(String(data='Drive|station=1|fresh=1|estop=0|cmd_v=0|cmd_w=0'))
            self.assertIsNotNone(self.gui.show)
            self.gui.stop_show()

    def test_external_drive_invalid_or_missing_commands_block_and_abort(self):
        base = 'Drive|station=1|fresh=1|estop=0'
        for suffix in ('|cmd_v=.1|cmd_w=0', '|cmd_v=0|cmd_w=.1',
                       '|cmd_v=nan|cmd_w=0', '|cmd_v=0|cmd_w=inf',
                       '|cmd_v=invalid|cmd_w=0', '|cmd_v=0', ''):
            self.gui.on_mode(String(data=base + '|cmd_v=0|cmd_w=0'))
            self.gui.start_show()
            self.assertIsNotNone(self.gui.show)
            self.gui.on_mode(String(data=base + suffix))
            self.assertIsNone(self.gui.show)
            self.assertTrue(self.gui.enabled)
            self.gui.start_show()
            self.assertIsNone(self.gui.show)

    def test_neutral_gate_requires_exactly_four_targets_and_measurements(self):
        for field in ('pose', 'measured_pose'):
            original = getattr(self.gui, field)
            for length in (0, 3, 5):
                setattr(self.gui, field, (original + [0.])[:length])
                self.gui.start_show()
                self.assertIsNone(self.gui.show)
            setattr(self.gui, field, original)

    def test_neutral_target_and_measured_gate(self):
        self.gui.jvars[1].set(-.1)
        self.gui.start_show()
        self.assertIsNone(self.gui.show)
        self.gui.jvars[1].set(0.)
        self.gui.pose[1] = -.1
        self.gui.start_show()
        self.assertIsNone(self.gui.show)
        self.gui.pose[1] = 0.
        self.gui.on_joint_states(JointState(name=list(JOINTS), position=[.115, -.1, .115, 0.]))
        self.gui.start_show()
        self.assertIsNone(self.gui.show)
        self.gui.on_joint_states(JointState(name=list(JOINTS), position=self.gui.pose))
        self.gui.measured_wall = 98.
        self.gui.start_show()
        self.assertIsNone(self.gui.show)
        self.gui.measured_wall = 100.
        self.gui.start_show()
        self.assertIsNotNone(self.gui.show)

    def test_sim_clock_progress_and_exact_finite_finish(self):
        self.gui.start_show()
        first = self.gui.show_target(100.)
        self.wall.return_value = 100.5
        self.assertEqual(first, self.gui.show_target(100.5))
        self.advance_clock(15.)
        pose = self.gui.show_target(100.5)
        self.assertAlmostEqual(pose[1], -.2)
        self.advance_clock(42.)
        self.gui.measured_pose = list(self.gui.pose)
        self.assertEqual(self.gui.show_target(100.5), self.gui.curves['rest_pose'])
        self.assertIsNone(self.gui.show)
        self.assertEqual(self.gui.show_text, 'Show finished')
        self.assertIsNone(self.gui.show_target(100.5))

    def test_stop_holds_without_disabling_or_resuming(self):
        self.gui.start_show()
        self.advance_clock(15.)
        held = self.gui.show_target(100.)
        self.gui.stop_show()
        self.assertTrue(self.gui.enabled)
        self.gui.pub['enable'].publish.assert_not_called()
        self.assertEqual(self.gui.target(.05), held)
        self.set_sim_time(20.)
        self.assertIsNone(self.gui.show_target(100.))
        self.assertEqual(self.gui.target(.05), held)

    def test_clock_stale_or_backward_aborts(self):
        self.gui.start_show()
        self.gui.last_mode = 102.
        self.gui.measured_wall = 102.
        self.gui.show_target(102.)
        self.assertIsNone(self.gui.show)
        self.assertTrue(self.gui.enabled)
        self.gui.last_mode = 100.
        self.gui.start_show()
        self.assertIsNotNone(self.gui.show)
        self.set_sim_time(9.)
        self.assertIsNone(self.gui.show)
        self.set_sim_time(10.)
        self.assertIsNone(self.gui.show)

    def test_manual_actions_abort(self):
        for action in ('start_drive', 'change_workflow', 'neutral', 'disable', 'estop', 'reset'):
            with self.subTest(action=action):
                self.gui.enabled, self.gui.estopped = True, False
                self.gui.drive = (0., 0.)
                self.gui.start_show()
                self.assertIsNotNone(self.gui.show)
                if action == 'start_drive':
                    self.gui.start_drive(1, 0)
                else:
                    getattr(self.gui, action)()
                self.assertIsNone(self.gui.show)

    def test_continuous_clock_then_forward_jump_holds_and_does_not_resume(self):
        self.gui.start_show()
        for _ in range(50):
            self.set_sim_time(self.gui.sim_time + .1)
            self.gui.measured_pose = list(self.gui.pose)
            self.gui.show_target(100.)
            self.assertIsNotNone(self.gui.show)
        held = list(self.gui.pose)
        self.assertAlmostEqual(held[1], -.2)
        self.set_sim_time(self.gui.sim_time + 5.)
        self.assertIsNone(self.gui.show)
        self.assertTrue(self.gui.enabled)
        self.assertEqual(self.gui.pose, held)
        self.assertEqual(self.gui.target(.05), held)
        self.gui.pub['enable'].publish.assert_not_called()
        self.set_sim_time(self.gui.sim_time + .1)
        self.assertIsNone(self.gui.show_target(100.))
        self.gui.start_show()
        self.assertIsNone(self.gui.show, 'New start must still require neutral')
        self.gui.neutral()
        self.gui.pose = list(self.gui.curves['rest_pose'])
        self.gui.measured_pose = list(self.gui.pose)
        self.gui.start_show()
        self.assertIsNotNone(self.gui.show)
        self.assertEqual(self.gui.show_epoch, self.gui.sim_time)

    def test_forward_clock_guard_only_affects_active_show(self):
        self.set_sim_time(20.)
        self.assertTrue(self.gui.enabled)
        self.gui.pub['enable'].publish.assert_not_called()
        self.gui.start_show()
        self.set_sim_time(20.3)
        self.assertIsNotNone(self.gui.show)
        self.set_sim_time(20.601)
        self.assertIsNone(self.gui.show)

    def test_show_tracking_error_requires_sustained_sim_time(self):
        self.gui.start_show()
        self.advance_clock(15.)
        held = self.gui.show_target(100.)
        self.assertAlmostEqual(held[1], -.2)
        self.gui.show_target(100.)
        self.assertEqual(self.gui.show_tracking_since, 15.)
        self.set_sim_time(15.2)
        self.gui.show_target(100.)
        self.assertIsNotNone(self.gui.show)
        self.gui.measured_pose = list(self.gui.pose)
        self.gui.show_target(100.)
        self.assertIsNone(self.gui.show_tracking_since)
        self.gui.measured_pose = list(self.gui.curves['rest_pose'])
        self.gui.show_target(100.)
        self.set_sim_time(15.5)
        held = list(self.gui.pose)
        self.assertEqual(self.gui.show_target(100.), held)
        self.assertIsNone(self.gui.show)
        self.assertTrue(self.gui.enabled)
        self.gui.pub['enable'].publish.assert_not_called()

    def test_malformed_or_stale_joints_cancel_only_show(self):
        bad_messages = [JointState(name=list(JOINTS), position=[float('nan')] * 4),
                        JointState(name=list(JOINTS), position=[float('inf')] * 4),
                        JointState(name=list(JOINTS)[:3], position=[0.] * 3),
                        JointState(name=list(JOINTS), position=[0.] * 3)]
        for msg in bad_messages:
            self.gui.measured_pose = list(self.gui.pose)
            self.gui.start_show()
            self.assertIsNotNone(self.gui.show)
            held = list(self.gui.pose)
            self.gui.on_joint_states(msg)
            self.assertIsNone(self.gui.show)
            self.assertEqual(self.gui.pose, held)
            self.assertTrue(self.gui.enabled)
            self.gui.pub['enable'].publish.assert_not_called()
        self.gui.measured_pose = list(self.gui.pose)
        self.gui.start_show()
        self.gui.measured_wall = 98.
        self.gui.show_target(100.)
        self.assertIsNone(self.gui.show)
        self.assertTrue(self.gui.enabled)

    def test_fault_station_loss_and_feedback_timeout_abort(self):
        for feedback in ('Fallen|estop=0|fresh=1|station=1',
                         'Balance|estop=1|fresh=1|station=1',
                         'Balance|estop=0|fresh=0|station=1',
                         'Balance|estop=0|fresh=1|station=0'):
            self.gui.enabled, self.gui.estopped = True, False
            self.gui.station_ready = True
            self.gui.start_show()
            self.gui.on_mode(String(data=feedback))
            self.assertIsNone(self.gui.show)
        self.gui.enabled, self.gui.estopped = True, False
        self.gui.station_ready = True
        self.gui.start_show()
        self.gui.clock_wall = 102.
        self.gui.show_target(102.)
        self.assertIsNone(self.gui.show)


class DashboardSquatDepthTest(unittest.TestCase):
    """Plain 'Squat' mode reaches the Deep squat table's full depth (not the
    shallow -0.8 rad 'squat' table) once deep_curves is available."""

    def setUp(self):
        DashboardShowTest.setUp(self)
        self.gui.deep_curves = {'table': [list(self.gui.pose), [.979, -1.6, .979, -1.6]]}
        self.gui.workflow.set('Squat')

    def _settle(self, dt=.05, ticks=250):
        pose = None
        for _ in range(ticks):
            pose = self.gui.target(dt)
        return pose

    def test_squat_mode_samples_deep_table_when_available(self):
        self.gui.depth.set(1.)
        target = self._settle()
        self.assertAlmostEqual(target[1], -1.6, places=3)

    def test_squat_mode_falls_back_to_shallow_table_without_deep_curves(self):
        self.gui.deep_curves = None
        self.gui.depth.set(1.)
        target = self._settle()
        self.assertGreater(target[1], -1.0)  # the plain squat table only reaches -0.8

    def test_negative_slider_value_still_squats_forward_not_a_separate_curve(self):
        self.gui.depth.set(-1.)
        target = self._settle()
        self.assertAlmostEqual(target[1], -1.6, places=3)


class DashboardDeepSquatTest(unittest.TestCase):
    def setUp(self):
        DashboardShowTest.setUp(self)
        self.raw_target = self.gui.target
        self.gui.target = self.sim_target
        self.follow_measurement = True
        self.gui.deep_curves = {'table': [list(self.gui.pose), [.75, -1.2, .75, -1.2]]}
        self.gui.controller_pose_bounds = None
        self.gui.controller_fresh = False
        self.gui.deep_geometry_ready = False
        self.gui.deep_text = ''
        self.gui.deep_trajectory = None
        self.gui.operator_zero = False
        self.gui.workflow.set('Deep squat')
        self.gui.depth.set(1.)

    def sim_target(self, dt):
        self.gui.sim_time += dt
        if self.follow_measurement and self.gui.deep_trajectory is not None:
            self.gui.measured_pose = list(self.gui.pose)
        return self.raw_target(dt)

    def profile(self, extra='|pose_hip_max=.8|pose_knee_min=-1.2|pose_hip_min=-.6|pose_knee_max=.2'):
        self.gui.on_mode(String(data='Balance|station=1|fresh=1|estop=0|cmd_v=0|cmd_w=0|pitch=.036|pose_z=.125' + extra))

    def test_explicit_profile_gate_and_existing_slew(self):
        held = list(self.gui.pose)
        self.assertEqual(self.gui.target(.05), held)
        self.profile()
        self.gui.start_deep()
        moved = self.gui.target(.05)
        self.assertLess(moved[1], 0.)
        self.assertLessEqual(max(abs(a-b) for a, b in zip(moved, held)), .015 + 1e-12)
        self.gui.experimental.set(False)
        self.assertEqual(self.gui.target(.05), moved)

    def test_deep_runtime_pitch_height_gate_holds_curve_owner(self):
        self.profile()
        self.gui.start_deep()
        self.assertIsNotNone(self.gui.deep_trajectory)
        self.assertTrue(self.gui.deep_geometry_ready)
        held = self.gui.target(.05)
        for extra in ('|pitch=.121', '|pitch=-.121', '|pose_z=.1149',
                      '|pitch=nan', '|pose_z=inf', '|pitch=invalid', '|station=0'):
            self.profile('|pose_hip_max=.8|pose_knee_min=-1.2|pose_hip_min=-.6|pose_knee_max=.2' + extra)
            self.assertTrue(self.gui.deep_paused)
            self.assertIsNotNone(self.gui.deep_trajectory)
            self.assertEqual(self.gui.target(.05), held)
            self.assertTrue(self.gui.enabled)
            self.gui.pub['enable'].publish.assert_not_called()
        self.profile()
        self.assertEqual(self.gui.target(.05), held, 'Fresh recovery must not auto-resume')
        self.gui.neutral()
        self.assertFalse(self.gui.deep_paused)

    def test_deep_runtime_geometry_thresholds_and_missing_fields(self):
        for extra in ('|pitch=.12|pose_z=.115', '|pitch=-.12|pose_z=.115'):
            self.profile('|pose_hip_max=.8|pose_knee_min=-1.2|pose_hip_min=-.6|pose_knee_max=.2' + extra)
            self.assertTrue(self.gui.deep_ready(100.))
        self.gui.on_mode(String(data='Balance|station=1|fresh=1|estop=0|cmd_v=0|cmd_w=0|pose_hip_max=.8|pose_knee_min=-1.2'))
        self.assertFalse(self.gui.deep_ready(100.))

    def test_mismatch_missing_invalid_and_stale_profile_hold(self):
        for profile in ('', '|pose_hip_max=.6|pose_knee_min=-1.2',
                        '|pose_hip_max=.8|pose_knee_min=-.8',
                        '|pose_hip_max=nan|pose_knee_min=-1.2',
                        '|pose_hip_max=.8|pose_knee_min=inf'):
            self.profile(profile)
            held = list(self.gui.pose)
            self.assertEqual(self.gui.target(.05), held)
        self.profile()
        self.gui.last_mode = 98.
        self.assertFalse(self.gui.deep_ready(100.))
        self.profile()
        self.gui.controller_fresh = False
        self.assertFalse(self.gui.deep_ready(100.))

    def test_no_optional_profile_and_manual_show_unchanged(self):
        self.gui.deep_curves = None
        self.profile()
        self.assertFalse(self.gui.deep_ready(100.))
        self.gui.workflow.set('Native joints')
        self.gui.depth.set(0.)
        self.gui.start_show()
        self.assertEqual(self.gui.show.amplitude, .25)
        self.gui.workflow.set('Deep squat')
        self.gui.change_workflow()
        self.assertIsNone(self.gui.show)
        self.assertEqual(self.gui.workflow.get(), 'Deep squat')
        self.assertEqual(self.gui.curves['hip_bounds'], [-.6, .6])
        self.assertEqual(self.gui.curves['knee_bounds'], [-.8, .2])

    def test_deep_neutral_entry_and_stale_sensors_hold_without_resume(self):
        self.profile()
        self.gui.measured_pose[1] = -.1
        self.gui.start_deep()
        self.assertIsNone(self.gui.deep_trajectory)
        self.gui.measured_pose = list(self.gui.pose)
        self.gui.start_deep()
        self.assertIsNotNone(self.gui.deep_trajectory)
        held = self.gui.target(.05)
        self.gui.measured_wall = 98.
        self.assertEqual(self.gui.target(.05), held)
        self.assertTrue(self.gui.deep_paused)
        self.gui.measured_wall = 100.
        self.assertEqual(self.gui.target(.05), held)

    def test_deep_manual_drive_aborts_and_holds_last_command(self):
        self.profile()
        self.gui.start_deep()
        held = self.gui.target(.05)
        self.gui.start_drive(1, 0)
        self.assertTrue(self.gui.deep_paused)
        self.assertEqual(self.gui.drive, (0., 0.))
        self.assertEqual(self.gui.target(.05), held)
        self.assertTrue(self.gui.enabled)

    def test_hold_squat_action_preserves_owner_and_requires_neutral(self):
        self.enter_full_depth()
        held = list(self.gui.pose)
        self.gui.stop_deep()  # Callback bound to the optional Hold squat button.
        self.assertIsNotNone(self.gui.deep_trajectory)
        self.assertTrue(self.gui.deep_paused)
        self.assertEqual(self.gui.target(.1), held)
        self.profile()
        self.assertEqual(self.gui.target(.1), held)
        self.gui.pub['enable'].publish.assert_not_called()
        self.gui.neutral()
        self.assertFalse(self.gui.deep_paused)
        self.assertGreater(self.gui.target(.1)[1], held[1])

    def test_deep_tracking_error_sim_dwell_and_no_auto_resume(self):
        self.enter_full_depth()
        self.follow_measurement = False
        self.gui.measured_pose = list(self.gui.curves['rest_pose'])
        held = self.gui.target(.05)
        self.assertIsNotNone(self.gui.deep_tracking_since)
        self.gui.target(.2)
        self.assertFalse(self.gui.deep_paused)
        self.gui.measured_pose = list(self.gui.pose)
        self.gui.target(.05)
        self.assertIsNone(self.gui.deep_tracking_since)
        self.gui.measured_pose = list(self.gui.curves['rest_pose'])
        self.gui.target(.05)
        self.gui.target(.3)
        self.assertTrue(self.gui.deep_paused)
        self.assertEqual(self.gui.pose, held)
        self.assertTrue(self.gui.enabled)
        self.gui.measured_pose = list(held)
        self.profile()
        self.assertEqual(self.gui.target(.05), held)
        self.gui.pub['enable'].publish.assert_not_called()

    def enter_full_depth(self):
        self.profile()
        self.gui.start_deep()
        for _ in range(230):
            self.gui.target(.05)
        self.assertAlmostEqual(self.gui.pose[1], -1.2)
        self.assertGreater(self.gui.pose[0], .6)

    def test_deep_exit_owns_curve_until_neutral_without_scale_clipping(self):
        for destination in ('Native joints', 'Roll'):
            with self.subTest(destination=destination):
                self.gui.workflow.set('Deep squat')
                self.gui.depth.set(1.)
                self.enter_full_depth()
                held = list(self.gui.pose)
                original_sliders = [v.get() for v in self.gui.jvars]
                # Simulate Tk scales that cannot accept the deep endpoint.
                for i, var in enumerate(self.gui.jvars):
                    lower, upper = (-.6, .6) if i % 2 == 0 else (-.8, .2)
                    original_set = var.set
                    def checked_set(value, lo=lower, hi=upper, setter=original_set):
                        self.assertTrue(lo <= value <= hi, 'Deep pose copied into manual scale')
                        setter(value)
                    var.set = checked_set
                if destination == 'Native joints':
                    self.gui.neutral()
                else:
                    self.gui.workflow.set(destination)
                    self.gui.change_workflow()
                self.assertEqual([v.get() for v in self.gui.jvars], original_sliders)
                self.assertIsNotNone(self.gui.deep_trajectory)
                self.assertEqual(self.gui.workflow.get(), 'Deep squat')
                previous = held
                while self.gui.deep_trajectory is not None:
                    pose = self.gui.target(.05)
                    self.assertLessEqual(max(abs(q-p) for q,p in zip(pose,previous)), .015+1e-10)
                    if self.gui.deep_trajectory is not None:
                        amplitude = self.gui.deep_trajectory.amplitude
                        first, last = self.gui.deep_curves['table']
                        expected = [a+(b-a)*amplitude for a,b in zip(first,last)]
                        for q,p in zip(pose,expected):
                            self.assertAlmostEqual(q,p)
                    previous = pose
                self.assertEqual(self.gui.pose, self.gui.curves['rest_pose'])
                self.assertEqual(self.gui.workflow.get(), destination)

    def test_deep_stop_fault_clock_jumps_hold_owner_and_require_explicit_return(self):
        self.enter_full_depth()
        held = list(self.gui.pose)
        self.gui.start_drive(1, 0)
        self.assertEqual(self.gui.drive, (0., 0.))
        self.assertEqual(self.gui.target(.05), held)
        self.profile()
        self.assertEqual(self.gui.target(.05), held)
        self.gui.neutral()
        self.assertFalse(self.gui.deep_paused)
        msg = Clock()
        msg.clock.sec = 15
        self.gui.on_clock(msg)
        self.assertTrue(self.gui.deep_paused)
        self.assertEqual(self.gui.target(.05), held)
        self.gui.neutral()
        msg.clock.sec = 14
        self.gui.on_clock(msg)
        self.assertTrue(self.gui.deep_paused)
        self.gui.neutral()
        self.gui.on_mode(String(data='EmergencyStop|estop=0|fresh=1'))
        self.assertIsNotNone(self.gui.deep_trajectory)
        self.assertTrue(self.gui.deep_paused)
        self.gui.enabled = True
        self.profile()
        self.assertEqual(self.gui.target(.05), held)
        self.gui.neutral()
        self.assertLess(abs(self.gui.target(.05)[1]), abs(held[1]))

    def test_cli_defaults_cap_and_ros_arguments(self):
        options, remaining = parse_options([])
        self.assertEqual(options.deep_squat_knee, 1.6)  # Deep squat is on by default (2026-09-16)
        self.assertFalse(options.no_deep_squat)
        options, remaining = parse_options(['--deep-squat-knee', '1.6', '--ros-args'])
        self.assertEqual(options.deep_squat_knee, 1.6)
        self.assertEqual(remaining, ['--ros-args'])
        for value in ('0', '-1', '1.6001', '1.8', '2.1', 'nan', 'inf'):
            with patch('sys.stderr'), self.assertRaises(SystemExit):
                parse_options(['--deep-squat-knee', value])
        options, _ = parse_options(['--no-deep-squat'])
        self.assertTrue(options.no_deep_squat)

    def test_controller_caps_cover_actual_table_not_unused_generator_bounds(self):
        self.gui.deep_curves['hip_bounds'] = [-1.57, 1.57]
        self.profile('|pose_hip_max=.75|pose_knee_min=-1.2|pose_hip_min=-.6|pose_knee_max=.2')
        self.assertTrue(self.gui.deep_ready(100.))
        self.profile('|pose_hip_max=.749|pose_knee_min=-1.2|pose_hip_min=-.6|pose_knee_max=.2')
        self.assertFalse(self.gui.deep_ready(100.))

    def test_approved_full_depth_requires_eleven_seconds(self):
        self.gui.deep_curves['table'][-1] = [.979, -1.6, .979, -1.6]
        self.profile('|pose_hip_max=1.05|pose_knee_min=-1.6|pose_hip_min=-.6|pose_knee_max=.2')
        self.gui.start_deep()
        self.gui.deep_trajectory.advance(1., 0.)
        self.assertGreaterEqual(self.gui.deep_trajectory.duration_s, 11.)
        options, _ = parse_options(['--deep-squat-knee', '1.6'])
        self.assertEqual(options.deep_squat_knee, 1.6)

    def test_deep_uses_simulation_delta_not_wall_delta(self):
        self.profile()
        self.gui.start_deep()
        initial = list(self.gui.pose)
        self.assertEqual(self.raw_target(.1), initial)
        self.gui.sim_time += .1
        moved = self.raw_target(0.)
        self.assertLess(moved[1], initial[1])
        self.assertEqual(self.raw_target(.1), moved)


class DashboardStandupTest(unittest.TestCase):
    """The extension-direction sibling of DashboardDeepSquatTest, sharing the
    same trajectory/hold state machine via self.gui.deep_mode."""

    def setUp(self):
        DashboardShowTest.setUp(self)
        self.raw_target = self.gui.target
        self.gui.target = self.sim_target
        self.follow_measurement = True
        self.gui.stand_curves = {'table': [list(self.gui.pose), [-.19, .5, -.19, .5]]}
        self.gui.controller_pose_bounds = None
        self.gui.controller_fresh = False
        self.gui.deep_geometry_ready = False
        self.gui.deep_text = ''
        self.gui.deep_trajectory = None
        self.gui.operator_zero = True
        self.gui.workflow.set('Standup')
        self.gui.deep_mode = 'Standup'
        self.gui.depth.set(1.)

    def sim_target(self, dt):
        self.gui.sim_time += dt
        if self.follow_measurement and self.gui.deep_trajectory is not None:
            self.gui.measured_pose = list(self.gui.pose)
        return self.raw_target(dt)

    def profile(self, extra='|pose_hip_max=.8|pose_knee_min=-1.2|pose_hip_min=-.6|pose_knee_max=.5'):
        self.gui.on_mode(String(data='Balance|station=1|fresh=1|estop=0|cmd_v=0|cmd_w=0|pitch=.036|pose_z=.125' + extra))

    def test_go_stand_tall_selects_standup_profile_and_holds_at_full_extension(self):
        self.gui.deep_curves = {'table': [list(self.gui.pose), [.75, -1.2, .75, -1.2]]}
        self.profile()
        self.assertTrue(self.gui.deep_ready(100.))
        self.gui.go_stand_tall()
        self.assertEqual(self.gui.deep_mode, 'Standup')
        self.assertEqual(self.gui.workflow.get(), 'Standup')
        self.assertIsNotNone(self.gui.deep_trajectory)
        moved = self.gui.target(.05)
        self.assertGreater(moved[1], 0.)  # knee moves toward the tallest extension, not a crouch
        # Reaching full amplitude does not by itself trigger any return.
        for _ in range(400):
            moved = self.gui.target(.1)
        self.assertAlmostEqual(moved[1], .5, places=6)
        held = self.gui.target(.1)
        self.assertEqual(held, moved)

    def test_narrower_knee_max_bound_blocks_standup_but_not_squat(self):
        self.gui.deep_curves = {'table': [list(self.gui.pose), [.75, -1.2, .75, -1.2]]}
        self.profile('|pose_hip_max=.8|pose_knee_min=-1.2|pose_hip_min=-.6|pose_knee_max=.2')
        self.gui.deep_mode = 'Standup'
        self.assertFalse(self.gui.deep_ready(100.), 'stand_curves reaches knee=.5, past the .2 cap')
        self.gui.deep_mode = 'Deep squat'
        self.assertTrue(self.gui.deep_ready(100.))

    def test_missing_profile_blocks_ready(self):
        self.gui.stand_curves = None
        self.profile()
        self.assertFalse(self.gui.deep_ready(100.))


class DashboardJumpTest(unittest.TestCase):
    def test_scale_state_refresh_is_not_an_operator_motion(self):
        g = self.gui
        g.stop_show, g.stop_deep = Mock(), Mock()
        g.updating_controls = True
        g.manual_joint_change()
        g.depth_change()
        g.stop_show.assert_not_called()
        g.stop_deep.assert_not_called()
        g.updating_controls = False
        g.manual_joint_change()
        g.stop_show.assert_called_once()
        g.stop_deep.assert_called_once()

    def test_visible_controls_lock_manual_joints_in_deep_mode(self):
        g = self.gui
        g.joint_scales = [Mock() for _ in range(4)]
        g.depth_label, g.depth_scale, g.jump_button = Mock(), Mock(), Mock()
        g.jump_status_var = Mock()
        g.workflow.set('Deep squat')
        Dashboard.refresh_controls(g, 100.)
        for scale in g.joint_scales:
            scale.configure.assert_called_with(state='disabled')
        g.depth_scale.configure.assert_called_with(from_=0, state='normal')
        g.experimental_jump = False
        Dashboard.refresh_controls(g, 100.)
        g.jump_button.configure.assert_called_with(state='disabled')
        self.assertIn('opt-in off', g.jump_status_var.set.call_args.args[0])
        g.workflow.set('Native joints')
        Dashboard.refresh_controls(g, 100.)
        for scale in g.joint_scales:
            scale.configure.assert_called_with(state='normal')

    def test_squat_slider_is_one_sided_but_roll_keeps_both_directions(self):
        g = self.gui
        g.joint_scales = [Mock() for _ in range(4)]
        g.depth_label, g.depth_scale, g.jump_button = Mock(), Mock(), Mock()
        g.jump_status_var = Mock()
        g.workflow.set('Squat')
        Dashboard.refresh_controls(g, 100.)
        g.depth_scale.configure.assert_called_with(from_=0, state='normal')
        g.workflow.set('Roll')
        Dashboard.refresh_controls(g, 100.)
        g.depth_scale.configure.assert_called_with(from_=-1, state='normal')

    def test_jump_status_wrap_uses_remaining_width(self):
        button, label = Mock(), Mock()
        button.winfo_reqwidth.return_value = 90
        for frame_width, expected in ((756, 654), (626, 524), (100, 1)):
            Dashboard.resize_jump_status(frame_width, button, label)
            label.configure.assert_called_with(wraplength=expected)
        button.configure.assert_not_called()

    def setUp(self):
        DashboardShowTest.setUp(self)
        g = self.gui
        g.experimental_jump = True
        g.jump_locked = False
        g.jump_fault = False
        g.jump_seen_active = False
        g.jump_reset_pending = False
        g.jump_ready_status = False
        g.jump_last_status = 0.
        g.jump_text = ''
        g.jump_reason = ''
        g.jump_pose_pending = False
        g.jump_status_sequence = 0
        g.jump_request_sequence = 0
        g.controller_fresh = True
        g.operator_zero = True
        g.pub['jump_request'] = Mock()
        g.pub['stand_up'] = Mock()
        g.pub['target_pose'] = Mock()
        g.on_jump_status(String(data='Idle|phase=Idle|ready=1|active=0|enabled=1|reason=idle'))

    def test_opt_in_neutral_fresh_ready_and_single_edge(self):
        self.assertFalse(parse_options([])[0].experimental_jump)
        self.assertTrue(parse_options(['--experimental-jump'])[0].experimental_jump)
        for name, value in [('experimental_jump', False), ('enabled', False),
                            ('jump_last_status', 98.), ('controller_fresh', False),
                            ('operator_zero', False), ('drive', (.1, 0.)),
                            ('measured_pose', [0., -.2, 0., -.2])]:
            original = getattr(self.gui, name)
            setattr(self.gui, name, value)
            self.gui.jump_once()
            self.gui.pub['jump_request'].publish.assert_not_called()
            setattr(self.gui, name, original)
        self.gui.jump_once()
        self.assertEqual([c.args[0].data for c in self.gui.pub['jump_request'].publish.call_args_list], [False, True])
        self.gui.jump_once()
        self.assertEqual(self.gui.pub['jump_request'].publish.call_count, 2)
        self.gui.on_jump_status(String(data='Idle|ready=1|active=0|enabled=1'))
        self.assertTrue(self.gui.jump_locked, 'Pre-acceptance ready must not release ownership')

    def test_controller_ownership_suppresses_commands_and_timer_watchdog(self):
        self.gui.jump_once()
        for publisher in self.gui.pub.values(): publisher.reset_mock()
        self.gui.on_jump_status(String(data='flight'))
        for action in (self.gui.enable, self.gui.stand_up, self.gui.start_show,
                       self.gui.start_deep, self.gui.neutral, self.gui.change_workflow,
                       self.gui.stop_drive, self.gui.reset):
            action()
        self.gui.start_drive(1, 0)
        g = self.gui
        g.node, g.root = Mock(), Mock()
        g.status, g.motors, g.telemetry = Mock(), Mock(), Mock()
        g.show_status, g.jump_status_var = Mock(), Mock()
        g.mode_text, g.motor_text, g.telemetry_text = 'Drive', '', ''
        g.last_tick, g.last_mode, g.feedback_grace_until = 0., 0., 0.
        with patch('gui.dashboard.rclpy.ok', return_value=True), patch('gui.dashboard.rclpy.spin_once'):
            g.tick()
        for publisher in g.pub.values(): publisher.publish.assert_not_called()
        self.assertTrue(g.jump_locked)
        g.disable()
        self.assertFalse(g.pub['enable'].publish.call_args.args[0].data)
        g.estop()
        self.assertTrue(g.pub['emergency_stop'].publish.call_args.args[0].data)

    def test_fault_requires_explicit_reset_and_fresh_ready(self):
        self.gui.jump_once()
        self.gui.on_jump_status(String(data='fault'))
        self.gui.on_jump_status(String(data='Idle|ready=1|active=0|enabled=1'))
        self.assertTrue(self.gui.jump_fault)
        self.gui.reset()
        self.assertTrue(self.gui.pub['reset_controller'].publish.call_args.args[0].data)
        self.assertTrue(self.gui.jump_locked)
        self.gui.on_jump_status(String(data='Idle|ready=1|active=0|enabled=1'))
        self.assertFalse(self.gui.jump_locked)
        self.assertFalse(self.gui.jump_fault)
        self.assertFalse(self.gui.enabled)

    def test_complete_allows_explicit_second_request_without_automatic_repeat(self):
        self.gui.jump_once()
        self.gui.on_jump_status(String(data='thrust'))
        self.gui.on_jump_status(String(data='Complete|phase=Complete|ready=1|active=0|enabled=1'))
        self.assertFalse(self.gui.jump_locked)
        self.assertTrue(self.gui.enabled)
        self.assertEqual(self.gui.pub['jump_request'].publish.call_count, 2)
        self.gui.jump_once()
        self.assertEqual(self.gui.pub['jump_request'].publish.call_count, 4)

    def test_idle_not_ready_startup_does_not_lock(self):
        self.gui.on_jump_status(String(data='Idle|phase=Idle|ready=0|active=0|enabled=1|reason=idle'))
        self.assertFalse(self.gui.jump_locked)
        self.assertFalse(self.gui.jump_seen_active)
        self.gui.jump_once()
        self.gui.pub['jump_request'].publish.assert_not_called()
        self.gui.on_jump_status(String(data='Idle|phase=Idle|ready=1|active=0|enabled=1|reason=idle'))
        self.gui.jump_once()
        self.assertTrue(self.gui.jump_locked)

    def test_explicit_rejection_ack_releases_pending_without_resend(self):
        self.gui.jump_once()
        self.gui.on_jump_status(String(data='Idle|phase=Idle|ready=0|active=0|enabled=1|reason=idle'))
        self.assertTrue(self.gui.jump_locked)
        self.gui.controller_fresh = False
        rejected = String(data='Idle|phase=Idle|ready=0|active=0|enabled=1|reason=not_ready_grounded_neutral')
        self.gui.on_jump_status(rejected)
        self.assertTrue(self.gui.jump_locked)
        self.gui.controller_fresh = True
        self.gui.on_jump_status(rejected)
        self.assertFalse(self.gui.jump_locked)
        self.assertTrue(self.gui.enabled)
        self.assertFalse(self.gui.jump_ready_status)
        self.assertEqual(self.gui.pub['jump_request'].publish.call_count, 2)

    def test_known_active_phases_and_malformed_status_remain_locked(self):
        for phase in ('Crouch', 'Thrust', 'Flight', 'Land', 'Settle'):
            self.gui.on_jump_status(String(data=f'{phase}|phase={phase}|active=1|ready=0|enabled=1'))
            self.assertTrue(self.gui.jump_locked)
            self.assertTrue(self.gui.jump_seen_active)
        self.gui.on_jump_status(String(data='Idle|ready=0'))
        self.assertTrue(self.gui.jump_locked)

    def quiet_tick(self):
        g = self.gui
        g.node, g.root = Mock(), Mock()
        g.status, g.motors, g.telemetry = Mock(), Mock(), Mock()
        g.show_status, g.jump_status_var = Mock(), Mock()
        g.mode_text, g.motor_text, g.telemetry_text = 'Balance', '', ''
        g.last_tick, g.last_mode, g.feedback_grace_until = 99.95, 100., 0.
        with patch('gui.dashboard.rclpy.ok', return_value=True), patch('gui.dashboard.rclpy.spin_once'):
            g.tick()

    def test_complete_does_not_replay_midflight_slider_or_workflow_edits(self):
        self.gui.jump_once()
        self.gui.on_jump_status(String(data='Flight|phase=Flight|active=1|ready=0|enabled=1'))
        self.gui.workflow.set('Squat')
        self.gui.depth.set(1.)
        self.gui.jvars[1].set(-.8)
        self.gui.on_jump_status(String(data='Complete|phase=Complete|active=0|ready=1|enabled=1'))
        self.assertTrue(self.gui.enabled)
        self.assertTrue(self.gui.pose_commands_blocked)
        self.quiet_tick()
        self.gui.pub['target_pose'].publish.assert_not_called()
        self.gui.start_show()
        self.assertIsNone(self.gui.show)
        self.gui.neutral()
        self.assertFalse(self.gui.pose_commands_blocked)
        self.assertEqual(self.gui.workflow.get(), 'Native joints')
        self.assertEqual(self.gui.jvars[1].get(), 0.)

    def test_reset_sag_requires_explicit_neutral_before_pose_publication(self):
        self.gui.workflow.set('Squat')
        self.gui.depth.set(1.)
        self.gui.pose[1] = -.7
        self.gui.reset()
        self.gui.measured_pose = [.115, -.041, .115, -.041]
        self.gui.enable()
        self.quiet_tick()
        self.gui.pub['target_pose'].publish.assert_not_called()
        self.assertTrue(self.gui.pose_commands_blocked)
        self.gui.neutral()
        self.assertEqual(self.gui.pose, self.gui.measured_pose)
        self.quiet_tick()
        command = self.gui.pub['target_pose'].publish.call_args.args[0].data
        self.assertGreater(command[1], -.041)
        self.assertLessEqual(command[1], -.026+1e-10)

    def test_reset_invalid_or_out_of_manual_bounds_never_clips_measured_pose(self):
        self.gui.reset()
        for pose in ([.98, -1.6, .98, -1.6], [float('nan')] * 4, [0.] * 3):
            self.gui.measured_pose = pose
            self.gui.neutral()
            self.assertTrue(self.gui.pose_commands_blocked)
        self.gui.measured_pose = list(self.gui.curves['rest_pose'])
        self.gui.measured_wall = 98.
        self.gui.neutral()
        self.assertTrue(self.gui.pose_commands_blocked)

    def test_jump_status_reason_and_pending_pose_are_visible(self):
        self.gui.on_jump_status(String(data='Idle|phase=Idle|active=0|ready=0|enabled=0|reason=disabled|pose_pending=0'))
        self.assertIn('disabled', self.gui.jump_text)
        self.gui.jump_once()
        self.assertIn('disabled', self.gui.jump_text)
        self.gui.on_jump_status(String(data='Idle|phase=Idle|active=0|ready=1|enabled=1|reason=idle|pose_pending=1'))
        self.assertFalse(self.gui.jump_ready_status)
        self.gui.jump_once()
        self.assertIn('pose pending', self.gui.jump_text)
        self.gui.pub['jump_request'].publish.assert_not_called()

    def test_disabled_pending_jump_can_reset_without_replaying_pose(self):
        self.gui.jump_once()
        self.gui.disable()
        self.gui.on_jump_status(String(data='Idle|phase=Idle|active=0|ready=0|enabled=1|reason=not_ready_grounded_neutral'))
        self.gui.reset()
        self.assertTrue(self.gui.jump_reset_pending)
        self.gui.pub['reset_controller'].publish.assert_called_once()
        self.gui.on_jump_status(String(data='Idle|phase=Idle|active=0|ready=0|enabled=1|reason=idle'))
        self.assertFalse(self.gui.jump_locked)
        self.assertFalse(self.gui.enabled)
        self.assertTrue(self.gui.pose_commands_blocked)


if __name__ == '__main__':
    unittest.main()
