"""Offline runner contracts; no ROS imports, domain or simulator required."""
from pathlib import Path
import ast
import runpy
import unittest


HARNESS = runpy.run_path(str(Path(__file__).resolve().parents[1] /
                             'tools/transformation_smoke.py'))


class TransformationSmokeTest(unittest.TestCase):
    def test_harness_ros_init_does_not_consume_child_launch_arguments(self):
        root = Path(__file__).resolve().parents[1]
        for name in ('transformation_smoke.py', 'jump_smoke.py'):
            tree = ast.parse((root/'tools'/name).read_text())
            calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
                     and isinstance(n.func, ast.Attribute) and n.func.attr == 'init'
                     and isinstance(n.func.value, ast.Name) and n.func.value.id == 'rclpy']
            self.assertTrue(calls)
            for call in calls:
                args = next((k.value for k in call.keywords if k.arg == 'args'), None)
                self.assertIsInstance(args, ast.List)
                self.assertEqual(args.elts, [])

    def test_bounded_estop_requires_fresh_six_joint_applied_zero(self):
        check = HARNESS['estop_observation_zero']
        commands = {'legs': {'wall': 10.1, 'data': [0.]*4},
                    'wheels': {'wall': 10.1, 'data': [0.]*2}}
        self.assertTrue(check(commands, 10., 10.15))
        self.assertFalse(check(commands, 10., 10.15, True, 1., 1.15))
        for efforts in (None, [0.]*5, [0.]*5+[.001], [0.]*5+[float('nan')]):
            cache = dict(commands, joints={'wall': 10.1, 'stamp': 1.1, 'data': {'efforts': efforts}})
            self.assertFalse(check(cache, 10., 10.15, True, 1., 1.15))
        cache = dict(commands, joints={'wall': 10.1, 'stamp': 1.1, 'data': {'efforts': [0.]*6}})
        self.assertTrue(check(cache, 10., 10.15, True, 1., 1.15))

    def test_bounded_estop_rejects_stale_or_pre_stop_efforts(self):
        check = HARNESS['estop_observation_zero']
        for wall, stamp in ((9.99, 1.1), (10.1, .99), (10.1, 1.),
                            (10.1, 1.001), (10.1, None), (10.1, 1.3)):
            cache = {'legs': {'wall': 10.2, 'data': [0.]*4},
                     'wheels': {'wall': 10.2, 'data': [0.]*2},
                     'joints': {'wall': wall, 'stamp': stamp, 'data': {'efforts': [0.]*6}}}
            self.assertFalse(check(cache, 10., 10.25, True, 1., 1.2))

    def test_geometry_storage_deduplicates_pair_not_clock_ticks(self):
        append = HARNESS['append_geometry_sample']
        self.assertTrue(append(1., None, (1., 1.), None))
        self.assertFalse(append(1.001, 1., (1., 1.), (1., 1.)))
        self.assertTrue(append(1.004, 1., (1.004, 1.), (1., 1.)))
        self.assertTrue(append(1.005, 1.004, (1.004, 1.005), (1.004, 1.)))
        self.assertFalse(append(1., 1., (1.001, 1.), (1., 1.)))

    def test_launch_model_arguments_match_expansion(self):
        arguments = ['jump_model:=true', 'jump_leg_speed:=12.566370614359172',
                     'jump_leg_velocity_damping:=.6']
        self.assertEqual(HARNESS['model_launch_arguments'](arguments+['spawn_z:=.2372']), arguments)

    def test_launch_reserved_duplicate_and_malformed_arguments_rejected(self):
        for arguments in (['gui:=true'], ['auto_enable:=false'], ['controller_config:=x'],
                          ['jump_model:=true', 'jump_model:=false'], ['invalid'], ['name:=']):
            with self.assertRaises(ValueError):
                HARNESS['model_launch_arguments'](arguments)

    def test_deep_request_cannot_change_roll_or_exceed_authorized_depth(self):
        validate = HARNESS['validate_deep_request']
        validate(None, 'all', None)
        validate(1., 'squat', None)
        validate(1.2, 'squat', 1.)
        validate(1.6, 'squat', 1.)
        for depth, mode, amount in ((2.1, 'squat', None), (1., 'all', None),
                                    (0., 'squat', None), (float('nan'), 'squat', None),
                                    (1., 'squat', .5)):
            with self.assertRaises(ValueError):
                validate(depth, mode, amount)

    def test_absolute_pitch_guard_independent_and_optional(self):
        check = HARNESS['check_absolute_pitch']
        check(.5, None)
        check(.12, .12)
        check(-.12, .12)
        for pitch in (.12001, -.12001, float('nan')):
            with self.assertRaisesRegex(RuntimeError, 'Absolute pitch'):
                check(pitch, .12)

    def test_default_keeps_all_three_motions(self):
        self.assertEqual(HARNESS['motion_curves'](),
                         ('squat', 'roll_left', 'roll_right'))

    def test_squat_only_never_selects_roll(self):
        sequence = [curve for _ in range(2)
                    for curve in HARNESS['motion_curves']('squat')
                    for returning in (False, True)]
        self.assertEqual(sequence, ['squat'] * 4)
        with self.assertRaises(KeyError):
            HARNESS['motion_curves']('unknown')

    def test_quintic_endpoints_and_monotonicity(self):
        blend = HARNESS['smooth5']
        self.assertEqual([blend(t) for t in (-1, 0, .5, 1, 2)], [0, 0, .5, 1, 1])
        self.assertTrue(all(blend(i/100) <= blend((i+1)/100) for i in range(100)))
        self.assertLess(blend(1e-4)/1e-4, 1e-6)
        self.assertLess((1-blend(1-1e-4))/1e-4, 1e-6)

    def test_20hz_scheduler_never_bursts(self):
        due = HARNESS['command_due']
        last, sent = None, []
        for i in range(1001):
            t = i / 1000
            if due(t, last):
                sent.append(t)
                last = t
        self.assertEqual(len(sent), 21)
        self.assertTrue(all(b-a >= .05-1e-9 for a, b in zip(sent, sent[1:])))
        self.assertTrue(due(2, last))
        self.assertFalse(due(2, 2))
        self.assertFalse(due(None, None))

    def test_station_profile_preserves_base_limits(self):
        base = {'p': {'torque': 3., 'station': {'enabled': False, 'gain': .8}}}
        merged = HARNESS['merge_config'](base, {'p': {'station': {'enabled': True}}})
        self.assertEqual(merged['p']['torque'], 3.)
        self.assertEqual(merged['p']['station'], {'enabled': True, 'gain': .8})
        self.assertFalse(base['p']['station']['enabled'])

    def test_fixed_anchor_peak_and_return_are_separate(self):
        def row(t, x):
            return dict(sim=t, x=x, y=0., yaw_unwrapped=0., pitch_error=0., pitch=.02,
                        roll=0., z=.23, joint_error=.01, leg_commands=[1.]*4,
                        joint_efforts=[1.]*6)
        rows = [row(0, 0), row(1, .1), row(2, .002)]
        result = HARNESS['metrics'](rows, rows[0])
        self.assertAlmostEqual(result['peak_xy_m'], .1)
        self.assertAlmostEqual(result['residual_xy_m'], .002)
        self.assertAlmostEqual(result['path_length_m'], .198)


if __name__ == '__main__':
    unittest.main()
