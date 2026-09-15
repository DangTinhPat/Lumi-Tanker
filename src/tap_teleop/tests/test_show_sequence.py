"""Offline finite performance checks using the generated CAD curve tables."""
import json
from pathlib import Path
import sys
import unittest

import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
from tap_teleop.pose_curves import RobotGeometry, generate, sample
from tap_teleop.show_sequence import ShowSequence


class ShowSequenceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.curves = json.loads((PACKAGE / 'config/pose_curves.json').read_text())

    def test_endpoints_and_finite_completion(self):
        seq = ShowSequence(self.curves)
        self.assertEqual(seq.duration_s, 32.)
        for t in (0, 10, 20, 30, 32, 1000):
            result = seq.evaluate(t)
            self.assertEqual(result.targets, seq.neutral)
            self.assertEqual(result.finished, t >= 32)
        for t, mode in ((5, 'squat'), (15, 'roll_left'), (25, 'roll_right')):
            np.testing.assert_allclose(seq.evaluate(t).targets, sample(self.curves, mode, .5))

    def test_bounds_and_monotonic_stage_order(self):
        for amplitude in (0, .5, 1):
            seq = ShowSequence(self.curves, amplitude)
            indices = []
            for t in np.linspace(0, 35, 1401):
                result = seq.evaluate(t)
                indices.append(seq.stage_names.index(result.stage))
                self.assertTrue(np.isfinite(result.targets).all())
                self.assertTrue(np.all(np.asarray(result.targets) >= seq.bounds[:, 0]))
                self.assertTrue(np.all(np.asarray(result.targets) <= seq.bounds[:, 1]))
            self.assertTrue(np.all(np.diff(indices) >= 0))

    def test_zero_velocity_acceleration_at_transition_endpoints(self):
        seq = ShowSequence(self.curves, 1)
        h = 1e-4
        for t in (2, 5, 7, 10, 12, 15, 17, 20, 22, 25, 27, 30):
            before, center, after = [np.array(seq.evaluate(x).targets) for x in (t-h, t, t+h)]
            self.assertLess(np.max(np.abs((after-before)/(2*h))), 1e-6)
            self.assertLess(np.max(np.abs((after-2*center+before)/(h*h))), 1e-3)

    def test_explicit_stop_resume_no_automatic_resume(self):
        seq = ShowSequence(self.curves)
        frozen = seq.stop(3.)
        self.assertTrue(seq.stopped)
        self.assertEqual(seq.evaluate(100.), frozen)
        self.assertEqual(seq.stop(101.), frozen)
        self.assertEqual(seq.resume(103.), frozen)
        self.assertFalse(seq.stopped)
        self.assertEqual(seq.evaluate(132.).targets, seq.neutral)
        self.assertTrue(seq.evaluate(132.).finished)
        self.assertTrue(seq.stop(133.).finished)
        self.assertTrue(seq.resume(200.).finished)

    def test_time_and_amplitude_validation(self):
        for value in (-1, float('nan'), float('inf')):
            seq = ShowSequence(self.curves)
            for method in (seq.evaluate, seq.stop, seq.resume):
                with self.assertRaises(ValueError):
                    method(value)
        for value in (-.1, 1.1, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                ShowSequence(self.curves, value)
        for kwargs in ({'transition_s': 0}, {'dwell_s': 0}, {'transition_s': -1}):
            with self.assertRaises(ValueError):
                ShowSequence(self.curves, **kwargs)
        seq = ShowSequence(self.curves)
        seq.stop(5)
        with self.assertRaises(ValueError):
            seq.resume(4)

    def test_curve_amplitude_is_interpolated_not_joint_endpoints(self):
        seq = ShowSequence(self.curves, 1)
        np.testing.assert_allclose(seq.evaluate(3.5).targets,
                                   sample(self.curves, 'squat', .5), atol=1e-12)
        self.assertEqual(seq.evaluate(3.5), seq.evaluate(3.5))


class CurrentUrdfSequenceTest(unittest.TestCase):
    """Regenerate from source Xacro; no simulator or ROS node is initialized."""

    @classmethod
    def setUpClass(cls):
        xacro = PACKAGE.parent / 'tap_robot_description/description/cad_robot/robot.urdf.xacro'
        cls.robot = RobotGeometry(xacro)
        cls.curves = generate(xacro)

    def assert_geometry(self, poses):
        worst_dx, minimum_margin = 0., float('inf')
        for pose in poses:
            pose = np.asarray(pose)
            self.assertTrue(np.isfinite(pose).all())
            self.assertTrue(np.all(pose[[0, 2]] >= -.6))
            self.assertTrue(np.all(pose[[0, 2]] <= .6))
            self.assertTrue(np.all(pose[[1, 3]] >= -.8))
            self.assertTrue(np.all(pose[[1, 3]] <= .2))
            geometry = self.robot.evaluate(pose)
            worst_dx = max(worst_dx, abs(geometry['dx']))
            minimum_margin = min(minimum_margin, geometry['lateral_margin'])
        self.assertLess(worst_dx, 1e-5, 'CoM residual exceeds 0.01 mm')
        self.assertGreater(minimum_margin, .025, 'Lateral support margin below 25 mm')
        return worst_dx, minimum_margin

    def test_stored_tables_match_current_urdf(self):
        stored = json.loads((PACKAGE / 'config/pose_curves.json').read_text())
        for mode in self.curves['tables']:
            np.testing.assert_allclose(stored['tables'][mode], self.curves['tables'][mode],
                                       atol=1e-9, rtol=0,
                                       err_msg='Regenerate stored tables after geometry/mass edits')

    def test_current_urdf_interpolated_curve_geometry(self):
        for amplitude in (.5, 1.):
            for mode in self.curves['tables']:
                with self.subTest(amplitude=amplitude, mode=mode):
                    # Include non-table amplitudes to check interpolation, not
                    # only the exact FK-neutral knots used during generation.
                    poses = [sample(self.curves, mode, a)
                             for a in np.linspace(0, amplitude, 1001)]
                    dx, margin = self.assert_geometry(poses)
                    print(f'URDF {mode} amplitude={amplitude:g}: '
                          f'max_dx={dx:.9g} m, min_margin={margin:.9g} m')

    def test_current_urdf_sequence_geometry(self):
        for amplitude in (.5, 1.):
            with self.subTest(amplitude=amplitude):
                seq = ShowSequence(self.curves, amplitude)
                self.assert_geometry(seq.evaluate(t).targets
                                     for t in np.linspace(0, seq.duration_s, 641))
                self.assertEqual(seq.evaluate(seq.duration_s).targets, seq.neutral)
                self.assertTrue(seq.evaluate(seq.duration_s).finished)

    def test_measured_speed_against_point_four_slew(self):
        controller_slew = .4  # Current integration gate, rad/s per joint.
        dt = .001
        for amplitude in (.5, 1.):
            with self.subTest(amplitude=amplitude):
                seq = ShowSequence(self.curves, amplitude, transition_s=3.)
                times = np.arange(0, seq.duration_s + dt / 2, dt)
                poses = np.array([seq.evaluate(t).targets for t in times])
                peak = np.abs(np.diff(poses, axis=0) / dt).max(axis=0)
                self.assertTrue(np.isfinite(peak).all())
                # Knee is linear in amplitude: peak quintic slope = 1.875.
                knee_travel = abs(self.curves['tables']['squat'][-1][1]
                                  - self.curves['rest_pose'][1]) * amplitude
                expected_knee_peak = knee_travel * 1.875 / 3.
                np.testing.assert_allclose(peak[[1, 3]], expected_knee_peak,
                                           atol=1e-6, rtol=0)
                self.assertTrue(np.all(peak[[0, 2]] < controller_slew))
                if amplitude == .5:
                    self.assertTrue(np.all(peak < controller_slew))
                else:
                    # A passing test records the known incompatibility; it
                    # must not imply full amplitude passes the physical gate.
                    self.assertTrue(np.all(peak[[1, 3]] > controller_slew))
                print(f'URDF amplitude={amplitude:g}, transition=3s: '
                      f'peak_speed={peak.tolist()} rad/s; controller_slew={controller_slew}')


if __name__ == '__main__':
    unittest.main()
