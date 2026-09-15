"""Optional deep squat FK tests; no physical depth recommendation."""
from pathlib import Path
import sys
import unittest

import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
from tap_teleop.deep_squat import DeepSquatTrajectory, _root, generate_deep_squat
from tap_teleop.pose_curves import RobotGeometry, sample

XACRO = PACKAGE.parent / 'tap_robot_description/description/cad_robot/robot.urdf.xacro'


class DeepSquatTest(unittest.TestCase):
    def test_geometry_candidate_16_and_eleven_second_full_transition(self):
        profile = generate_deep_squat(XACRO, knee_max=1.6, hip_bounds=(-.6, 1.05), samples=21)
        self.assertFalse(profile['physical_gate_passed'])
        self.assertLess(profile['table'][-1][0], 1.05)
        trajectory = DeepSquatTrajectory(profile, minimum_transition_s=11.)
        trajectory.advance(1., 0.)
        self.assertGreaterEqual(trajectory.duration_s, 11.)
        trajectory.advance(1., 10.)
        self.assertLess(trajectory.amplitude, 1.)
        np.testing.assert_allclose(trajectory.advance(1., 1.), profile['table'][-1], atol=1e-12)
        trajectory.advance(0., 10.)
        self.assertGreater(trajectory.amplitude, 0.)
        np.testing.assert_allclose(trajectory.advance(0., 1.), profile['rest_pose'], atol=1e-12)
        for duration in (-1., float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                DeepSquatTrajectory(profile, minimum_transition_s=duration)

    def test_quintic_eleven_seconds_speed_and_retarget_position_continuity(self):
        profile = {'table': [[.115, 0., .115, 0.], [.979, -1.6, .979, -1.6]]}
        trajectory = DeepSquatTrajectory(profile)
        previous = np.array(profile['table'][0])
        for _ in range(1100):
            pose = np.array(trajectory.advance(1., .01))
            self.assertLessEqual(np.abs(pose-previous).max()/.01, .3+1e-10)
            previous = pose
        np.testing.assert_allclose(pose, profile['table'][-1], atol=1e-10)
        self.assertEqual(trajectory.amplitude, 1.)
        self.assertAlmostEqual(trajectory.duration_s, 11.)
        trajectory.advance(0., 3.)
        held = trajectory.advance(0., 0.)
        np.testing.assert_array_equal(trajectory.advance(.7, 0.), held)
        for _ in range(500):
            before = np.array(trajectory.advance(.7, 0.))
            after = np.array(trajectory.advance(.7, .01))
            self.assertLessEqual(np.abs(after-before).max()/.01, .3+1e-10)

    def test_coordinated_amplitude_slew_actual_fk_and_slider_reversals(self):
        profile = generate_deep_squat(XACRO, knee_max=1.2, hip_bounds=(-.6, .8), samples=101)
        robot = RobotGeometry(XACRO)
        trajectory = DeepSquatTrajectory(profile)
        previous = np.array(profile['rest_pose'])
        dt = .02
        for request in ([1.] * 250 + [0.] * 250 + [.5] * 125):
            pose = np.array(trajectory.advance(request, dt))
            self.assertLessEqual(np.abs(pose-previous).max()/dt, .3+1e-10)
            expected = sample({'tables': {'deep': profile['table']}}, 'deep', trajectory.amplitude)
            np.testing.assert_allclose(pose, expected, atol=1e-12)
            self.assertLess(abs(robot.evaluate(pose)['dx']), 1e-5)
            self.assertEqual(pose[0], pose[2])
            self.assertEqual(pose[1], pose[3])
            previous = pose
        for amplitude, elapsed in ((float('nan'), .1), (1.1, .1), (.5, -1), (.5, float('inf'))):
            with self.assertRaises(ValueError):
                trajectory.advance(amplitude, elapsed)

    def test_actual_geometry_bounded_symmetric_and_shortening(self):
        data = generate_deep_squat(XACRO, knee_max=.8, hip_bounds=(-.6, .6), samples=11)
        robot = RobotGeometry(XACRO)
        self.assertFalse(data['physical_gate_passed'])
        self.assertEqual(data['knee_bounds'], [-.8, 0.])
        self.assertTrue(np.all(np.diff(data['com_height_m']) < 0))
        for pose, height in zip(data['table'], data['com_height_m']):
            self.assertEqual(pose[:2], pose[2:])
            self.assertTrue(-.6 <= pose[0] <= .6)
            self.assertTrue(-.8 <= pose[1] <= 0)
            result = robot.evaluate(pose)
            self.assertLess(abs(result['dx']), 1e-9)
            self.assertAlmostEqual(result['height'], height)

    def test_invalid_requests(self):
        for value in (0, -1, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                generate_deep_squat(XACRO, knee_max=value)
        for bounds in ((0, 0), (1, -1), (-2, 2), (float('nan'), .6)):
            with self.assertRaises(ValueError):
                generate_deep_squat(XACRO, knee_max=.2, hip_bounds=bounds)
        for samples in (0, 1, 2.5, True):
            with self.assertRaises(ValueError):
                generate_deep_squat(XACRO, knee_max=.2, samples=samples)
        with self.assertRaises(ValueError):
            generate_deep_squat(XACRO, knee_max=2.2)

    def test_no_neutral_crossing_rejected(self):
        with self.assertRaisesRegex(ValueError, 'neutral crossing'):
            generate_deep_squat(XACRO, knee_max=.2, hip_bounds=(-.6, -.2), samples=3)
        for fn in (lambda x: x*x+1, lambda x: float('nan'), lambda x: x*x-.25):
            with self.assertRaises(ValueError):
                _root(fn, -1, 1)
        self.assertAlmostEqual(_root(lambda x: x-.2, -1, 1), .2, places=8)


if __name__ == '__main__':
    unittest.main()
