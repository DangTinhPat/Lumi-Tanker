"""Offline FK verification. Does not join a ROS domain or start a simulator."""
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/tap_teleop'))
from tap_teleop.pose_curves import RobotGeometry, generate, sample

XACRO = ROOT / 'src/tap_robot_description/description/cad_robot/robot.urdf.xacro'


class PoseCurvesTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = RobotGeometry(XACRO)
        cls.curves = generate(XACRO)

    def test_neutral_includes_constant_hip_offset(self):
        pose = self.curves['rest_pose']
        self.assertGreater(abs(pose[0]), .01)
        self.assertLess(abs(self.robot.evaluate(pose)['dx']), 1e-8)

    def test_negative_knee_shortens_native_cad(self):
        self.assertEqual(self.curves['shortening_sign'], -1)
        self.assertGreater(self.curves['squat_drop_m'], .01)
        heights = [self.robot.evaluate(p)['height'] for p in self.curves['tables']['squat']]
        self.assertTrue(np.all(np.diff(heights) < 0))

    def test_symmetric_same_sign(self):
        for pose in self.curves['tables']['squat']:
            np.testing.assert_allclose(pose[:2], pose[2:], atol=1e-12)

    def test_interpolated_curves_neutral_and_bounded(self):
        for mode in self.curves['tables']:
            for amount in np.linspace(0, 1, 101):
                pose = sample(self.curves, mode, amount)
                self.assertTrue(np.all(np.isfinite(pose)))
                for i, angle in enumerate(pose):
                    bounds = self.curves['hip_bounds' if i % 2 == 0 else 'knee_bounds']
                    self.assertLessEqual(bounds[0], angle)
                    self.assertGreaterEqual(bounds[1], angle)
                result = self.robot.evaluate(pose)
                self.assertLess(abs(result['dx']), 1e-5, (mode, amount, result))
                self.assertGreater(result['lateral_margin'], .025)

    def test_roll_directions_opposite(self):
        left = self.robot.evaluate(sample(self.curves, 'roll_left', 1))
        right = self.robot.evaluate(sample(self.curves, 'roll_right', 1))
        self.assertLess(left['roll'] * right['roll'], 0)


if __name__ == '__main__':
    unittest.main()
