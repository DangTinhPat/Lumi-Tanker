"""Analytic checks for the optimistic scalar jump feasibility screen."""
import importlib.util
from pathlib import Path
import unittest

PATH = Path(__file__).resolve().parents[1] / 'tools/analyze_jump_envelope.py'
SPEC = importlib.util.spec_from_file_location('jump_envelope', PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class JumpEnvelopeTests(unittest.TestCase):
    def profile(self):
        return {'table': [[0., 0., 0., 0.], [.5, -.5, .5, -.5], [1., -1., 1., -1.]],
                'com_height_m': [.3, .2, .1]}

    def test_speed_bound_for_linear_path(self):
        result = MODULE.screen(self.profile(), 1., 10., 2.)
        self.assertAlmostEqual(result['candidate_vertical_speed_bound_m_s'], .4)
        self.assertAlmostEqual(result['optimistic_ballistic_rise_bound_m'], .4**2/(2*9.81))

    def test_no_motor_work_cannot_jump(self):
        self.assertEqual(MODULE.screen(self.profile(), 1., 0., 2.)[
            'optimistic_ballistic_rise_bound_m'], 0.)

    def test_no_joint_speed_cannot_jump(self):
        self.assertEqual(MODULE.screen(self.profile(), 1., 10., 0.)[
            'optimistic_ballistic_rise_bound_m'], 0.)


if __name__ == '__main__':
    unittest.main()
