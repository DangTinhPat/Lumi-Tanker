"""Wheel reference data must not silently become calibrated actuator physics."""
import hashlib
import math
from pathlib import Path
import unittest

import yaml

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / 'src/tap_robot_description/description/cad_robot'


class WheelMotorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.data = yaml.safe_load((MODEL / 'wheel_motor.yaml').read_text())

    def test_selection_and_source(self):
        self.assertEqual(self.data['model'], 'BL4820M12')
        self.assertEqual(self.data['joints'], ['cad_left_wheel', 'cad_right_wheel'])
        source = self.data['source']
        self.assertEqual(hashlib.sha256((ROOT / source['snapshot']).read_bytes()).hexdigest(),
                         source['sha256'])

    def test_units_and_derived_speed(self):
        p, d = self.data['published'], self.data['derived']
        self.assertAlmostEqual(p['torque_nm'], 130 / 1000)
        omega = p['rated_speed_rpm'] * 2 * math.pi / 60
        self.assertAlmostEqual(d['rated_speed_rad_s'], omega)
        self.assertAlmostEqual(d['direct_drive_ground_speed_at_rated_rpm_m_s'], omega * .030)
        self.assertEqual(p['torque_rating_type'], 'unspecified')

    def test_unknowns_do_not_replace_mass_or_actuator(self):
        self.assertTrue(all(value is None for value in self.data['unconfirmed'].values()))
        self.assertIsNone(self.data['published']['no_load_speed_unit'])
        self.assertIsNone(self.data['published']['dimension_unit'])
        masses = yaml.safe_load((MODEL / 'masses.yaml').read_text())
        self.assertEqual(masses['wheel'], .135)
        self.assertAlmostEqual(masses['body'] + 2 * sum(masses[k] for k in ('thigh', 'shin', 'wheel')),
                               3.938)
        for filename in ('robot.urdf.xacro', 'simulation.urdf.xacro'):
            self.assertNotIn('wheel_motor.yaml', (MODEL / filename).read_text())


if __name__ == '__main__':
    unittest.main()
