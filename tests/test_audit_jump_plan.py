import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from audit_jump_plan import audit


class AuditTests(unittest.TestCase):
    def plan(self):
        return dict(phase_intervals=[1, 1, 1], phase_durations_s=[.1, .2, .3],
                    q=np.zeros((4, 6)), v=np.ones((4, 6)),
                    torque_per_side_nm=np.ones((3, 3)))

    @staticmethod
    def model(q, v):
        J = np.zeros((2, 6)); J[0, 0] = J[1, 1] = 1
        return np.eye(6), np.zeros(6), np.zeros(3), v[:3], np.zeros(3), J

    def test_bilateral_work_and_phase_duration(self):
        result = audit(self.plan(), self.model)
        for name, work in [('thrust', .6), ('flight', 1.2), ('landing', 1.8)]:
            self.assertAlmostEqual(result['phases'][name]['positive_motor_work_j'], work)
        self.assertFalse(result['gazebo_validated'])

    def test_impact_energy_and_unilateral_rejection(self):
        result = audit(self.plan(), self.model)['touchdown']
        self.assertAlmostEqual(result['impact_energy_loss_j'], 1.)
        self.assertFalse(result['sticking_mu1_feasible'])
        p = self.plan(); p['v'][:, :2] = [0, -2]
        result = audit(p, self.model)['touchdown']
        self.assertTrue(result['sticking_mu1_feasible'])
        self.assertAlmostEqual(result['impact_energy_loss_j'], 2.)

    def test_nonfinite_rejected(self):
        p = self.plan(); p['v'][0, 0] = np.nan
        with self.assertRaises(ValueError):
            audit(p, self.model)

    def test_contact_defect_detects_inconsistent_acceleration(self):
        p = self.plan()
        p['v'][1, 1] = 2.
        phase = audit(p, self.model)['phases']['thrust']
        self.assertAlmostEqual(phase['max_contact_acceleration_defect_m_s2'], 10.)
        self.assertGreater(phase['max_acceleration_defect_norm'], 10.)


if __name__ == '__main__':
    unittest.main()
