"""Focused CAD interference regressions; passing is not an envelope certificate."""
import importlib.util
from pathlib import Path
import unittest

import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('squat_audit', ROOT / 'tools/audit_squat_geometry.py')
audit_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit_module)


class SquatGeometryAuditTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.audit = audit_module.SquatAudit(
            ROOT / 'src/tap_robot_description/description/cad_robot/robot.urdf.xacro')

    def test_winding_distinguishes_inside_outside_and_surface(self):
        box = trimesh.creation.box(extents=[2, 2, 2])
        points = np.array([[0., 0., 0.], [3., 0., 0.], [1., 0., 0.]])
        self.assertEqual(audit_module.vertices_inside(box, points), 1)

    def test_broader_root_continuation_and_floor_clearance(self):
        previous = 0.0
        for knee in (0, -.8, -1.2, -1.6, -1.9, -2, -2.1):
            roots = self.audit.neutral_roots(knee)
            self.assertTrue(roots)
            hip = min(roots, key=lambda h: abs(h-previous))
            self.assertGreaterEqual(hip, previous)
            state = self.audit.evaluate(hip, knee, check_mesh=False)
            self.assertLess(abs(state['neutral_residual_m']), 1e-8)
            self.assertGreater(min(state['box_floor_clearances_m'].values()), 0)
            previous = hip
        self.assertGreater(len(roots), 1)
        self.assertAlmostEqual(previous, 1.284655, places=5)
        self.assertAlmostEqual(state['body_height_m'], .080652, places=5)

    def test_deep_wheel_thigh_interference_both_sides(self):
        for knee, expected in ((-1.2, False), (-1.9, True), (-2.1, True)):
            hip = max(self.audit.neutral_roots(knee))
            state = self.audit.evaluate(hip, knee)
            for side, samples in state['wheel_thigh_samples'].items():
                with self.subTest(knee=knee, side=side):
                    self.assertEqual(samples['interference_confirmed'], expected)
                    if not expected:
                        self.assertGreater(samples['sampled_vertex_surface_gap_m'], .04)


if __name__ == '__main__':
    unittest.main()
