"""Exporter validation without ADAM, CasADi, pytest, or archived plans."""

import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest import mock

import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('export_jump_plan', ROOT / 'tools/export_jump_plan.py')
EXPORTER = importlib.util.module_from_spec(SPEC)
MODEL_MODULE = types.ModuleType('jump_dynamics')
MODEL_MODULE.build_dynamics = mock.Mock()
MODEL_MODULE.model_metadata = mock.Mock()
LQR_MODULE = types.ModuleType('jump_lqr')
LQR_MODULE.compute_gains = mock.Mock()
with mock.patch.dict(sys.modules, {'jump_dynamics': MODEL_MODULE, 'jump_lqr': LQR_MODULE}):
    SPEC.loader.exec_module(EXPORTER)


class ExportJumpPlanTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix='tap-export-jump-test-')
        self.addCleanup(temporary.cleanup)
        root = Path(temporary.name)
        self.urdf = root / 'model.urdf'
        self.urdf.write_text('<robot name="mock"/>')
        self.source = root / 'plan.json'
        self.output = root / 'profile.yaml'
        q = np.zeros((4, 6))
        q[:, 1] = .24
        q[:, 3] = [.4, .3, .2, .2]
        q[:, 4] = [-1.2, -.9, -.6, -.6]
        self.plan = {
            'solver_success': True, 'status': 'Solve_Succeeded',
            'max_constraint_violation': 0.,
            'urdf_sha256': hashlib.sha256(self.urdf.read_bytes()).hexdigest(),
            'phase_intervals': [1, 1, 1], 'phase_durations_s': [.1, .2, .3],
            'q': q.tolist(), 'v': np.zeros((4, 6)).tolist(),
            'torque_per_side_nm': [[.1, .2, .03]] * 3,
        }
        J = np.array([[1., 0., 0., 0., 0., -.03], [0., 1., 0., 0., 0., 0.]])
        self.model = mock.Mock(return_value=(
            np.eye(6), np.array([0., 3.938*9.81, 0., .4, .6, .02]),
            np.array([0., 0., .21]), np.zeros(3), np.array([0., 0., .03]), J))
        self.gain_values = np.arange(4*3*12, dtype=float).reshape(4, 3, 12) / 1000
        self.build = self.enterContext(mock.patch.object(EXPORTER, 'build_dynamics',
                                                        return_value=self.model))
        self.enterContext(mock.patch.object(EXPORTER, 'model_metadata',
                                            return_value={'total_mass_kg': 3.938}))
        self.gains = self.enterContext(mock.patch.object(EXPORTER, 'compute_gains',
                                                        return_value=self.gain_values))

    def reject(self, plan, message, before_model=True):
        self.source.write_text(json.dumps(plan))
        self.output.write_text('existing output\n')
        with self.assertRaisesRegex(ValueError, message):
            EXPORTER.export(self.source, self.urdf, self.output)
        self.assertEqual(self.output.read_text(), 'existing output\n')
        if before_model:
            self.build.assert_not_called()
            self.gains.assert_not_called()

    def test_reject_failed_solver(self):
        self.plan['solver_success'] = False
        self.reject(self.plan, 'converged')

    def test_precision_preparation_is_opt_in(self):
        self.source.write_text(json.dumps(self.plan))
        EXPORTER.export(self.source, self.urdf, self.output)
        original = yaml.safe_load(self.output.read_text())['balance_controller_node']['ros__parameters']
        EXPORTER.export(self.source, self.urdf, self.output, precision_preparation=True)
        precision = yaml.safe_load(self.output.read_text())['balance_controller_node']['ros__parameters']
        self.assertEqual(original['jump.crouch_duration_s'], 3.)
        self.assertEqual(precision['jump.crouch_duration_s'], 11.)
        self.assertEqual(precision['jump.plan_preparation_wait_s'], 8.)
        self.assertEqual(precision['station_keeper.position_gain_hz'], 3.)
        self.assertEqual(precision['jump.plan_samples'], original['jump.plan_samples'])
        self.assertEqual(precision['jump.leg_torque_limit_nm'], 5.)

    def test_reject_non_success_status(self):
        for status in ('Infeasible_Problem_Detected', 'Solved_To_Acceptable_Level', None):
            with self.subTest(status=status):
                self.plan['status'] = status
                self.reject(self.plan, 'converged')

    def test_reject_nonfinite_residual(self):
        for value in (float('nan'), float('inf'), -float('inf')):
            with self.subTest(value=value):
                self.plan['max_constraint_violation'] = value
                self.reject(self.plan, 'converged')

    def test_reject_excessive_residual(self):
        self.plan['max_constraint_violation'] = 1.0001e-5
        self.reject(self.plan, 'converged')

    def test_reject_hash_mismatch(self):
        self.urdf.write_text('<robot name="changed"/>')
        self.reject(self.plan, 'URDF differs')

    def test_reject_invalid_dimensions(self):
        cases = [('q', np.zeros((3, 6))), ('q', np.zeros((4, 5))),
                 ('v', np.zeros((5, 6))), ('v', np.zeros((4, 7))),
                 ('torque_per_side_nm', np.zeros((4, 3))),
                 ('torque_per_side_nm', np.zeros((3, 6)))]
        for key, value in cases:
            with self.subTest(key=key, shape=value.shape):
                plan = copy.deepcopy(self.plan)
                plan[key] = value.tolist()
                self.reject(plan, 'trajectory dimensions')

    def test_reject_capacity_overflow_with_consistent_shapes(self):
        self.plan.update(phase_intervals=[85, 85, 86],
                         q=np.zeros((257, 6)).tolist(), v=np.zeros((257, 6)).tolist(),
                         torque_per_side_nm=np.zeros((256, 3)).tolist())
        self.reject(self.plan, 'trajectory dimensions')

    def test_reject_nonfinite_trajectory(self):
        for key in ('q', 'v', 'torque_per_side_nm'):
            for value in (float('nan'), float('inf'), -float('inf')):
                with self.subTest(key=key, value=value):
                    plan = copy.deepcopy(self.plan)
                    plan[key][0][0] = value
                    self.reject(plan, 'Nonfinite trajectory')

    def test_reject_nonfinite_timestamps(self):
        for value in (float('nan'), float('inf'), -float('inf')):
            with self.subTest(value=value):
                plan = copy.deepcopy(self.plan)
                plan['phase_durations_s'][1] = value
                self.reject(plan, 'Nonfinite trajectory')

    def test_reject_invalid_gains(self):
        for values in (np.zeros((3, 3, 12)), np.zeros((4, 12, 3)),
                       np.full((4, 3, 12), np.nan), np.full((4, 3, 12), np.inf)):
            with self.subTest(shape=values.shape, finite=np.isfinite(values).all()):
                self.gains.return_value = values
                self.reject(self.plan, 'Invalid offline feedback gains', before_model=False)

    def test_export_valid_mock_plan_at_residual_threshold(self):
        self.plan['max_constraint_violation'] = 1e-5
        self.source.write_text(json.dumps(self.plan))
        with mock.patch('builtins.print'):
            EXPORTER.export(self.source, self.urdf, self.output)
        text = self.output.read_text()
        params = yaml.safe_load(text)['balance_controller_node']['ros__parameters']
        rows = np.asarray(params['jump.plan_samples']).reshape(4, 52)
        np.testing.assert_allclose(rows[:, 0], [0., .1, .3, .6])
        np.testing.assert_allclose(rows[:, 16:], self.gain_values.reshape(4, 36))
        np.testing.assert_allclose(rows[-1, [7, 8, 10]], [.2, .3, .01])
        self.assertEqual(params['jump.plan_durations_s'], [.1, .2, .3])
        self.assertEqual(params['jump.profile'], 'manual_curve_experimental')
        self.assertEqual(params['jump.plan_touchdown_com_z_m'], .21)
        self.assertIn('NOT validated', text)
        self.assertIn(self.plan['urdf_sha256'], text)
        self.build.assert_called_once_with(self.urdf)
        self.gains.assert_called_once_with(self.plan, self.model, pose_priority=1.)


if __name__ == '__main__':
    unittest.main()
