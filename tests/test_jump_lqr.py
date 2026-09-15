"""Offline TVLQR math checks. No simulation or landing-success claims."""

import copy
import importlib.util
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
AVAILABLE = all(importlib.util.find_spec(name) for name in ('casadi', 'scipy', 'adam'))


def load_tool(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / 'tools' / (name + '.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@unittest.skipUnless(AVAILABLE, 'Requires the dedicated ADAM/CasADi/SciPy offline environment')
class JumpLqrTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        xacro = shutil.which('xacro')
        if xacro is None:
            raise unittest.SkipTest('Current CAD fixture requires xacro in PATH')
        xml = subprocess.check_output(
            [xacro, str(ROOT / 'src/tap_robot_description/description/cad_robot/robot.urdf.xacro'),
             'simulation:=true', 'jump_model:=true'], text=True)
        temporary = tempfile.TemporaryDirectory(prefix='tap-jump-lqr-')
        cls.addClassCleanup(temporary.cleanup)
        path = Path(temporary.name) / 'expanded.urdf'
        path.write_text(xml)
        cls.lqr = load_tool('jump_lqr')
        cls.dynamics = load_tool('jump_dynamics').build_dynamics(path)
        cls.models = {mode: cls.lqr.build_linearization(cls.dynamics, mode)
                      for mode in (True, False)}

    def sample(self, grounded):
        q = np.array([.01, .24, -.12, .6, -1.1, .4])
        v = np.array([.1, -.2, .3, -.4, .8, -1.2])
        if grounded:
            J = np.asarray(self.dynamics(q, v)[5])
            v[:2] = np.linalg.solve(J[:, :2], -J[:, 2:] @ v[2:])
        return q, v, np.array([.3, 1.2, -.04])

    def test_jacobians_match_central_differences(self):
        eps = 1e-6
        for grounded, model in self.models.items():
            with self.subTest(grounded=grounded):
                q, v, tau = self.sample(grounded)
                point = np.concatenate((q, v, tau))
                _, A, B = model(q, v, tau)

                def evaluate(z):
                    return np.asarray(model(z[:6], z[6:12], z[12:])[0]).ravel()

                numerical = np.column_stack([
                    (evaluate(point + eps*d) - evaluate(point - eps*d)) / (2*eps)
                    for d in np.eye(15)])
                np.testing.assert_allclose(np.hstack((np.asarray(A), np.asarray(B))),
                                           numerical, rtol=2e-5, atol=2e-5)

    def test_grounded_acceleration_and_virtual_work(self):
        from scipy.linalg import null_space
        q, v, tau = self.sample(True)
        M, bias, _, _, _, J = (np.asarray(x) for x in self.dynamics(q, v))
        acceleration = np.asarray(self.models[True](q, v, tau)[0]).ravel()[6:]
        eps = 1e-6
        jdot = (np.asarray(self.dynamics(q + eps*v, v)[5])
                - np.asarray(self.dynamics(q - eps*v, v)[5])) / (2*eps)
        np.testing.assert_allclose(J @ acceleration + jdot @ v, 0., atol=2e-9)
        passive = np.r_[np.zeros(3), .04*v[3:5], .002*v[5]]
        residual = M @ acceleration + bias.ravel() + passive - np.r_[np.zeros(3), 2*tau]
        np.testing.assert_allclose(null_space(J).T @ residual, 0., atol=2e-9)

    def test_flight_input_and_damping_match_net_torque_contract(self):
        q, v, tau = self.sample(False)
        M, bias, *_ = (np.asarray(x) for x in self.dynamics(q, v))
        xdot, _, B = (np.asarray(x) for x in self.models[False](q, v, tau))
        actuation = np.vstack((np.zeros((3, 3)), 2*np.eye(3)))
        passive = np.r_[np.zeros(3), .04*v[3:5], .002*v[5]]
        np.testing.assert_allclose(xdot[:6, 0], v)
        np.testing.assert_allclose(M @ xdot[6:, 0] + bias.ravel(), actuation @ tau - passive,
                                   atol=2e-10)
        np.testing.assert_allclose(B[:6], 0., atol=1e-12)
        np.testing.assert_allclose(M @ B[6:], actuation, atol=2e-10)
        self.assertGreater(v @ passive, 0.)

    def test_zoh_double_integrator(self):
        Ad, Bd = self.lqr._discretize(np.array([[0., 1.], [0., 0.]]),
                                      np.array([[0.], [1.]]), .2)
        np.testing.assert_allclose(Ad, [[1., .2], [0., 1.]], atol=1e-14)
        np.testing.assert_allclose(Bd, [[.02], [.2]], atol=1e-14)

    def test_effective_local_feedback_matches_finite_differences(self):
        expected = np.zeros((3, 12))
        expected[0, 9] = expected[1, 10] = .6
        np.testing.assert_array_equal(self.lqr.LOCAL_FEEDBACK, expected)
        eps = 1e-6
        for grounded, model in self.models.items():
            with self.subTest(grounded=grounded):
                q, v, tau = self.sample(grounded)
                reference = np.r_[q, v]
                _, A, B = (np.asarray(x) for x in model(q, v, tau))

                def evaluate(error):
                    state = reference + error
                    net = tau - expected @ error
                    return np.asarray(model(state[:6], state[6:], net)[0]).ravel()

                numerical = np.column_stack([
                    (evaluate(eps*d) - evaluate(-eps*d)) / (2*eps) for d in np.eye(12)])
                np.testing.assert_allclose(A - B @ expected, numerical, rtol=2e-5, atol=2e-5)

    def plan(self):
        q, v, tau = self.sample(True)
        return {'phase_intervals': [2, 3, 2], 'phase_durations_s': [.1, .2, .3],
                'coordinate_order': ['body_x', 'body_z', 'pitch', 'hip', 'knee', 'wheel'],
                'q': np.tile(q, (8, 1)).tolist(), 'v': np.tile(v, (8, 1)).tolist(),
                'torque_per_side_nm': np.tile(tau, (7, 1)).tolist()}

    def test_finite_gains_and_backward_recursion(self):
        plan = self.plan()
        original = copy.deepcopy(plan)
        gains = self.lqr.compute_gains(plan, self.dynamics)
        self.assertEqual(gains.shape, (8, 3, 12))
        self.assertTrue(np.isfinite(gains).all())
        self.assertGreater(np.linalg.norm(gains), 0.)
        self.assertEqual(plan, original)
        np.testing.assert_array_equal(gains[-1], gains[-2])
        # Independently use the subtractive Riccati formula to check phase
        # indexing, duration scaling, per-side input convention and minus sign.
        Qground, _ = self.lqr._cost_matrices(True)
        P = .5 * Qground
        q, v, tau = self.sample(True)
        modes = [True, True, False, False, False, True, True]
        dts = [.05, .05, .2/3, .2/3, .2/3, .15, .15]
        for i in reversed(range(7)):
            _, A, B = self.models[modes[i]](q, v, tau)
            A, B = np.asarray(A), np.asarray(B)
            Ad, Bd = self.lqr._discretize(A - B @ self.lqr.LOCAL_FEEDBACK, B, dts[i])
            Q, R = self.lqr._cost_matrices(modes[i])
            S = dts[i]*R + Bd.T @ P @ Bd
            L = Bd.T @ P @ Ad
            np.testing.assert_allclose(S @ gains[i], L, rtol=2e-9, atol=2e-9)
            P = dts[i]*Q + Ad.T @ P @ Ad - L.T @ np.linalg.solve(S, L)
            P = (P + P.T) / 2

    def test_pose_priority_changes_only_leg_position_cost(self):
        base, r = self.lqr._cost_matrices(True)
        tuned, rt = self.lqr._cost_matrices(True, 25.)
        expected = base.copy()
        expected[3, 3] *= 25.; expected[4, 4] *= 25.
        np.testing.assert_allclose(tuned, expected)
        np.testing.assert_allclose(r, rt)
        for value in (0., 101., float('nan')):
            with self.assertRaises(ValueError):
                self.lqr.compute_gains({}, None, pose_priority=value)

    def test_invalid_plans_rejected_before_linearization(self):
        for key, value in [('phase_intervals', [2, 2.5, 2]),
                           ('phase_durations_s', [.1, 0., .3]),
                           ('q', [[0.]*6]), ('torque_per_side_nm', [[0.]*6]*7),
                           ('v', [[float('nan')]*6]*8),
                           ('coordinate_order', ['hip', 'knee', 'wheel']*2)]:
            with self.subTest(key=key):
                plan = self.plan()
                plan[key] = value
                with self.assertRaises(ValueError):
                    self.lqr.compute_gains(plan, None)


if __name__ == '__main__':
    unittest.main()
