"""Independent analytic momentum and source/frame regression checks, no ROS."""
import copy
import importlib.util
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("jump_trace", ROOT / "tools/analyze_jump_trace.py")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def link(name, mass=1., xyz="0 0 0", rpy="0 0 0", diagonal=(1., 2., 3.)):
    a, b, c = diagonal
    return f'''<link name="{name}"><inertial>
      <origin xyz="{xyz}" rpy="{rpy}"/><mass value="{mass}"/>
      <inertia ixx="{a}" iyy="{b}" izz="{c}" ixy="0" ixz="0" iyz="0"/>
      </inertial></link>'''


def joint(name, parent, child, xyz="0 0 0", kind="revolute"):
    return f'''<joint name="{name}" type="{kind}"><parent link="{parent}"/>
      <child link="{child}"/><origin xyz="{xyz}"/><axis xyz="0 1 0"/></joint>'''


def model(xml):
    return MODULE.MomentumModel(ET.fromstring(f"<robot>{xml}</robot>"))


def sample(time=0., quaternion=(0., 0., 0., 1.), positions=None, velocities=None):
    return dict(body_position=[0., 0., 0.], quaternion=list(quaternion),
                joints=dict(positions=positions or {}, velocities=velocities or {}),
                sim=1000. + time * 9., source_stamps=dict(pose=time, joints=time - .002),
                control=dict(pitch_rate=123.))


class JumpMomentumTests(unittest.TestCase):
    def assertVector(self, actual, expected, atol=1e-10):
        np.testing.assert_allclose(actual, expected, atol=atol, rtol=0)

    def test_single_body_spin_no_orbital_despite_offset(self):
        body = model(link("body", mass=2., xyz="2 -3 4"))
        result = body.evaluate(sample(), [1., 2., 3.], [4., 5., 6.])
        self.assertVector(result["com_world_m"], [2., -3., 4.])
        self.assertVector(result["spin_world_kg_m2_s"], [1., 4., 9.])
        self.assertVector(result["orbital_world_kg_m2_s"], [0., 0., 0.])

    def test_fixed_pair_spin_and_orbital(self):
        body = model(link("body", xyz="-1 0 0") + link("child")
                     + joint("fixed", "body", "child", "1 0 0", "fixed"))
        result = body.evaluate(sample(), [0., 0., 2.])
        self.assertVector(result["com_world_m"], [0., 0., 0.])
        self.assertVector(result["spin_world_kg_m2_s"], [0., 0., 12.])
        self.assertVector(result["orbital_world_kg_m2_s"], [0., 0., 4.])
        self.assertVector(result["total_world_kg_m2_s"], [0., 0., 16.])

    def test_inertial_origin_rotation_once_and_products(self):
        xml = link("body", xyz="2 3 4", rpy=f"0 {-np.pi / 2} 0")
        xml = xml.replace('ixz="0"', 'ixz="0.2"').replace('izz="3.0"', 'izz="2.5"')
        body = model(xml)
        result = body.evaluate(sample(), [1., 0., 0.])
        self.assertVector(result["com_world_m"], [2., 3., 4.])
        self.assertVector(result["total_world_kg_m2_s"], [2.5, 0., -.2])

    def test_revolute_link_com_motion_and_relative_spin(self):
        body = model(link("body") + link("child", xyz="1 0 0")
                     + joint("hip", "body", "child"))
        s = sample(positions={"hip": 0.}, velocities={"hip": 2.})
        result = body.evaluate(s, [0., 0., 0.])
        self.assertVector(result["com_world_m"], [.5, 0., 0.])
        self.assertVector(result["spin_world_kg_m2_s"], [0., 4., 0.])
        self.assertVector(result["orbital_world_kg_m2_s"], [0., 1., 0.])

    def test_both_native_axes_same_sign_and_chain_velocity(self):
        xml = link("body")
        for side, offset in (("left", "0 1 0"), ("right", "0 -1 0")):
            xml += link(side, xyz="1 0 0") + joint(side, "body", side, offset)
        body = model(xml)
        s = sample(positions=dict(left=0., right=0.), velocities=dict(left=2., right=2.))
        result = body.evaluate(s, [0., 0., 0.])
        self.assertVector(result["spin_world_kg_m2_s"], [0., 8., 0.])
        self.assertVector(result["orbital_world_kg_m2_s"], [0., 4. / 3., 0.])
        chain = model(link("body") + link("a") + link("b", xyz="1 0 0")
                      + joint("a", "body", "a") + joint("b", "a", "b", "1 0 0"))
        s = sample(positions=dict(a=0., b=0.), velocities=dict(a=2., b=3.))
        result = chain.evaluate(s, [0., 0., 0.])
        self.assertVector(result["spin_world_kg_m2_s"], [0., 14., 0.])
        self.assertVector(result["orbital_world_kg_m2_s"], [0., 28. / 3., 0.])

    def test_rotation_covariance_translation_and_velocity_invariance(self):
        body = model(link("body", mass=2., xyz=".1 .2 -.3")
                     + link("child", xyz=".4 -.1 .2") + joint("hip", "body", "child", ".3 0 .2"))
        s = sample(positions={"hip": .7}, velocities={"hip": -2.})
        omega = np.array([.4, -.8, .6])
        original = body.evaluate(s, omega)
        axis = np.array([1., 2., 3.]) / np.sqrt(14.)
        angle = .9
        q = np.r_[axis * np.sin(angle / 2), np.cos(angle / 2)]
        rotation = MODULE.quaternion_rotation(q)
        s["quaternion"] = q * 3.
        shift = np.array([100., -30., 7.])
        s["body_position"] = shift
        transformed = body.evaluate(s, rotation @ omega, [7., -8., 9.])
        self.assertVector(transformed["com_world_m"], rotation @ original["com_world_m"] + shift)
        for field in ("spin_world_kg_m2_s", "orbital_world_kg_m2_s", "total_world_kg_m2_s"):
            self.assertVector(transformed[field], rotation @ original[field])
            self.assertAlmostEqual(np.linalg.norm(transformed[field]), np.linalg.norm(original[field]))

    def test_quaternion_rates_nonuniform_source_time_sign_and_scale(self):
        axis = np.array([1., -2., 3.]) / np.sqrt(14.)
        samples = []
        for i, t in enumerate([0., .001, .003, .003, .004, .007]):
            q = np.r_[axis * np.sin(t), np.cos(t)] * ((-1.) ** i * (i + 1))
            samples.append(sample(t, q))
        rates = MODULE.body_angular_rates(samples)
        for rate in rates:
            self.assertVector(rate["omega_world_rad_s"], 2 * axis, atol=1e-5)
            self.assertEqual(rate["source"], "differentiated_pose_quaternion")
        self.assertEqual(rates[2], rates[3])
        self.assertTrue(rates[0]["one_sided"])
        self.assertFalse(rates[1]["one_sided"])
        self.assertEqual(rates[-1]["stencil_source_s"], [.003, .004, .007])

    def test_explicit_full_rates_preferred_and_frames_respected(self):
        s = sample(quaternion=[0., 0., np.sin(np.pi / 4), np.cos(np.pi / 4)])
        s["body_angular_velocity"] = [1., 2., 3.]
        s["source_stamps"]["angular_velocity"] = -.004
        rate = MODULE.body_angular_rates([s])[0]
        self.assertVector(rate["omega_world_rad_s"], [-2., 1., 3.])
        self.assertEqual(rate["source_stamp_s"], -.004)
        s["world_angular_velocity"] = [4., 5., 6.]
        self.assertVector(MODULE.body_angular_rates([s])[0]["omega_world_rad_s"], [4., 5., 6.])

    def test_differentiated_rates_are_world_expressed(self):
        # Initial roll rotates local +Y motion into world +Z.
        half_roll = np.pi / 4
        samples = []
        for t in (0., .001, .003):
            sy, cy = np.sin(t / 2), np.cos(t / 2)
            q = [np.sin(half_roll) * cy, np.cos(half_roll) * sy,
                 np.sin(half_roll) * sy, np.cos(half_roll) * cy]
            samples.append(sample(t, q))
        for rate in MODULE.body_angular_rates(samples):
            self.assertVector(rate["omega_world_rad_s"], [0., 0., 1.], atol=1e-6)

    def test_two_pose_secant_and_empty_trace(self):
        rates = MODULE.body_angular_rates([sample(), sample(.001, [0., np.sin(.001), 0., np.cos(.001)])])
        for rate in rates:
            self.assertVector(rate["omega_world_rad_s"], [0., 2., 0.], atol=1e-6)
            self.assertTrue(rate["one_sided"])
        self.assertEqual(MODULE.body_angular_rates([]), [])

    def test_nonphysical_inertia_and_mirrored_axis_rejected(self):
        with self.assertRaises(ValueError):
            model(link("body", diagonal=(1., 2., 4.)))
        xml = link("body") + link("child") + joint("hip", "body", "child")
        with self.assertRaises(ValueError):
            model(xml.replace('axis xyz="0 1 0"', 'axis xyz="0 -1 0"'))

    def test_missing_rate_does_not_turn_pitch_rate_into_full_vector(self):
        s = sample()
        rows, info = MODULE.momentum_trace(ET.fromstring("<robot>" + link("body") + "</robot>"), [s])
        self.assertIsNone(rows[0]["centroidal"])
        self.assertEqual(rows[0]["controller_pitch_rate_rad_s"], 123.)
        self.assertTrue(info["limitations"])
        self.assertIsNone(MODULE.body_angular_rates([s, copy.deepcopy(s)])[0]["omega_world_rad_s"])

    def test_bad_quaternions_timestamps_and_joint_state_rejected(self):
        for q in ([0., 0., 0., 0.], [0., float("nan"), 0., 1.]):
            with self.assertRaises(ValueError):
                MODULE.body_angular_rates([sample(quaternion=q)])
        for samples in ([sample(1.), sample(0.)], [sample(float("nan"))],
                        [sample(), sample(quaternion=[0., 1., 0., 0.])]):
            with self.assertRaises(ValueError):
                MODULE.body_angular_rates(samples)
        body = model(link("body") + link("child") + joint("hip", "body", "child"))
        with self.assertRaises(KeyError):
            body.evaluate(sample(), [0., 0., 0.])
        with self.assertRaises(ValueError):
            body.evaluate(sample(positions={"hip": 0.}, velocities={"hip": float("nan")}), [0., 0., 0.])

    def test_archived_model_and_samples(self):
        archive = ROOT / "log/jump_1789216663751564324"
        if not (archive / "expanded.urdf").exists():
            self.skipTest("Optional local archive unavailable")
        import json
        root = ET.parse(archive / "expanded.urdf").getroot()
        samples = json.loads((archive / "samples.json").read_text())[:5]
        rows, info = MODULE.momentum_trace(root, samples)
        self.assertAlmostEqual(info["mass_kg"], 3.938)
        self.assertEqual(len(info["inertial_links"]), 7)
        for s, row in zip(samples, rows):
            self.assertVector(row["centroidal"]["com_world_m"], s["com"])
            self.assertTrue(np.isfinite(row["centroidal"]["total_world_kg_m2_s"]).all())


if __name__ == "__main__":
    unittest.main()
