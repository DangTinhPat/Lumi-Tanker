"""Offline ADAM projection checks; no simulation or jump feasibility claims."""

import importlib.util
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("jump_dynamics", ROOT / "tools/jump_dynamics.py")
MODEL = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODEL)


def transform(xyz=(0., 0., 0.), rpy=(0., 0., 0.)):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    H = np.eye(4)
    H[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                  [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]]
    H[:3, 3] = xyz
    return H


def origin(element):
    if element is None:
        return np.eye(4)
    return transform(np.fromstring(element.get("xyz", "0 0 0"), sep=" "),
                     np.fromstring(element.get("rpy", "0 0 0"), sep=" "))


def oracle(root, q):
    """Independent homogeneous-transform FK, retaining each URDF COM and tensor."""
    frames = {"cad_body": transform((q[0], 0., q[1]), (0., q[2], 0.))}
    joints = {j.find("child").get("link"): j for j in root.findall("joint")}
    values = dict(zip(MODEL.JOINT_NAMES, np.tile(q[3:], 2)))

    def frame(name):
        if name not in frames:
            j = joints[name]
            frames[name] = (frame(j.find("parent").get("link")) @ origin(j.find("origin"))
                            @ transform(rpy=(0., values[j.get("name")], 0.)))
        return frames[name]

    inertials = {}
    for link in root.findall("link"):
        name = link.get("name")
        data = link.find("inertial")
        H = frame(name) @ origin(data.find("origin"))
        entries = data.find("inertia").attrib
        I = np.array([[float(entries["i" + "".join(sorted((a, b)))]) for b in "xyz"] for a in "xyz"])
        inertials[name] = (float(data.find("mass").get("value")), H[:3, 3], H[:3, :3] @ I @ H[:3, :3].T)
    mass = sum(item[0] for item in inertials.values())
    com = sum(m * pos for m, pos, _ in inertials.values()) / mass
    axle = (frame("cad_left_wheel_link")[:3, 3] + frame("cad_right_wheel_link")[:3, 3]) / 2
    return com, axle, inertials


@unittest.skipUnless(importlib.util.find_spec("adam") and importlib.util.find_spec("casadi"),
                     "Requires the dedicated ADAM/CasADi offline environment")
class JumpDynamicsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        xacro = shutil.which("xacro")
        if xacro is None:
            raise unittest.SkipTest("Current CAD fixture requires xacro in PATH")
        xml = subprocess.check_output(
            [xacro, str(ROOT / "src/tap_robot_description/description/cad_robot/robot.urdf.xacro"),
             "simulation:=true", "jump_model:=true"], text=True)
        cls.tmp = tempfile.TemporaryDirectory(prefix="tap-jump-dynamics-")
        cls.addClassCleanup(cls.tmp.cleanup)
        cls.path = Path(cls.tmp.name) / "expanded.urdf"
        cls.path.write_text(xml)
        cls.root = ET.fromstring(xml)
        cls.dynamics = MODEL.build_dynamics(cls.path)
        cls.metadata = MODEL.model_metadata(cls.path)

    def evaluate(self, q, v=None):
        return {key: np.asarray(value) for key, value in self.dynamics(
            q=q, v=np.zeros(6) if v is None else v).items()}

    def test_api_and_mass_metadata(self):
        self.assertEqual(self.dynamics.name_in(), ["q", "v"])
        self.assertEqual(self.dynamics.name_out(),
                         ["M", "bias", "com", "com_velocity", "axle", "Jcontact"])
        self.assertEqual([self.dynamics.size_out(i) for i in range(6)],
                         [(6, 6), (6, 1), (3, 1), (3, 1), (3, 1), (2, 6)])
        self.assertAlmostEqual(self.metadata["total_mass_kg"], 3.938)
        self.assertEqual(len(self.metadata["link_masses_kg"]), 7)
        self.assertEqual(len(self.metadata["urdf_sha256"]), 64)
        self.assertEqual(MODEL.S.shape, (12, 6))
        np.testing.assert_array_equal(MODEL.S[6:9], MODEL.S[9:12])

    def test_spd_and_translational_mass(self):
        rng = np.random.default_rng(914)
        for _ in range(24):
            q = rng.uniform(-1., 1., 6)
            M = self.evaluate(q)["M"]
            np.testing.assert_allclose(M, M.T, atol=1e-12)
            self.assertGreater(np.linalg.eigvalsh(M).min(), 0.)
            np.testing.assert_allclose(M[:2, :2], np.eye(2) * 3.938, atol=1e-12)

    def test_fk_and_com_velocity_finite_differences(self):
        rng = np.random.default_rng(26)
        eps = 1e-6
        for _ in range(12):
            q, v = rng.uniform(-1., 1., (2, 6))
            result = self.evaluate(q, v)
            com, axle, _ = oracle(self.root, q)
            np.testing.assert_allclose(result["com"].ravel(), com, atol=1e-12)
            np.testing.assert_allclose(result["axle"].ravel(), axle, atol=1e-12)
            forward, backward = oracle(self.root, q + eps*v), oracle(self.root, q - eps*v)
            np.testing.assert_allclose(result["com_velocity"].ravel(),
                                       (forward[0]-backward[0])/(2*eps), atol=2e-9)
            jac = np.column_stack([(oracle(self.root, q + eps*d)[1] -
                                    oracle(self.root, q - eps*d)[1])/(2*eps) for d in np.eye(6)])
            expected = jac[[0, 2], :]
            expected[0] -= .03 * np.array([0, 0, 1, 1, 1, 1])
            np.testing.assert_allclose(result["Jcontact"], expected, atol=2e-9)

    def test_gravity_is_potential_gradient(self):
        q = np.array([.12, .28, -.21, .73, -1.2, .54])
        eps = 1e-6
        gradient = np.array([(oracle(self.root, q + eps*d)[0][2] -
                              oracle(self.root, q - eps*d)[0][2])/(2*eps)
                             for d in np.eye(6)]) * 3.938 * MODEL.GRAVITY
        bias = self.evaluate(q)["bias"].ravel()
        np.testing.assert_allclose(bias, gradient, atol=2e-8)
        self.assertAlmostEqual(bias[0], 0.)
        self.assertAlmostEqual(bias[1], 3.938 * 9.81)

    def test_mass_matches_independent_link_kinetic_energy(self):
        q = np.array([.12, .28, -.21, .73, -1.2, .54])
        v = np.array([.2, -.4, .8, -.5, .7, 1.2])
        eps = 1e-6
        inertials = oracle(self.root, q)[2]
        plus = oracle(self.root, q + eps*v)[2]
        minus = oracle(self.root, q - eps*v)[2]
        energy = 0.
        for name, (mass, _, inertia) in inertials.items():
            velocity = (plus[name][1] - minus[name][1])/(2*eps)
            rate = v[2]
            if "thigh" in name or "shin" in name or "wheel" in name:
                rate += v[3]
            if "shin" in name or "wheel" in name:
                rate += v[4]
            if "wheel" in name:
                rate += v[5]
            energy += .5 * (mass * velocity @ velocity + rate**2 * inertia[1, 1])
        self.assertAlmostEqual(.5 * v @ self.evaluate(q)["M"] @ v, energy, delta=2e-9)

    def test_contact_sign_and_carrier_rate(self):
        J = self.evaluate([0., .24, 0., 0., 0., 0.])["Jcontact"]
        np.testing.assert_allclose(J[:, 5], [-.03, 0.], atol=1e-12)
        np.testing.assert_allclose(J @ np.array([.03, 0., 0., 0., 0., 1.]), 0., atol=1e-12)
        # Cancel axle translation at nonzero carrier rates and match total spin.
        q = np.array([0., .3, -.2, .6, -1.1, .8])
        J = self.evaluate(q)["Jcontact"]
        v = np.array([0., 0., .4, -.3, .8, -.9])
        v[:2] = -J[:, 2:] @ v[2:]
        np.testing.assert_allclose(J @ v, 0., atol=1e-12)

    def test_velocity_bias_energy_identity(self):
        rng = np.random.default_rng(501)
        eps = 1e-6
        for _ in range(8):
            q, v = rng.uniform(-1., 1., (2, 6))
            result = self.evaluate(q, v)
            Mdot = (self.evaluate(q + eps*v)["M"] - self.evaluate(q - eps*v)["M"])/(2*eps)
            gravity_power = 3.938 * MODEL.GRAVITY * result["com_velocity"][2, 0]
            self.assertAlmostEqual(v @ result["bias"].ravel(),
                                   .5 * v @ Mdot @ v + gravity_power, delta=2e-8)

    def test_symbolic_mx_derivatives(self):
        import casadi as cs
        q, v = cs.MX.sym("q", 6), cs.MX.sym("v", 6)
        outputs = self.dynamics(q, v)
        derivative = cs.Function("derivative", [q, v], [cs.jacobian(outputs[1], q)])
        self.assertTrue(np.isfinite(np.asarray(derivative(np.zeros(6), np.ones(6)))).all())

    def test_reject_mirrored_axis_and_preview_mount(self):
        xml = self.path.read_text()
        for invalid in (
                xml.replace('xyz="0 1 0"', 'xyz="0 -1 0"', 1),
                xml.replace('</robot>', '<link name="cad_ground"/></robot>')):
            path = Path(self.tmp.name) / "invalid.urdf"
            path.write_text(invalid)
            with self.assertRaises(ValueError):
                MODEL.build_dynamics(path)


if __name__ == "__main__":
    unittest.main()
