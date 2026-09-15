"""Check expanded URDF physics, source freshness, and analytic wheel inertia."""

import hashlib
import unittest
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import xacro
import yaml


MODEL = Path(__file__).resolve().parents[1] / 'src/tap_robot_description/description/cad_robot'


def tensor(values):
    p = {key: float(value) for key, value in values.items()}
    return np.array([[p['ixx'], p['ixy'], p['ixz']],
                     [p['ixy'], p['iyy'], p['iyz']],
                     [p['ixz'], p['iyz'], p['izz']]])


class InertialTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = ET.fromstring(xacro.process_file(str(MODEL / 'robot.urdf.xacro')).toxml())
        cls.links = {link.get('name'): link for link in cls.robot.findall('link')}
        cls.geometry = yaml.safe_load((MODEL / 'mesh_inertials.yaml').read_text())

    def test_supplier_masses_and_total(self):
        expected = {'cad_body': 2.902}
        for side in ('left', 'right'):
            expected.update({f'cad_{side}_thigh': 0.233, f'cad_{side}_shin': 0.150,
                             f'cad_{side}_wheel_link': 0.135})
        actual = {name: float(link.find('inertial/mass').get('value'))
                  for name, link in self.links.items() if link.find('visual') is not None}
        self.assertEqual(actual, expected)
        self.assertAlmostEqual(sum(actual.values()), 3.938)
        self.assertIsNone(self.links['cad_ground'].find('inertial'))
        self.assertEqual(sum(j.get('type') != 'fixed' for j in self.robot.findall('joint')), 6)

    def test_tensors_are_physical_and_com_within_mesh_bounds(self):
        import trimesh
        for name, link in self.links.items():
            if link.find('visual') is None:
                continue
            with self.subTest(link=name):
                inertial = link.find('inertial')
                matrix = tensor(inertial.find('inertia').attrib)
                self.assertTrue(np.isfinite(matrix).all())
                moments = np.linalg.eigvalsh(matrix)
                self.assertGreater(moments[0], 0)
                self.assertLessEqual(moments[2], sum(moments[:2]) + 1e-12)
                com = np.fromstring(inertial.find('origin').get('xyz'), sep=' ')
                rpy = np.fromstring(link.find('visual/origin').get('rpy'), sep=' ')
                self.assertEqual(inertial.find('origin').get('rpy'), link.find('visual/origin').get('rpy'))
                mesh_name = Path(link.find('visual/geometry/mesh').get('filename')).name
                mesh = trimesh.load_mesh(MODEL / 'meshes' / mesh_name)
                mesh.apply_transform(trimesh.transformations.euler_matrix(*rpy))
                self.assertTrue(np.all(com >= mesh.bounds[0]))
                self.assertTrue(np.all(com <= mesh.bounds[1]))
                np.testing.assert_allclose(com, mesh.center_mass, atol=1e-12)

    def test_only_body_rotates_negative_90_about_hip_axis(self):
        import trimesh
        body = self.links['cad_body']
        rpy = np.fromstring(body.find('visual/origin').get('rpy'), sep=' ')
        np.testing.assert_allclose(rpy, [0, -np.pi / 2, 0])
        rotation = trimesh.transformations.euler_matrix(*rpy)[:3, :3]
        expected_rotation = np.array([[0., 0., -1.], [0., 1., 0.], [1., 0., 0.]])
        np.testing.assert_allclose(rotation, expected_rotation, atol=1e-15)
        properties = self.geometry['body']
        inertial = body.find('inertial')
        np.testing.assert_allclose(
            np.fromstring(inertial.find('origin').get('xyz'), sep=' '),
            expected_rotation @ properties['center_of_mass'], atol=1e-12)
        # URDF tensor entries are in the rotated inertial frame, not link axes.
        inertial_rotation = trimesh.transformations.euler_matrix(
            *np.fromstring(inertial.find('origin').get('rpy'), sep=' '))[:3, :3]
        np.testing.assert_allclose(
            inertial_rotation @ tensor(inertial.find('inertia').attrib) @ inertial_rotation.T,
            expected_rotation @ (2.902 * tensor(properties['inertia_per_kg'])) @ expected_rotation.T,
            atol=1e-12)
        for name, link in self.links.items():
            if name not in ('cad_body', 'cad_ground'):
                np.testing.assert_allclose(np.fromstring(link.find('visual/origin').get('rpy'), sep=' '), 0)
        joints = {j.get('name'): j for j in self.robot.findall('joint')}
        for side, sign in [('left', 1), ('right', -1)]:
            for suffix, expected in [('hip', [0, sign * .125, 0]),
                                     ('knee', [-.01410935, sign * .002, -.10476742]),
                                     ('wheel', [.04663076, sign * .0275, -.1])]:
                joint = joints[f'cad_{side}_{suffix}']
                np.testing.assert_allclose(np.fromstring(joint.find('origin').get('xyz'), sep=' '), expected)
                self.assertIsNone(joint.find('origin').get('rpy'))
                self.assertEqual(joint.find('axis').get('xyz'), '0 1 0')

    def test_mirrored_parts_transform_com_and_full_tensor(self):
        reflection = np.diag([1., -1., 1.])
        for part in ('thigh', 'shin', 'wheel_link'):
            left = self.links[f'cad_left_{part}'].find('inertial')
            right = self.links[f'cad_right_{part}'].find('inertial')
            np.testing.assert_allclose(
                np.fromstring(right.find('origin').get('xyz'), sep=' '),
                reflection @ np.fromstring(left.find('origin').get('xyz'), sep=' '), atol=2e-8)
            np.testing.assert_allclose(tensor(right.find('inertia').attrib),
                                       reflection @ tensor(left.find('inertia').attrib) @ reflection,
                                       atol=1e-10)

    def test_wheel_matches_analytic_solid_cylinder(self):
        mass, radius, width = 0.135, 0.030, 0.025
        transverse = mass * (3 * radius**2 + width**2) / 12
        axial = mass * radius**2 / 2
        matrix = tensor(self.links['cad_left_wheel_link'].find('inertial/inertia').attrib)
        np.testing.assert_allclose(np.diag(matrix), [transverse, axial, transverse], rtol=0.002)

    def test_geometry_matches_mesh_hashes(self):
        for name, properties in self.geometry.items():
            digest = hashlib.sha256((MODEL / 'meshes' / f'{name}.stl').read_bytes()).hexdigest()
            self.assertEqual(properties['mesh_sha256'], digest,
                             f'{name}: regenerate mesh inertials after editing meshes')


if __name__ == '__main__':
    unittest.main()
