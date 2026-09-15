"""Simulation contracts, without starting Gazebo or requiring tap_control."""
import unittest
import math
from pathlib import Path
from unittest.mock import patch
import xml.etree.ElementTree as ET

import xacro
import yaml

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / 'src/tap_robot_description/description/cad_robot'
SIM = ROOT / 'src/tap_sim'


class SimDescriptionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from ament_index_python.packages import get_package_share_directory

        def find_package(name):
            return str(SIM) if name == 'tap_sim' else get_package_share_directory(name)

        with patch('ament_index_python.packages.get_package_share_directory', side_effect=find_package):
            cls.robot = ET.fromstring(xacro.process_file(str(MODEL / 'simulation.urdf.xacro')).toxml())
        cls.preview = ET.fromstring(xacro.process_file(str(MODEL / 'robot.urdf.xacro')).toxml())

    def test_free_root_preserves_all_seven_inertials(self):
        self.assertEqual(self.robot.get('name'), 'tap_robot')
        self.assertEqual(len(self.robot.findall('link')), 7)
        self.assertIsNone(self.robot.find("link[@name='cad_ground']"))
        self.assertIsNotNone(self.preview.find("link[@name='cad_ground']"))
        total = 0
        for link in self.robot.findall('link'):
            old = self.preview.find(f"link[@name='{link.get('name')}']")
            self.assertEqual(ET.tostring(link.find('inertial')), ET.tostring(old.find('inertial')))
            self.assertIsNotNone(link.find('collision/geometry'))
            self.assertIsNone(old.find('collision'))
            total += float(link.find('inertial/mass').get('value'))
        self.assertAlmostEqual(total, 3.938)

    def test_native_axes_and_geometry(self):
        for joint in self.robot.findall('joint'):
            self.assertEqual(joint.find('axis').get('xyz'), '0 1 0')
            self.assertGreater(float(joint.find('dynamics').get('damping')), 0)
        for side in ('left', 'right'):
            wheel = self.robot.find(f"link[@name='cad_{side}_wheel_link']/collision/geometry/cylinder")
            self.assertEqual(float(wheel.get('radius')), 0.03)
            self.assertEqual(float(wheel.get('length')), 0.025)
        self.assertAlmostEqual(2 * (.125 + .002 + .0275), .309)
        self.assertAlmostEqual(.2348 - .10476742 - .1 - .03, .00003258)
        q = .11548309575
        contact_height = .03 + .20476742 * math.cos(q) + .03252141 * math.sin(q)
        self.assertGreater(.2372 - contact_height, 0)
        self.assertLess(.2372 - contact_height, .0001)

    def test_command_order_and_initial_positions(self):
        config = yaml.safe_load((SIM / 'config/ros2_control.yaml').read_text())
        self.assertEqual(config['controller_manager']['ros__parameters']['update_rate'], 1000)
        expected = ['cad_left_hip', 'cad_left_knee', 'cad_right_hip', 'cad_right_knee']
        self.assertEqual(config['leg_effort_controller']['ros__parameters']['joints'], expected)
        self.assertEqual(config['wheel_effort_controller']['ros__parameters']['joints'],
                         ['cad_left_wheel', 'cad_right_wheel'])
        joints = self.robot.findall('ros2_control/joint')
        self.assertEqual(len(joints), 6)
        for joint in joints:
            self.assertEqual(joint.find('command_interface').get('name'),
                             'velocity' if joint.get('name').endswith('wheel') else 'effort')
            expected_initial = .11548309575 if joint.get('name').endswith('hip') else 0
            self.assertEqual(float(joint.find("state_interface[@name='position']/param").text), expected_initial)

    def test_imu_not_rotated_with_visual_and_world_timing(self):
        sensor = self.robot.find("gazebo[@reference='cad_body']/sensor")
        self.assertEqual(sensor.find('pose').text, '0 0 0 0 0 0')
        self.assertEqual(sensor.find('gz_frame_id').text, 'cad_body')
        self.assertEqual(sensor.find('topic').text, '/imu')
        self.assertEqual(sensor.find('update_rate').text, '500')
        world = ET.parse(SIM / 'worlds/empty.sdf').find('world')
        self.assertEqual(world.get('name'), 'empty')
        self.assertEqual(world.find('gravity').text, '0 0 -9.81')
        self.assertEqual(float(world.find('physics/max_step_size').text), .001)

    def test_bringup_never_resets_or_changes_gravity(self):
        script = (SIM / 'scripts/activate_controllers').read_text()
        self.assertNotIn('set_pose', script)
        self.assertNotIn('set_physics', script)
        self.assertNotIn('/reset_controller', script)
        self.assertIn('self.imu_ready and self.joints_ready', script)
        self.assertIn("if auto_enable:", script)


if __name__ == '__main__':
    unittest.main()
