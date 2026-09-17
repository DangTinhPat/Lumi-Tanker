"""Opt-in actuator contracts without launching Gazebo or exporting truth."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET
import xacro

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / 'src/tap_robot_description/description/cad_robot/simulation.urdf.xacro'


class JumpDescriptionTests(unittest.TestCase):
    def setUp(self):
        from ament_index_python.packages import get_package_share_directory

        def find_package(name):
            return str(ROOT / 'src/tap_sim') if name == 'tap_sim' else get_package_share_directory(name)

        lookup = patch('ament_index_python.packages.get_package_share_directory', side_effect=find_package)
        lookup.start()
        self.addCleanup(lookup.stop)

    def test_opt_in_preserves_inertials_and_default_interfaces(self):
        normal = ET.fromstring(xacro.process_file(str(MODEL)).toxml())
        jump = ET.fromstring(xacro.process_file(str(MODEL), mappings={'jump_model': 'true'}).toxml())
        self.assertEqual(normal.find('ros2_control/hardware/plugin').text, 'gz_ros2_control/GazeboSimSystem')
        self.assertEqual(jump.find('ros2_control/hardware/plugin').text, 'tap_sim/BoundedGazeboSystem')
        for link in normal.findall('link'):
            other = jump.find(f"link[@name='{link.get('name')}']")
            self.assertEqual(ET.tostring(link.find('inertial')), ET.tostring(other.find('inertial')))
        for joint in jump.findall('joint'):
            wheel = joint.get('name').endswith('wheel')
            self.assertEqual(float(joint.find('limit').get('effort')), .13 if wheel else 5)
            self.assertEqual(float(joint.find('limit').get('velocity')), 1000)
            self.assertEqual(float(joint.find('dynamics').get('damping')), .001 if wheel else .02)
        # Contact and model-pose telemetry are permanently excluded. Landing
        # phase is inferred by the controller from MPU6500 data and encoders.
        self.assertEqual(len(jump.findall(".//sensor[@type='contact']")), 0)
        self.assertEqual(len(normal.findall(".//sensor[@type='contact']")), 0)
        self.assertIsNone(jump.find(".//plugin[@name='tap_sim::ContactSnapshot']"))
        self.assertIsNone(jump.find(".//plugin[@name='gz::sim::systems::PosePublisher']"))
        self.assertEqual(len(normal.findall(".//sensor[@type='contact']")), 0)

    def test_experimental_speed_is_explicit(self):
        robot = ET.fromstring(xacro.process_file(str(MODEL), mappings={
            'jump_model': 'true', 'jump_leg_speed': '28.274333882308138',
            'jump_leg_velocity_damping': '0.2'}).toxml())
        self.assertEqual(float(robot.find("joint[@name='cad_left_knee']/limit").get('velocity')), 1000)
        self.assertEqual(float(robot.find("ros2_control/hardware/param[@name='leg_speed_limit']").text), 28.274333882308138)
        self.assertEqual(float(robot.find("ros2_control/hardware/param[@name='leg_velocity_damping']").text), .2)

    @unittest.skipUnless(shutil.which('gz'), 'Gazebo SDF CLI unavailable')
    def test_converted_sdf_has_no_truth_sensor(self):
        xml = xacro.process_file(str(MODEL), mappings={'jump_model': 'true'}).toxml()
        with tempfile.TemporaryDirectory() as directory:
            model = Path(directory) / 'robot.urdf'
            model.write_text(xml)
            converted = subprocess.run(['gz', 'sdf', '-p', str(model)],
                                       check=True, capture_output=True, text=True, timeout=15)
        sdf = ET.fromstring(converted.stdout)
        joints = sdf.findall('model/joint')
        self.assertEqual(len(joints), 6)
        for joint in joints:
            self.assertEqual(float(joint.find('axis/limit/velocity').text), 1000)
        self.assertEqual(len(sdf.findall(".//sensor[@type='contact']")), 0)


if __name__ == '__main__':
    unittest.main()
