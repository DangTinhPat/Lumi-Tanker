"""jump_launch_command rejects mismatched model/controller options before
starting ROS - the Python port of run_jump.sh (removed 2026-09-16; see
tools/tap_launch.py)."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from tap_launch import JUMP_PROFILES, jump_launch_command


class JumpLauncherTests(unittest.TestCase):
    def test_unknown_profile(self):
        with self.assertRaisesRegex(ValueError, 'Unknown jump profile'):
            jump_launch_command('/workspace', 'unknown')

    def test_owned_arguments_cannot_override_profile(self):
        for option in ('jump_model:=false', 'jump_leg_speed:=100',
                       'jump_leg_velocity_damping:=0', 'controller_config:=legacy.yaml'):
            with self.subTest(option=option):
                with self.assertRaisesRegex(ValueError, 'owns model'):
                    jump_launch_command('/workspace', 'manual-curve', [option])

    def test_rated_and_manual_curve_build_the_expected_launch_command(self):
        for profile, (config, speed) in JUMP_PROFILES.items():
            command = jump_launch_command('/workspace', profile, ['gui:=false'])
            self.assertEqual(command[:4], ['ros2', 'launch', 'tap_sim', 'sim.launch.py'])
            self.assertIn('jump_model:=true', command)
            self.assertIn(f'jump_leg_speed:={speed}', command)
            self.assertIn('jump_leg_velocity_damping:=0.6', command)
            self.assertIn(f'controller_config:=/workspace/src/tap_control/config/{config}', command)
            self.assertIn('gui:=false', command)


if __name__ == '__main__':
    unittest.main()
