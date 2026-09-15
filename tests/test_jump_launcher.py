"""Launcher rejects mismatched model/controller options before starting ROS."""
from pathlib import Path
import subprocess
import unittest

SCRIPT = Path(__file__).resolve().parents[1] / 'run_jump.sh'


class JumpLauncherTests(unittest.TestCase):
    def test_shell_syntax(self):
        subprocess.run(['bash', '-n', str(SCRIPT)], check=True)

    def test_unknown_profile(self):
        result = subprocess.run([str(SCRIPT), 'unknown'], capture_output=True, text=True)
        self.assertEqual(result.returncode, 2)
        self.assertIn('Usage:', result.stderr)

    def test_owned_arguments_cannot_override_profile(self):
        for option in ('jump_model:=false', 'jump_leg_speed:=100',
                       'jump_leg_velocity_damping:=0', 'controller_config:=legacy.yaml'):
            with self.subTest(option=option):
                result = subprocess.run([str(SCRIPT), 'manual-curve', option],
                                        capture_output=True, text=True)
                self.assertEqual(result.returncode, 2)
                self.assertIn('owns model', result.stderr)


if __name__ == '__main__':
    unittest.main()
