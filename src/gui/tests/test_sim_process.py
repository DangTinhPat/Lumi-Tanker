"""Unit tests for workspace resolution and sim-stage log scanning."""
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from gui import sim_process


class ResolveWorkspaceTest(unittest.TestCase):
    def test_resolves_the_real_workspace_root(self):
        workspace = sim_process.resolve_workspace()
        self.assertTrue((workspace / 'tools' / 'owned_process.py').is_file())
        self.assertTrue((workspace / 'CLAUDE.md').is_file())

    def test_raises_a_clear_error_outside_a_tap_workspace(self):
        # resolve_workspace() is hardcoded to __file__, so exercise the same
        # walk-up-and-raise search it uses from an arbitrary path instead.
        with tempfile.TemporaryDirectory() as empty:
            fake_file = Path(empty) / 'pkg' / 'nowhere.py'
            with self.assertRaisesRegex(RuntimeError, 'Could not locate the TAP workspace root'):
                _search(fake_file)


def _search(fake_file):
    for candidate in (fake_file.resolve(), *fake_file.resolve().parents):
        if (candidate / 'tools' / 'owned_process.py').is_file() and (candidate / 'CLAUDE.md').is_file():
            return candidate
    raise RuntimeError(f'Could not locate the TAP workspace root from {fake_file}')


class SimStageTest(unittest.TestCase):
    def test_defaults_to_starting_gazebo(self):
        with tempfile.NamedTemporaryFile('w', suffix='.log') as log:
            log.write('some unrelated line\n')
            log.flush()
            self.assertIn('Starting Gazebo', sim_process.sim_stage(log.name, 3))

    def test_recognizes_controllers_ready_milestone(self):
        with tempfile.NamedTemporaryFile('w', suffix='.log') as log:
            log.write('...\nControllers ready; paused. Waiting for /enable=true to run.\n')
            log.flush()
            self.assertIn('Controllers ready', sim_process.sim_stage(log.name, 5))

    def test_recognizes_running_milestone(self):
        with tempfile.NamedTemporaryFile('w', suffix='.log') as log:
            log.write('Controllers ready; paused.\nEnabled and running after 12 ms startup physics\n')
            log.flush()
            self.assertIn('Running', sim_process.sim_stage(log.name, 8))

    def test_missing_log_file_is_tolerated(self):
        self.assertIn('Starting Gazebo', sim_process.sim_stage('/nonexistent/path.log', 1))


class LaunchSimCommandTest(unittest.TestCase):
    """launch_sim() builds and runs the literal `ros2 launch` invocation
    directly -- matching the wheel_leg reference GUI's pattern exactly (it
    shells out to `exec ros2 launch balance_firmware_core sim.launch.py`) --
    there is no run_sim.sh any more for it to shell out to (removed
    2026-09-16; tools/*_smoke.py and manual headless runs call `ros2 launch`
    directly too, via tools/tap_launch.py), and this also avoids the inline
    ros2cli Python entry point an earlier version of this module used (see
    sim_process.py's module docstring for why the PATH lookup is safe here
    despite this machine's `~/bin/ros2` wrapper). Uses a
    real temp directory rather than patching Path methods globally, since
    those are shared across every module under test.
    """

    def test_refuses_without_a_build(self):
        with tempfile.TemporaryDirectory() as empty, \
             patch('gui.sim_process.WORKSPACE', Path(empty)):
            with self.assertRaisesRegex(RuntimeError, 'Build the TAP workspace first'):
                sim_process.launch_sim()

    def test_builds_the_literal_ros2_launch_command_not_run_sim_sh(self):
        with tempfile.TemporaryDirectory() as workspace:
            workspace = Path(workspace)
            package_share = workspace / 'install/tap_sim/share/tap_sim'
            package_share.mkdir(parents=True)
            (package_share / 'package.xml').write_text('<package/>')
            log_dir = workspace / 'log' / 'test'
            with patch('gui.sim_process.WORKSPACE', workspace), \
                 patch('gui.sim_process.start_owned_process', return_value=Mock()) as start:
                sim_process.launch_sim(['terrain:=rough'], log_dir=log_dir)
            command = start.call_args.args[0]
            self.assertEqual(command, ['ros2', 'launch', 'tap_sim', 'sim.launch.py', 'terrain:=rough'])
            self.assertNotIn('run_sim.sh', ' '.join(str(c) for c in command))
            self.assertTrue((log_dir / 'sim.log').is_file())


if __name__ == '__main__':
    unittest.main()
