"""Unit tests for the gz service request shapes, mocked subprocess.run."""
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from gui import gz_world


class PauseAndSetPoseTest(unittest.TestCase):
    def test_pause_builds_the_proven_world_control_request(self):
        with patch('gui.gz_world.subprocess.run') as run:
            run.return_value = Mock(returncode=0, stdout='data: true', stderr='')
            gz_world.pause(True)
            args = run.call_args.args[0]
            self.assertEqual(args[:4], ['gz', 'service', '-s', '/world/empty/control'])
            self.assertIn('gz.msgs.WorldControl', args)
            self.assertIn('gz.msgs.Boolean', args)
            self.assertIn('pause: true', args)
            gz_world.pause(False)
            self.assertIn('pause: false', run.call_args.args[0])

    def test_set_pose_builds_the_proven_pose_request(self):
        with patch('gui.gz_world.subprocess.run') as run:
            run.return_value = Mock(returncode=0, stdout='data: true', stderr='')
            gz_world.set_pose('tap_robot', 0., 0., .2372, w=1.)
            args = run.call_args.args[0]
            self.assertEqual(args[:4], ['gz', 'service', '-s', '/world/empty/set_pose'])
            self.assertIn('gz.msgs.Pose', args)
            request = args[args.index('--req') + 1]
            self.assertEqual(request, 'name: "tap_robot" position {x: 0.0 y: 0.0 z: 0.2372} '
                                       'orientation {w: 1.0 x: 0.0 y: 0.0 z: 0.0}')

    def test_multi_step_steps_then_re_asserts_pause(self):
        # multi_step clears the world's own pause flag as a side effect (both
        # here and in the wheel_leg reference GUI's _gz_step), so the caller
        # must never be left un-paused without asking for it.
        with patch('gui.gz_world.subprocess.run') as run:
            run.return_value = Mock(returncode=0, stdout='data: true', stderr='')
            gz_world.multi_step(200)
            self.assertEqual(run.call_count, 2)
            step_args = run.call_args_list[0].args[0]
            self.assertEqual(step_args[:4], ['gz', 'service', '-s', '/world/empty/control'])
            self.assertIn('multi_step: 200', step_args)
            pause_args = run.call_args_list[1].args[0]
            self.assertIn('pause: true', pause_args)

    def test_nonzero_returncode_raises(self):
        with patch('gui.gz_world.subprocess.run') as run:
            run.return_value = Mock(returncode=1, stdout='', stderr='nope')
            with self.assertRaises(RuntimeError):
                gz_world.pause(True)

    def test_missing_data_true_raises(self):
        with patch('gui.gz_world.subprocess.run') as run:
            run.return_value = Mock(returncode=0, stdout='data: false', stderr='')
            with self.assertRaises(RuntimeError):
                gz_world.pause(True)


if __name__ == '__main__':
    unittest.main()
