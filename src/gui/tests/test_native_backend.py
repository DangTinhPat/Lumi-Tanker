import unittest

from gui.native_backend import NativeBackend


class NativeBackendTest(unittest.TestCase):
    def test_parse_is_key_value_only(self):
        fields = NativeBackend._parse("STATE|mode=Balance|pitch=0.1|ground_truth_to_controller=0")
        self.assertEqual(fields["mode"], "Balance")
        self.assertEqual(fields["ground_truth_to_controller"], "0")

    def test_runner_is_source_tree_native_binary(self):
        backend = NativeBackend()
        runner = backend.runner_path()
        self.assertTrue(runner.name == "tap_mujoco_native")
        self.assertTrue(runner.is_file())


if __name__ == "__main__":
    unittest.main()
