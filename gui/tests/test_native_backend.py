import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

from tap_gui.native_backend import NativeBackend


class NativeBackendTest(unittest.TestCase):
    def test_parse_is_key_value_only(self):
        fields = NativeBackend._parse("STATE|mode=Balance|pitch=0.1|ground_truth_to_controller=0")
        self.assertEqual(fields["mode"], "Balance")
        self.assertEqual(fields["ground_truth_to_controller"], "0")

    def test_runner_is_source_tree_native_binary(self):
        with TemporaryDirectory() as directory:
            root = Path(directory)
            runner = root / "build/simulation/bin/tap_mujoco_gim_v3"
            runner.parent.mkdir(parents=True)
            runner.write_text("#!/bin/sh\nexit 0\n")
            runner.chmod(0o755)
            self.assertEqual(NativeBackend(root=root).runner_path(), runner)


if __name__ == "__main__":
    unittest.main()
