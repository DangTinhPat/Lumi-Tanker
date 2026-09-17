"""Regression guard for the retired simulator-state jump harness."""
from pathlib import Path
import runpy
import unittest


ROOT = Path(__file__).resolve().parents[1]


class SensorOnlyDynamicContractTest(unittest.TestCase):
    def test_legacy_entrypoint_cannot_enable_jump(self):
        harness = runpy.run_path(str(ROOT / 'tools/jump_smoke.py'))
        self.assertEqual(harness['sensor_only_contract'](),
                         {'jump_enabled': False, 'pitch_source': 'imu'})


if __name__ == '__main__':
    unittest.main()
