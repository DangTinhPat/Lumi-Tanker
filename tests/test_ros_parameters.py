"""Canonical parameter evidence without ROS or a simulator."""
from pathlib import Path
import runpy
import unittest

canonical = runpy.run_path(str(Path(__file__).resolve().parents[1]/'tools/ros_parameters.py'))['canonical_ros_parameters']


def layer(values):
    return {'balance_controller_node': {'ros__parameters': values}}


class CanonicalParametersTest(unittest.TestCase):
    def test_later_file_overrides_both_notations_without_duplicate_names(self):
        base = layer({'mpc': {'max_ground_accel': 6.}, 'station_keeper': {'enabled': False}})
        override = layer({'mpc.max_ground_accel': 2., 'station_keeper.enabled': True})
        self.assertEqual(canonical(base, override), layer({'mpc.max_ground_accel': 2., 'station_keeper.enabled': True}))
        self.assertEqual(canonical(override, base), layer({'mpc.max_ground_accel': 6., 'station_keeper.enabled': False}))

    def test_bool_int_double_string_arrays_keep_their_types(self):
        values = {'enabled': True, 'horizon': 20, 'gain': 2.5, 'profile': 'rated', 'pose': [0., -1.6]}
        result = canonical(layer({'jump': values}))['balance_controller_node']['ros__parameters']
        for key, value in values.items():
            self.assertIs(type(result['jump.'+key]), type(value))
            self.assertEqual(result['jump.'+key], value)
        changed = canonical(layer({'gain': 2}), layer({'gain': 2.}))
        self.assertIs(type(changed['balance_controller_node']['ros__parameters']['gain']), float)

    def test_pose_bounds_remain_direct_parameter_names(self):
        result = canonical(layer({'pose_knee_min_rad': -.8}),
                           layer({'pose_knee_min_rad': -1.6, 'pose_hip_max_rad': 1.05}))
        self.assertEqual(result['balance_controller_node']['ros__parameters']['pose_knee_min_rad'], -1.6)

    def test_same_layer_nested_dotted_definition_order(self):
        result = canonical(layer({'station': {'enabled': False}, 'station.enabled': True}))
        self.assertTrue(result['balance_controller_node']['ros__parameters']['station.enabled'])

    def test_input_is_not_mutated(self):
        source = layer({'pose': [1., 2.]})
        canonical(source)['balance_controller_node']['ros__parameters']['pose'].append(3.)
        self.assertEqual(source, layer({'pose': [1., 2.]}))

    def test_missing_node_and_invalid_parameter_namespace_rejected(self):
        for invalid in ({}, {'balance_controller_node': {}}, layer([]), layer({1: True})):
            with self.assertRaises(ValueError):
                canonical(invalid)


if __name__ == '__main__':
    unittest.main()
