"""Pure YAML-derived ROS parameter layering; not a live parameter service dump."""
from copy import deepcopy


def canonical_ros_parameters(*layers, node='balance_controller_node'):
    """Flatten each file before merging; later dotted/nested definitions win.

    Scalar types and arrays are preserved, without coercing bool/int/double.
    Only the explicitly selected node is supported; wildcard resolution is not
    inferred from these harness-specific configuration files.
    """
    result = {}

    def flatten(mapping, prefix=''):
        if not isinstance(mapping, dict):
            raise ValueError('ROS parameter namespace must be a mapping')
        for key, value in mapping.items():
            if not isinstance(key, str) or not key:
                raise ValueError('ROS parameter names must be nonempty strings')
            name = f'{prefix}.{key}' if prefix else key
            if isinstance(value, dict):
                flatten(value, name)
            else:
                result[name] = deepcopy(value)

    for layer in layers:
        if not isinstance(layer, dict) or node not in layer:
            raise ValueError(f'Missing explicit parameter node {node}')
        entry = layer[node]
        if not isinstance(entry, dict) or 'ros__parameters' not in entry:
            raise ValueError(f'Missing ros__parameters for {node}')
        flatten(entry['ros__parameters'])
    return {node: {'ros__parameters': result}}
