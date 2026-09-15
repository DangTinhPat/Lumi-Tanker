#!/usr/bin/env python3
"""Split the supplied CAD assembly into joint-local, metre-scale RViz meshes.

Optional import-time dependencies: numpy, trimesh. Runtime needs neither.
"""

import argparse
from pathlib import Path

import numpy as np
import trimesh


# CAD X is lateral, CAD Y points down, CAD Z points backwards.
CAD_TO_ROS = np.array([[0, 0, -1], [1, 0, 0], [0, -1, 0]])
HIP = np.array([125.0, 0.0, 0.0])
KNEE = np.array([127.0, 104.76742, 14.10935])
WHEEL = np.array([154.5, 204.76742, -32.52141])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('assembly', type=Path)
    parser.add_argument('--output', type=Path, default=Path(__file__).resolve().parents[1]
                        / 'src/tap_robot_description/description/cad_robot/meshes')
    args = parser.parse_args()
    assembly = trimesh.load_mesh(args.assembly)
    parts = assembly.split(only_watertight=False)
    if len(parts) != 7:
        raise ValueError(f'Expected seven connected CAD parts, got {len(parts)}')

    meshes = {}
    for part in parts:
        width = part.extents[0]
        side = 'left' if part.bounds.mean(axis=0)[0] > 0 else 'right'
        sign = 1 if side == 'left' else -1
        if np.isclose(width, 205):
            name, anchor = 'body', np.zeros(3)
        elif np.isclose(width, 44):
            name, anchor = f'{side}_thigh', HIP.copy()
        elif np.isclose(width, 30):
            name, anchor = f'{side}_shin', KNEE.copy()
        elif np.isclose(width, 25):
            name, anchor = f'{side}_wheel', WHEEL.copy()
        else:
            raise ValueError(f'Unrecognized CAD part width: {width}')
        anchor[0] *= sign
        if name in meshes:
            raise ValueError(f'Duplicate component: {name}')
        part.vertices = (part.vertices - anchor) @ CAD_TO_ROS.T * 0.001
        meshes[name] = part

    args.output.mkdir(parents=True, exist_ok=True)
    for name, mesh in sorted(meshes.items()):
        mesh.export(args.output / f'{name}.stl')
        print(f'{name}: {len(mesh.faces)} triangles')


if __name__ == '__main__':
    main()
