#!/usr/bin/env python3
"""Offline neutral-branch and sampled CAD clearance audit; never launches ROS/Gazebo."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import brentq
import trimesh

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/tap_teleop'))
from tap_teleop.pose_curves import RobotGeometry, rotation, vector


def vertices_inside(mesh, points):
    """Solid-angle winding on watertight triangles, avoiding optional rtree/FCL."""
    if not mesh.is_watertight or not mesh.is_winding_consistent:
        raise ValueError('Containment audit requires consistently wound, watertight meshes')
    low, high = mesh.bounds
    points = points[np.all((points > low) & (points < high), axis=1)]
    count = 0
    for start in range(0, len(points), 24):
        delta = mesh.triangles[None, :, :, :] - points[start:start+24, None, None, :]
        a, b, c = delta[:, :, 0], delta[:, :, 1], delta[:, :, 2]
        na, nb, nc = [np.linalg.norm(v, axis=2) for v in (a, b, c)]
        dot = lambda u, v: np.einsum('ijk,ijk->ij', u, v)
        numerator = dot(a, np.cross(b, c))
        denominator = na*nb*nc + dot(a, b)*nc + dot(b, c)*na + dot(c, a)*nb
        solid_angle = np.sum(2*np.arctan2(numerator, denominator), axis=1)
        # Interior winding is +/-4*pi; a regular surface point is 2*pi.
        count += int(np.count_nonzero(np.abs(solid_angle) > 3*np.pi))
    return count


def sampled_surface_gap(first, second):
    """Minimum sampled vertex-to-triangle distance, NOT exact surface separation."""
    minimum = float('inf')
    for target, source in ((first, second), (second, first)):
        for start in range(0, len(source.vertices), 32):
            _, distance, _ = trimesh.proximity.closest_point_naive(
                target, source.vertices[start:start+32])
            minimum = min(minimum, float(np.min(distance)))
    return minimum


def origin_transform(element):
    transform = np.eye(4)
    roll, pitch, yaw = vector(element, 'rpy')
    transform[:3, :3] = (rotation([0, 0, 1], yaw) @ rotation([0, 1, 0], pitch)
                         @ rotation([1, 0, 0], roll))
    transform[:3, 3] = vector(element, 'xyz')
    return transform


class SquatAudit:
    def __init__(self, xacro_path):
        self.path = Path(xacro_path).resolve()
        self.model = RobotGeometry(self.path)
        expanded = subprocess.run(['xacro', str(self.path), 'simulation:=true'],
                                  check=True, capture_output=True, text=True).stdout
        self.xml = ET.fromstring(expanded)
        self.expanded_hash = hashlib.sha256(expanded.encode()).hexdigest()
        self.links = {link.get('name'): link for link in self.xml.findall('link')}
        hip = self.xml.find("joint[@name='cad_left_hip']/limit")
        knee = self.xml.find("joint[@name='cad_left_knee']/limit")
        self.hip_limits = tuple(float(hip.get(key)) for key in ('lower', 'upper'))
        self.knee_limits = tuple(float(knee.get(key)) for key in ('lower', 'upper'))
        self.meshes, self.mesh_hashes = {}, {}
        for side in ('left', 'right'):
            for part in ('thigh', 'wheel_link'):
                name = f'cad_{side}_{part}'
                visual = self.links[name].find('visual')
                mesh_xml = visual.find('geometry/mesh')
                uri = mesh_xml.get('filename')
                # Saved audit snapshots keep original package URIs; prefer adjacent meshes.
                local = self.path.parent / 'meshes' / Path(uri).name
                if not local.exists():
                    if uri.startswith('package://'):
                        from ament_index_python.packages import get_package_share_directory
                        package, relative = uri[len('package://'):].split('/', 1)
                        local = Path(get_package_share_directory(package)) / relative
                    else:
                        local = self.path.parent / uri
                mesh = trimesh.load_mesh(local)
                mesh.apply_scale(vector(mesh_xml, 'scale', (1, 1, 1)))
                mesh.apply_transform(origin_transform(visual.find('origin')))
                self.meshes[name] = mesh
                self.mesh_hashes[name] = hashlib.sha256(local.read_bytes()).hexdigest()

    def neutral_roots(self, knee):
        residual = lambda hip: self.model.evaluate([hip, knee, hip, knee])['dx']
        grid = np.linspace(*self.hip_limits, 129)
        values = [residual(h) for h in grid]
        roots = []
        for index, h in enumerate(grid):
            if abs(values[index]) < 1e-10:
                roots.append(float(h))
            if index and values[index-1]*values[index] < 0:
                roots.append(float(brentq(residual, grid[index-1], h)))
        return sorted({round(root, 12) for root in roots})

    def transforms(self, hip, knee):
        angles = {f'cad_{side}_{part}': angle for side in ('left', 'right')
                  for part, angle in (('hip', hip), ('knee', knee))}
        world = {'cad_body': np.eye(4)}
        pending = list(self.model.joints)
        while pending:
            ready = [j for j in pending if j[2] in world and j[3] not in world]
            if not ready:
                break
            for name, kind, parent, child, origin, axis in ready:
                motion = np.eye(4)
                if kind in ('revolute', 'continuous'):
                    motion[:3, :3] = rotation(axis, angles.get(name, 0))
                world[child] = world[parent] @ origin @ motion
            pending = [j for j in pending if j[3] not in world]
        return world

    def evaluate(self, hip, knee, check_mesh=True):
        world = self.transforms(hip, knee)
        heights = []
        for side in ('left', 'right'):
            name = f'cad_{side}_wheel_link'
            collision = self.links[name].find('collision')
            shape = collision.find('geometry/cylinder')
            transform = world[name] @ origin_transform(collision.find('origin'))
            axis_z = abs(transform[2, 2])
            support = (float(shape.get('radius'))*np.sqrt(max(0, 1-axis_z**2))
                       + float(shape.get('length'))*.5*axis_z)
            heights.append(support - transform[2, 3])
        if abs(heights[0]-heights[1]) > 1e-7:
            raise ValueError('Symmetric zero-roll pose has unequal wheel support heights')
        height = float(np.mean(heights))
        clearances = {}
        for name, link in self.links.items():
            for index, collision in enumerate(link.findall('collision')):
                box = collision.find('geometry/box')
                if box is None:
                    continue
                transform = world[name] @ origin_transform(collision.find('origin'))
                size = vector(box, 'size')
                low = height + transform[2, 3] - np.sum(abs(transform[2, :3])*size/2)
                clearances[f'{name}:{index}'] = float(low)
        geometry = self.model.evaluate([hip, knee, hip, knee])
        result = {'hip_rad': hip, 'knee_rad': knee, 'body_height_m': height,
                  'com_height_m': geometry['height'], 'neutral_residual_m': geometry['dx'],
                  'box_floor_clearances_m': clearances}
        if check_mesh:
            result['wheel_thigh_samples'] = {}
            for side in ('left', 'right'):
                thigh_name, wheel_name = f'cad_{side}_thigh', f'cad_{side}_wheel_link'
                thigh = self.meshes[thigh_name]
                wheel = self.meshes[wheel_name].copy()
                wheel.apply_transform(np.linalg.inv(world[thigh_name]) @ world[wheel_name])
                inside_wheel = vertices_inside(thigh, wheel.vertices)
                inside_thigh = vertices_inside(wheel, thigh.vertices)
                result['wheel_thigh_samples'][side] = {
                    'sampled_vertex_surface_gap_m': sampled_surface_gap(thigh, wheel),
                    'wheel_vertices_inside_thigh': inside_wheel,
                    'thigh_vertices_inside_wheel': inside_thigh,
                    'interference_confirmed': bool(inside_wheel or inside_thigh)}
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--xacro', type=Path, default=ROOT / 'src/tap_robot_description/description/cad_robot/robot.urdf.xacro')
    parser.add_argument('--knee-min', type=float, default=-2.1)
    parser.add_argument('--step', type=float, default=.1)
    parser.add_argument('--output', type=Path, help='Optional JSON report; defaults to stdout')
    args = parser.parse_args()
    audit = SquatAudit(args.xacro)
    if not (np.isfinite(args.step) and args.step > 0 and np.isfinite(args.knee_min)
            and audit.knee_limits[0] <= args.knee_min < 0 <= audit.knee_limits[1]):
        parser.error('Require positive finite step and negative knee-min within URDF bounds')
    previous = None
    rows = []
    knees = np.linspace(0, args.knee_min, int(np.ceil(abs(args.knee_min)/args.step))+1)
    for knee in knees:
        roots = audit.neutral_roots(float(knee))
        if not roots:
            rows.append({'knee_rad': float(knee), 'neutral_roots_rad': [], 'branch_lost': True})
            break
        selected = min(roots, key=lambda h: abs(h-(0 if previous is None else previous)))
        row = audit.evaluate(selected, float(knee))
        row['neutral_roots_rad'] = roots
        rows.append(row)
        previous = selected
    report = {'schema': 1, 'simulation_only': True, 'xacro': str(audit.path),
              'expanded_simulation_sha256': audit.expanded_hash,
              'mesh_sha256': audit.mesh_hashes, 'hip_urdf_limits_rad': audit.hip_limits,
              'knee_urdf_limits_rad': audit.knee_limits,
              'assumptions': ['Zero body pitch/roll; identical left/right native joint angles.',
                              'Neutral root minimizes COM-to-support-line X offset using estimated masses.',
                              'Root search brackets sign changes on 129 hip samples; tangential roots may be missed.',
                              'Branch continuation chooses root nearest the preceding sample, starting nearest zero hip.',
                              '71-point pose-curve limits and controller clamps are not applied.',
                              'Floor clearance covers collision boxes, not all CAD surfaces.',
                              'Vertex-to-triangle gap is an upper bound on true mesh separation, not a clearance guarantee.',
                              'Inside vertices confirm overlap; their absence does not exclude triangle-only intersection.',
                              'Only wheel/thigh mesh pairs checked; not a full self-collision or continuous-motion certificate.',
                              'Encoder 30..145 degrees gives travel span only; CAD offset and sign remain unknown.'],
              'samples': rows}
    content = json.dumps(report, indent=2, allow_nan=False) + '\n'
    if args.output:
        args.output.write_text(content)
        print(f'Wrote {len(rows)} samples to {args.output}')
    else:
        print(content, end='')


if __name__ == '__main__':
    main()
