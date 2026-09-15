"""Generate bounded CoM-neutral trajectories from expanded URDF geometry."""
from pathlib import Path
import hashlib
import subprocess
import xml.etree.ElementTree as ET

import numpy as np

JOINTS = ('cad_left_hip', 'cad_left_knee', 'cad_right_hip', 'cad_right_knee')


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + np.sin(angle) * skew + (1 - np.cos(angle)) * skew @ skew


def vector(node, key, default=(0, 0, 0)):
    return np.array(default if node is None or node.get(key) is None
                    else [float(v) for v in node.get(key).split()], dtype=float)


class RobotGeometry:
    def __init__(self, xacro):
        xml = subprocess.run(['xacro', str(xacro)], check=True, capture_output=True,
                             text=True).stdout
        self.fingerprint = hashlib.sha256(xml.encode()).hexdigest()
        root = ET.fromstring(xml)
        self.links = {}
        for link in root.findall('link'):
            inertial = link.find('inertial')
            if inertial is not None:
                self.links[link.get('name')] = (
                    float(inertial.find('mass').get('value')),
                    vector(inertial.find('origin'), 'xyz'))
        self.mass = sum(m for m, _ in self.links.values())
        self.joints = []
        for joint in root.findall('joint'):
            origin = joint.find('origin')
            r, p, y = vector(origin, 'rpy')
            transform = np.eye(4)
            transform[:3, :3] = (rotation([0, 0, 1], y) @
                                 rotation([0, 1, 0], p) @ rotation([1, 0, 0], r))
            transform[:3, 3] = vector(origin, 'xyz')
            self.joints.append((joint.get('name'), joint.get('type'),
                                joint.find('parent').get('link'),
                                joint.find('child').get('link'), transform,
                                vector(joint.find('axis'), 'xyz', (1, 0, 0))))
        for name in JOINTS:
            joint = next(j for j in self.joints if j[0] == name)
            if not np.allclose(joint[5], [0, 1, 0]):
                raise ValueError('Native pose contract requires all +Y axes')

    def evaluate(self, pose):
        angles = dict(zip(JOINTS, pose))
        # Fix the physical body frame, ignoring the RViz ground mounting joint.
        world = {'cad_body': np.eye(4)}
        pending = list(self.joints)
        while pending:
            available = [j for j in pending if j[2] in world and j[3] not in world]
            if not available:
                break
            for name, kind, parent, child, origin, axis in available:
                motion = np.eye(4)
                if kind in ('revolute', 'continuous'):
                    motion[:3, :3] = rotation(axis, angles.get(name, 0))
                elif kind != 'fixed':
                    raise ValueError(f'Unsupported joint {name}: {kind}')
                world[child] = world[parent] @ origin @ motion
            pending = [j for j in pending if j[3] not in world]
        if not self.links.keys() <= world.keys():
            raise ValueError('Mass-bearing links outside CAD body subtree')
        com = sum(m * (world[n] @ np.r_[c, 1])[:3]
                  for n, (m, c) in self.links.items()) / self.mass
        left = world['cad_left_wheel_link'][:3, 3]
        right = world['cad_right_wheel_link'][:3, 3]
        roll = float(np.arctan2(right[2] - left[2], left[1] - right[1]))
        level = rotation([1, 0, 0], roll)
        com, left, right = level @ com, level @ left, level @ right
        axle = (left + right) / 2
        # Under asymmetric roll, the support line need not be parallel to Y.
        fraction = (com[1] - right[1]) / (left[1] - right[1])
        support_x = right[0] + fraction * (left[0] - right[0])
        return {'dx': float(com[0] - support_x),
                'height': float(com[2] - axle[2] + .03),
                'roll': roll,
                'lateral_margin': float(min(com[1] - right[1], left[1] - com[1]))}

    def neutral(self, left_knee, right_knee):
        # Common hip correction preserves native same-sign symmetry. A zero
        # constant term is deliberately NOT imposed on the resulting curve.
        def residual(hip):
            return self.evaluate([hip, left_knee, hip, right_knee])['dx']
        a, b = -.6, .6
        fa, fb = residual(a), residual(b)
        if fa * fb > 0:
            raise ValueError('No neutral hip inside conservative envelope')
        for _ in range(40):
            mid = (a + b) / 2
            fm = residual(mid)
            if abs(fm) < 1e-10:
                return [mid, left_knee, mid, right_knee]
            if fa * fm <= 0:
                b = mid
            else:
                a, fa = mid, fm
        return [mid, left_knee, mid, right_knee]


def generate(xacro):
    robot = RobotGeometry(xacro)
    rest = robot.neutral(0, 0)
    h0 = robot.evaluate(rest)['height']
    # Probe both signs instead of inheriting the old robot's bending direction.
    direction = min((-.1, .1), key=lambda k: robot.evaluate(robot.neutral(k, k))['height'])
    sign = float(np.sign(direction))
    maximum = .8
    tables = {'squat': [], 'roll_left': [], 'roll_right': []}
    for depth in np.linspace(0, maximum, 71):
        knee = sign * float(depth)
        try:
            poses = {'squat': robot.neutral(knee, knee),
                     'roll_left': robot.neutral(knee, 0),
                     'roll_right': robot.neutral(0, knee)}
        except ValueError:
            break
        if any(robot.evaluate(p)['lateral_margin'] <= .025 for p in poses.values()):
            break
        for mode, pose in poses.items():
            tables[mode].append(pose)
    if len(tables['squat']) < 2:
        raise ValueError('No usable trajectory in conservative envelope')
    return {'schema': 1, 'experimental': True, 'urdf_sha256': robot.fingerprint,
            'joint_order': list(JOINTS), 'hip_bounds': [-.6, .6],
            'knee_bounds': [-.8, .2], 'shortening_sign': sign,
            'rest_pose': rest, 'rest_com_height_m': h0,
            'squat_drop_m': h0 - robot.evaluate(tables['squat'][-1])['height'],
            'tables': tables}


def sample(data, mode, amount):
    table = np.asarray(data['tables'][mode])
    x = float(np.clip(amount, 0, 1)) * (len(table) - 1)
    i = min(int(x), len(table) - 2)
    return ((1 - (x - i)) * table[i] + (x - i) * table[i + 1]).tolist()
