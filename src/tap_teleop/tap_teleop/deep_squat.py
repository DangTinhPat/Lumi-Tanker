"""Optional symmetric squat geometry, not a physical depth approval.

No existing manual, roll, or show defaults are changed. knee_max is a positive
depth in radians: native knees run from zero to -knee_max, both with +Y axes.
Callers must select a physically verified depth and controller envelope.
URDF bounds are provisional native CAD limits, not calibrated hardware limits.
The reported encoder range 30..145 degrees is not mapped without its CAD offset.
"""
import math
import subprocess
import xml.etree.ElementTree as ET

import numpy as np

from .pose_curves import JOINTS, RobotGeometry, sample
from .show_sequence import quintic


class DeepSquatTrajectory:
    """Coordinated quintic amplitude plans on a uniform neutral-curve table.

    Retargeting starts at the current amplitude, preserving position only.
    Table knots and retargeting prevent a global acceleration guarantee.
    Always starts at amplitude zero; dt is caller-provided simulation time.
    """

    def __init__(self, profile, joint_speed=.3, minimum_transition_s=11.):
        table = np.asarray(profile['table'], dtype=float)
        if table.ndim != 2 or table.shape[1] != 4 or len(table) < 2 or not np.isfinite(table).all():
            raise ValueError('Expected a finite Nx4 uniform curve table')
        if not math.isfinite(joint_speed) or joint_speed <= 0:
            raise ValueError('Joint speed must be positive and finite')
        if not math.isfinite(minimum_transition_s) or minimum_transition_s < 0:
            raise ValueError('Minimum transition time must be finite and nonnegative')
        self.max_joint_path_derivative = float(np.abs(np.diff(table, axis=0)).max() * (len(table)-1))
        if self.max_joint_path_derivative <= 0:
            raise ValueError('Curve has no joint travel')
        self.amplitude = 0.
        self.joint_speed = joint_speed
        self.minimum_transition_s = max(11., minimum_transition_s)
        self.requested_amplitude = 0.
        self.start_amplitude = 0.
        self.elapsed_s = 0.
        self.duration_s = .5
        self._curve = {'tables': {'deep': table.tolist()}}

    def advance(self, amplitude, dt):
        amplitude, dt = float(amplitude), float(dt)
        if not math.isfinite(amplitude) or not 0 <= amplitude <= 1:
            raise ValueError('Amplitude must be finite and in [0, 1]')
        if not math.isfinite(dt) or dt < 0:
            raise ValueError('dt must be finite and nonnegative')
        if amplitude != self.requested_amplitude:
            self.start_amplitude = self.amplitude
            self.requested_amplitude = amplitude
            delta = abs(amplitude-self.start_amplitude)
            self.duration_s = max(.5, self.minimum_transition_s * delta,
                                  1.875 * self.max_joint_path_derivative * delta / self.joint_speed)
            self.elapsed_s = 0.
        self.elapsed_s = min(self.duration_s, self.elapsed_s + dt)
        if self.elapsed_s >= self.duration_s - 1e-12:
            self.elapsed_s = self.duration_s
            self.amplitude = self.requested_amplitude
        else:
            self.amplitude = self.start_amplitude + (
                self.requested_amplitude-self.start_amplitude) * quintic(self.elapsed_s/self.duration_s)
        return sample(self._curve, 'deep', self.amplitude)


def _bounds(values):
    if len(values) != 2:
        raise ValueError('Bounds require two values')
    lower, upper = map(float, values)
    if not all(map(math.isfinite, (lower, upper))) or lower >= upper:
        raise ValueError('Bounds must be finite and strictly increasing')
    return lower, upper


def _root(fn, lower, upper):
    """Require a unique sampled crossing, then bisect and verify its residual."""
    xs = np.linspace(lower, upper, 65)
    ys = [float(fn(x)) for x in xs]
    if not all(map(math.isfinite, ys)):
        raise ValueError('Nonfinite CoM residual')
    candidates = [(x, x) for x, y in zip(xs, ys) if abs(y) <= 1e-10]
    candidates += [(xs[i], xs[i+1]) for i in range(len(xs)-1)
                   if abs(ys[i]) > 1e-10 and abs(ys[i+1]) > 1e-10
                   and ys[i] * ys[i+1] < 0]
    if len(candidates) != 1:
        raise ValueError('Expected one neutral crossing inside hip bounds')
    a, b = candidates[0]
    fa = fn(a)
    for _ in range(60):
        mid = (a + b) / 2
        fm = fn(mid)
        if not math.isfinite(fm):
            raise ValueError('Nonfinite root residual')
        if abs(fm) <= 1e-10:
            return float(mid)
        if fa * fm < 0:
            b = mid
        else:
            a, fa = mid, fm
    raise ValueError('Neutral root failed residual tolerance')


def generate_deep_squat(xacro, *, knee_max, hip_bounds=None, samples=101):
    """Return native poses, CoM heights/residuals and explicit effective bounds.

    hip_bounds defaults to the intersection of both URDF hip limits. An
    explicit envelope must lie within those limits; it is never silently
    widened or clipped. Invalid requested depths fail instead of returning a
    truncated curve. Values are SI; no simulation or ROS node is launched.
    """
    knee_max = float(knee_max)
    if not math.isfinite(knee_max) or knee_max <= 0:
        raise ValueError('knee_max must be positive and finite')
    if isinstance(samples, bool) or not isinstance(samples, int) or samples < 2:
        raise ValueError('samples must be an integer >= 2')
    xml = subprocess.run(['xacro', str(xacro)], check=True, text=True,
                         capture_output=True).stdout
    root = ET.fromstring(xml)
    limits = {}
    for name in JOINTS:
        matching = [j for j in root.findall('joint') if j.get('name') == name]
        if len(matching) != 1 or matching[0].get('type') != 'revolute':
            raise ValueError(f'Expected a bounded revolute joint: {name}')
        limit = matching[0].find('limit')
        if limit is None or 'lower' not in limit.attrib or 'upper' not in limit.attrib:
            raise ValueError(f'Missing URDF limits: {name}')
        limits[name] = _bounds([limit.get('lower'), limit.get('upper')])
    urdf_hips = (max(limits[n][0] for n in JOINTS[::2]),
                 min(limits[n][1] for n in JOINTS[::2]))
    hips = _bounds(urdf_hips if hip_bounds is None else hip_bounds)
    if hips[0] < urdf_hips[0] or hips[1] > urdf_hips[1]:
        raise ValueError('Requested hip envelope exceeds URDF joint limits')
    for name in JOINTS[1::2]:
        lo, hi = limits[name]
        if -knee_max < lo or not lo <= 0 <= hi:
            raise ValueError(f'Requested knee path exceeds URDF limits: {name}')
    geometry = RobotGeometry(xacro)
    # Reject a description changing between expansion and FK construction.
    import hashlib
    if hashlib.sha256(xml.encode()).hexdigest() != geometry.fingerprint:
        raise ValueError('URDF changed during curve generation')
    table, heights, offsets = [], [], []
    for knee in np.linspace(0., -knee_max, samples):
        def residual(hip):
            return geometry.evaluate([hip, knee, hip, knee])['dx']
        hip = _root(residual, *hips)
        pose = [hip, float(knee), hip, float(knee)]
        state = geometry.evaluate(pose)
        if not all(math.isfinite(state[key]) for key in ('height', 'dx', 'lateral_margin')):
            raise ValueError('Nonfinite FK result')
        if state['height'] <= 0 or state['lateral_margin'] <= 0:
            raise ValueError('Requested depth leaves geometric support envelope')
        if heights and state['height'] > heights[-1] + 1e-7:
            raise ValueError('Requested knee direction does not continuously shorten squat')
        table.append(pose)
        heights.append(state['height'])
        offsets.append(state['dx'])
    return {'schema': 1, 'experimental': True, 'physical_gate_passed': False,
            'coordinate_frame': 'native_cad', 'encoder_offset_calibrated': False,
            'urdf_sha256': geometry.fingerprint, 'joint_order': list(JOINTS),
            'hip_bounds': list(hips), 'knee_bounds': [-knee_max, 0.],
            'urdf_joint_limits': {n: list(v) for n, v in limits.items()},
            'rest_pose': table[0], 'table': table, 'com_height_m': heights,
            'com_offset_m': offsets, 'drop_m': heights[0] - heights[-1]}
