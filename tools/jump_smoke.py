#!/usr/bin/env python3
"""Isolated, contact-gated world-COM jump evidence; never applies forces/resets.

Interface defaults are provisional until integration: two stamped
ros_gz_interfaces/Contacts streams (including empty airborne heartbeats),
/jump_request Bool rising edge, /jump_status String phase or phase|key=value.
Override topics/phases/launch arguments explicitly for the integrated contract.
"""
import argparse
from collections import defaultdict
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import uuid
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / 'tools') not in sys.path:
    sys.path.insert(0, str(ROOT / 'tools'))
from ros_parameters import canonical_ros_parameters

HARNESS_BYTES = Path(__file__).read_bytes()


def jump_model_arguments(speed, damping, extra_launch=()):
    if not math.isfinite(damping) or damping < 0:
        raise ValueError('jump_leg_velocity_damping must be finite and >=0')
    if any(v.startswith(('gui:=', 'auto_enable:=', 'controller_config:=', 'jump_model:=',
                         'jump_leg_speed:=', 'jump_leg_velocity_damping:=')) for v in extra_launch):
        raise ValueError('Owned launch arguments cannot be overridden')
    return ['jump_model:=true', f'jump_leg_speed:={speed}',
            f'jump_leg_velocity_damping:={damping}']


def phase_value(text, key=None):
    parts = text.split('|')
    if key:
        fields = dict(p.split('=', 1) for p in parts if '=' in p)
        if key not in fields:
            raise ValueError(f'Missing phase key {key}')
        return fields[key]
    return parts[0]


def joint_map(names, values, required):
    if len(names) != len(set(names)) or len(names) != len(values):
        raise ValueError('Duplicate names or incomplete joint values')
    result = dict(zip(names, values))
    if not set(required) <= result.keys():
        raise ValueError('Missing movable joints')
    if not all(math.isfinite(v) for v in result.values()):
        raise ValueError('Nonfinite joints')
    return result


def stale_sources(cache, now, wall, freshness, contact_freshness):
    return [k for k, item in cache.items() if item['stamp'] is None or
            now-item['stamp'] > (contact_freshness if k in ('left', 'right') else freshness)
            or wall-item['wall'] > 2.]


def request_level(now, request_time, baseline_only=False):
    return not baseline_only and request_time is not None and 0 <= now-request_time < .1


def check_applied_efforts(efforts, required):
    if efforts is None or not set(required) <= efforts.keys():
        raise ValueError('Applied actuator effort evidence missing')
    for name in required:
        cap = .13 if name in ('cad_left_wheel', 'cad_right_wheel') else 5.
        if not math.isfinite(efforts[name]) or abs(efforts[name]) > cap+1e-6:
            raise ValueError(f'Applied torque exceeds physical cap: {name}={efforts[name]}, cap={cap}')


def hard_pose_guard(height, pitch):
    if not all(math.isfinite(v) for v in (height, pitch)) or height < .115 or abs(pitch) > .5:
        raise RuntimeError(f'Hard pose guard: body_z={height:.5f}, pitch={pitch:.5f}')


def clock_progress_guard(stage, wall, clock_wall, launch_wall, startup_timeout):
    if stage == 'startup':
        if wall-launch_wall > startup_timeout:
            raise RuntimeError('Startup wall timeout awaiting activation/coherent inputs')
    elif wall-clock_wall > 2.:
        raise RuntimeError('Clock stopped')


class SettlingWindow:
    def __init__(self, duration=8., stable_duration=1.):
        self.duration, self.stable_duration = duration, stable_duration
        self.start = self.stable_since = None
        self.transients = []
        self.passed = False

    def update(self, row):
        t = row['sim']
        if self.start is None:
            self.start = t
        reasons = []
        if not all(row['contacts']) or any(row.get('contact_air_since_previous', [])):
            reasons.append('contact')
        if abs(row['pitch']) >= .15:
            reasons.append('pitch')
        if row.get('jump_ready') != 1:
            reasons.append('ready')
        if reasons:
            self.stable_since = None
            self.transients.append({'sim': t, 'reasons': reasons})
        elif self.stable_since is None:
            self.stable_since = t
        if t-self.start < self.duration:
            return False
        self.passed = self.stable_since is not None and t-self.stable_since >= self.stable_duration
        if not self.passed:
            raise RuntimeError('Settling window ended without continuous 1s grounded/pitch<.15/ready=1')
        return True


def startup_diagnostics(events):
    result = {'contact_packets': {'left': 0, 'right': 0}, 'gate_transitions': [],
              'first_true_previous_false_wall_gap_s': None,
              'max_gate_wall_gap_s': 0., 'peak_leg_command_nm': [0.]*4,
              'peak_applied_effort_nm': {}, 'last_pose': None, 'last_joint_positions': None}
    previous_gate = None
    first_true_seen = False
    for event in events:
        if event.get('stage') != 'startup':
            continue
        kind, data = event['kind'], event['data']
        if kind == 'actuator_enable':
            if previous_gate is not None:
                gap = event['wall']-previous_gate['wall']
                result['max_gate_wall_gap_s'] = max(result['max_gate_wall_gap_s'], gap)
                if data and not first_true_seen and not previous_gate['data']:
                    result['first_true_previous_false_wall_gap_s'] = gap
            if previous_gate is None or previous_gate['data'] != data:
                result['gate_transitions'].append({'sim': event['sim'], 'wall': event['wall'], 'value': data})
            first_true_seen = first_true_seen or data
            previous_gate = event
        elif kind in ('left', 'right'):
            result['contact_packets'][kind] += 1
        elif kind == 'legs':
            result['peak_leg_command_nm'] = [max(a, abs(b)) for a, b in
                                            zip(result['peak_leg_command_nm'], data['data'])]
        elif kind == 'joints':
            measured = data['data']
            result['last_joint_positions'] = measured['positions']
            for name, value in (measured['efforts'] or {}).items():
                result['peak_applied_effort_nm'][name] = max(
                    result['peak_applied_effort_nm'].get(name, 0.), abs(value))
        elif kind == 'pose':
            result['last_pose'] = {'sim': event['sim'], 'position_quaternion': data['data']}
    return result


def wheel_contact(msg, wheel_token, ground_token):
    """Reject an incorrectly routed sensor instead of interpreting it as flight."""
    ground = False
    for contact in msg.contacts:
        a, b = contact.collision1.name, contact.collision2.name
        if wheel_token not in a and wheel_token not in b:
            raise ValueError(f'Contact packet does not identify wheel {wheel_token}')
        other = b if wheel_token in a else a
        if ground_token not in other:
            raise ValueError(f'Unexpected non-ground wheel contact: {other}')
        ground = True
    return ground


class WorldCOM:
    """Use existing parsed URDF metadata, but do NOT level the body or zero wheels."""
    def __init__(self, geometry):
        self.geometry = geometry
        self.required = [j[0] for j in geometry.joints if j[1] != 'fixed']

    def evaluate(self, position, quaternion, angles):
        import numpy as np
        sys.path.insert(0, str(ROOT / 'src/tap_teleop'))
        from tap_teleop.pose_curves import rotation
        if not set(self.required) <= angles.keys():
            raise ValueError('Missing movable joint for COM')
        values = list(position) + list(quaternion) + list(angles.values())
        if not all(math.isfinite(v) for v in values):
            raise ValueError('Nonfinite world COM input')
        x, y, z, w = quaternion
        if abs(x*x+y*y+z*z+w*w-1) > .001:
            raise ValueError('Invalid pose quaternion')
        base = np.eye(4)
        base[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                           [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                           [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
        base[:3, 3] = position
        frames = {'cad_body': base}
        pending = list(self.geometry.joints)
        while pending:
            available = [j for j in pending if j[2] in frames and j[3] not in frames]
            if not available:
                break
            for name, kind, parent, child, origin, axis in available:
                motion = np.eye(4)
                if kind in ('continuous', 'revolute'):
                    motion[:3, :3] = rotation(axis, angles[name])
                elif kind == 'prismatic':
                    motion[:3, 3] = axis * angles[name]
                elif kind != 'fixed':
                    raise ValueError(f'Unsupported joint {kind}')
                frames[child] = frames[parent] @ origin @ motion
            pending = [j for j in pending if j[3] not in frames]
        if not self.geometry.links.keys() <= frames.keys():
            raise ValueError('Mass outside cad_body subtree')
        com = sum(m*(frames[n] @ np.r_[c, 1])[:3]
                  for n, (m, c) in self.geometry.links.items()) / self.geometry.mass
        return com.tolist()


class FlightEvidence:
    def __init__(self, minimum_air=.03, required_height=.10):
        self.minimum_air = minimum_air
        self.required_height = required_height
        self.previous = self.candidate = self.takeoff = self.landing = None
        self.last_ground_sample = None
        self.apex = None
        self.air_duration = 0.
        self.height_pass = False

    def update(self, row):
        t = row['sim']
        grounded = any(row['contacts']) or any(row.get('contact_ground_since_previous', []))
        if any(row['contacts']):
            self.last_ground_sample = row
        if self.takeoff is None:
            if not grounded and self.candidate is None and self.previous is not None:
                if any(self.previous['contacts']) or any(self.previous.get('contact_ground_since_previous', [])):
                    self.candidate = {'first_air': row, 'last_ground': self.last_ground_sample or self.previous}
                    self.apex = row
            if self.candidate is not None:
                if row['com'][2] > self.apex['com'][2]:
                    self.apex = row
                if grounded:
                    self.candidate = self.apex = None
                elif min(row.get('contact_stamps', [t]))-max(self.candidate['first_air'].get(
                        'contact_stamps', [self.candidate['first_air']['sim']])) >= self.minimum_air:
                    self.takeoff = self.candidate
        if self.takeoff is not None and self.landing is None:
            if grounded:
                self.landing = row
            else:
                self.air_duration = t-self.takeoff['first_air']['sim']
                if row['com'][2] > self.apex['com'][2]:
                    self.apex = row
                # Conservative sampled boundary, no credit for pre-takeoff extension.
                reference = max(self.takeoff[k]['com'][2] for k in ('first_air', 'last_ground'))
                self.height_pass = self.apex['com'][2]-reference > self.required_height
        self.previous = row

    def summary(self):
        if self.takeoff is None:
            return {'height_pass': False, 'takeoff_detected': False}
        a, b = self.takeoff['last_ground'], self.takeoff['first_air']
        reference = max(a['com'][2], b['com'][2])
        return {'takeoff_detected': True, 'takeoff_bracket_s': [a['sim'], b['sim']],
                'takeoff_bracket_width_s': b['sim']-a['sim'],
                'takeoff_com_z_bracket_m': [a['com'][2], b['com'][2]],
                'takeoff_com_bracket_span_m': abs(a['com'][2]-b['com'][2]),
                'conservative_takeoff_com_z_m': reference,
                'apex_com_z_m': self.apex['com'][2], 'apex_sim_s': self.apex['sim'],
                'height_m': self.apex['com'][2]-reference, 'height_pass': self.height_pass,
                'both_airborne_s': self.air_duration,
                'landing_sim_s': self.landing['sim'] if self.landing else None}


def recovery_metrics(rows, anchor, max_xy=.05, max_residual=.02):
    if any(not math.isfinite(v) or v <= 0 for v in (max_xy, max_residual)):
        raise ValueError('Recovery drift bounds must be finite and positive')
    if not rows:
        return {'passed': False, 'reason': 'No completed recovery hold'}
    distance = lambda row: math.dist(row['com'][:2], anchor['com'][:2])
    peak, final = max(map(distance, rows)), distance(rows[-1])
    pitch = max(abs(r['pitch']) for r in rows)
    return {'peak_xy_m': peak, 'final_xy_m': final, 'max_absolute_pitch_rad': pitch,
            'duration_s': rows[-1]['sim']-rows[0]['sim'],
            'passed': peak <= max_xy and final <= max_residual and pitch <= .2
            and all(all(r['contacts']) for r in rows)}


def saturation_metrics(commands, limits):
    """Time-weighted held commands, grouped by recorded jump phase."""
    result = {}
    for i, limit in enumerate(limits):
        if not math.isfinite(limit) or limit <= 0:
            raise ValueError('Positive physical torque limit required')
        duration = saturated = longest = run = 0.
        phases = defaultdict(lambda: {'duration_s': 0., 'saturated_s': 0.})
        for a, b in zip(commands, commands[1:]):
            dt = b['sim']-a['sim']
            if dt < 0:
                raise ValueError('Command time reversed')
            hit = abs(a['values'][i]) >= limit*(1-1e-6)
            duration += dt
            saturated += dt*hit
            run = run+dt if hit else 0.
            longest = max(longest, run)
            phases[a['phase']]['duration_s'] += dt
            phases[a['phase']]['saturated_s'] += dt*hit
        result[str(i)] = {'limit_nm': limit, 'saturated_s': saturated,
                          'longest_s': longest, 'fraction': saturated/duration if duration else 0.,
                          'phases': dict(phases)}
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain', type=int, default=117)
    parser.add_argument('--config', type=Path, default=ROOT/'src/tap_control/config/jump.yaml')
    parser.add_argument('--launch-arg', action='append', default=[])
    parser.add_argument('--jump-leg-speed', type=float, default=12.566370614359172)
    parser.add_argument('--jump-leg-velocity-damping', type=float, default=0.)
    parser.add_argument('--baseline-only', action='store_true',
                        help='Bounded-actuator stationary hold; never sends a jump request')
    parser.add_argument('--baseline-hold', type=float, default=15.)
    parser.add_argument('--max-xy', type=float, default=.05)
    parser.add_argument('--max-residual', type=float, default=.02)
    parser.add_argument('--startup-wall-timeout', type=float, default=70.)
    parser.add_argument('--left-contact-topic', default='/left_wheel_contacts')
    parser.add_argument('--right-contact-topic', default='/right_wheel_contacts')
    parser.add_argument('--left-wheel-token', default='cad_left_wheel')
    parser.add_argument('--right-wheel-token', default='cad_right_wheel')
    parser.add_argument('--ground-token', default='ground_plane')
    parser.add_argument('--pose-topic', default='/tap_robot/pose')
    parser.add_argument('--request-topic', default='/jump_request')
    parser.add_argument('--jump-status-topic', default='/jump_status')
    parser.add_argument('--phase-key', default='phase')
    parser.add_argument('--recovered-phases', default='Idle,Complete')
    parser.add_argument('--fault-phases', default='Fault,Failed,Aborted')
    parser.add_argument('--settle', type=float, default=8.)
    parser.add_argument('--timeout', type=float, default=20.)
    parser.add_argument('--recovery-hold', type=float, default=5.)
    parser.add_argument('--freshness', type=float, default=.15)
    parser.add_argument('--contact-freshness', type=float, default=.05)
    parser.add_argument('--coherence', type=float, default=.02)
    parser.add_argument('--minimum-air', type=float, default=.03)
    parser.add_argument('--flight-pitch-stop', type=float, default=.5)
    args = parser.parse_args()
    if not 1 <= args.domain <= 232:
        parser.error('Use isolated domain 1..232')
    for name in ('settle', 'timeout', 'recovery_hold', 'freshness', 'contact_freshness',
                 'coherence', 'minimum_air', 'flight_pitch_stop', 'jump_leg_speed', 'baseline_hold',
                 'startup_wall_timeout', 'max_xy', 'max_residual'):
        if not math.isfinite(getattr(args, name)) or getattr(args, name) <= 0:
            parser.error(f'{name} must be finite and positive')
    try:
        model_args = jump_model_arguments(args.jump_leg_speed, args.jump_leg_velocity_damping, args.launch_arg)
    except ValueError as exc:
        parser.error(str(exc))

    import rclpy
    import yaml
    from geometry_msgs.msg import Twist
    from rosgraph_msgs.msg import Clock
    from ros_gz_interfaces.msg import Contacts
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, Float64MultiArray, String
    from tf2_msgs.msg import TFMessage
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from owned_process import cleanup_group, start_owned_process
    from tap_launch import sim_launch_command
    sys.path.insert(0, str(ROOT/'src/tap_teleop'))
    from tap_teleop.pose_curves import RobotGeometry

    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    os.environ['GZ_PARTITION'] = f'tap_jump_{os.getpid()}_{uuid.uuid4().hex[:10]}'
    out = ROOT/'log'/f'jump_{time.time_ns()}'
    out.mkdir(parents=True)
    hashes = {}

    def snapshot(path, name):
        data = path.read_bytes()
        (out/name).write_bytes(data)
        hashes[name] = hashlib.sha256(data).hexdigest()
        return data

    (out/'jump_smoke.py').write_bytes(HARNESS_BYTES)
    hashes['jump_smoke.py'] = hashlib.sha256(HARNESS_BYTES).hexdigest()
    base = snapshot(ROOT/'src/tap_control/config/balance_controller.yaml', 'base.yaml')
    override = snapshot(args.config.resolve(), 'override.yaml')
    effective = canonical_ros_parameters(yaml.safe_load(base), yaml.safe_load(override))
    effective_bytes = yaml.safe_dump(effective, sort_keys=True).encode()
    (out/'effective.yaml').write_bytes(effective_bytes)
    hashes['effective.yaml'] = hashlib.sha256(effective_bytes).hexdigest()
    binary = ROOT/'install/tap_control/lib/tap_control/balance_controller_node'
    hashes['binary'] = hashlib.sha256(snapshot(binary, 'balance_controller_node')).hexdigest()
    model = ROOT/'src/tap_robot_description/description/cad_robot'
    for name in ('robot.urdf.xacro', 'simulation.urdf.xacro', 'masses.yaml', 'mesh_inertials.yaml'):
        snapshot(model/name, name)
    for path, name in ((ROOT/'src/tap_sim/launch/sim.launch.py', 'sim.launch.py'),
                       (ROOT/'src/tap_control/core/include/robot_control/JumpPlan.hpp', 'JumpPlan.hpp'),
                       (ROOT/'src/tap_sim/worlds/jump.sdf', 'jump.sdf'),
                       (ROOT/'src/tap_sim/src/bounded_gazebo_system.cpp', 'bounded_gazebo_system.cpp'),
                       (ROOT/'src/tap_sim/src/contact_snapshot.cpp', 'contact_snapshot.cpp'),
                       (ROOT/'src/tap_sim/include/tap_sim/actuator_bounds.hpp', 'actuator_bounds.hpp'),
                       (ROOT/'src/tap_sim/include/tap_sim/actuator_gate.hpp', 'actuator_gate.hpp'),
                       (ROOT/'src/tap_sim/include/tap_sim/leg_velocity_reference.hpp', 'leg_velocity_reference.hpp'),
                       (ROOT/'src/tap_control/core/src/JumpController.cpp', 'JumpController.cpp'),
                       (ROOT/'src/tap_control/core/include/robot_control/JumpController.hpp', 'JumpController.hpp'),
                       (ROOT/'src/tap_control/core/include/robot_control/WheelFrame.hpp', 'WheelFrame.hpp'),
                       (ROOT/'src/tap_control/ros2_nodes/BalanceControllerNode.cpp', 'BalanceControllerNode.cpp'),
                       (ROOT/'tools/transformation_smoke.py', 'transformation_smoke.py'),
                       (ROOT/'tools/owned_process.py', 'owned_process.py'),
                       (ROOT/'tools/ros_parameters.py', 'ros_parameters.py'),
                       (ROOT/'src/tap_teleop/tap_teleop/pose_curves.py', 'pose_curves.py')):
        snapshot(path, name)
    for path in (ROOT/'src/tap_control/core/src/WheelFrame.cpp',
                 ROOT/'src/tap_sim/src/leg_velocity_reference.cpp'):
        if path.exists():
            snapshot(path, path.name)
    for path in (ROOT/'install/tap_sim/lib').glob('*.so'):
        hashes[str(path.relative_to(ROOT))] = hashlib.sha256(path.read_bytes()).hexdigest()
    snapshot(ROOT/'install/tap_sim/lib/libtap_contact_snapshot.so', 'libtap_contact_snapshot.so')
    xml = subprocess.run(['xacro', str(model/'simulation.urdf.xacro'), *model_args],
                         capture_output=True, text=True, check=True).stdout
    (out/'expanded.urdf').write_text(xml)
    hashes['expanded.urdf'] = hashlib.sha256(xml.encode()).hexdigest()
    geometry = RobotGeometry(out/'expanded.urdf')
    fk = WorldCOM(geometry)
    physical_limits = {j.get('name'): dict(j.find('limit').attrib)
                       for j in ET.fromstring(xml).findall('joint') if j.find('limit') is not None}
    leg_order = ('cad_left_hip', 'cad_left_knee', 'cad_right_hip', 'cad_right_knee')
    leg_limits = [float(physical_limits[n]['effort']) for n in leg_order]
    (out/'manifest.json').write_text(json.dumps({
        'hashes': hashes, 'arguments': {k: str(v) if isinstance(v, Path) else v
                                      for k, v in vars(args).items()},
        'partition': os.environ['GZ_PARTITION']}, indent=2, allow_nan=False))
    trace = (out/'events.jsonl').open('w')
    launch_log = (out/'launch.log').open('w')
    cache, rows, recovery_rows = {}, [], []
    contact_ground_seen = {'left': False, 'right': False}
    contact_air_seen = {'left': False, 'right': False}
    settling = SettlingWindow(args.settle)
    command_rows = []
    failure = None
    process = None
    cleanup_result = None
    now = None
    clock_wall = time.monotonic()
    callback_error = None
    stage = 'startup'
    flight = FlightEvidence(args.minimum_air)
    recovery = {'passed': False, 'reason': 'Not attempted'}
    estop = {'passed': False}
    anchor = None
    request_time = None
    ready_time = None
    recovery_start = None
    last_command = last_sample = None
    incoherent_since = None
    recovered = set(args.recovered_phases.split(','))
    faults = set(args.fault_phases.split(','))
    # Keep the context alive through Ctrl-C so e-stop and evidence finalization run.
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('tap_jump_smoke')

    def event(kind, data):
        trace.write(json.dumps({'kind': kind, 'sim': now, 'wall': time.monotonic(),
                                'stage': stage, 'data': data}, allow_nan=False)+'\n')

    def stamp(header):
        return header.stamp.sec + header.stamp.nanosec*1e-9

    def store(key, data, source=None):
        cache[key] = {'data': data, 'stamp': source if source is not None else now,
                      'wall': time.monotonic(), 'source_stamp': source}
        event(key, cache[key])

    def guarded(fn):
        def callback(msg):
            nonlocal callback_error
            try:
                fn(msg)
            except (ValueError, KeyError, TypeError, IndexError) as exc:
                callback_error = f'{fn.__name__}: {exc}'
        return callback

    @guarded
    def clock(msg):
        nonlocal now, clock_wall
        value = msg.clock.sec + msg.clock.nanosec*1e-9
        if now is not None and value < now:
            raise ValueError('Clock moved backward; reset forbidden')
        if value != now:
            now, clock_wall = value, time.monotonic()

    @guarded
    def pose(msg):
        if len(msg.transforms) != 1 or msg.transforms[0].child_frame_id not in ('', 'tap_robot'):
            raise ValueError('Require dedicated model-only pose')
        t = msg.transforms[0]
        p, q = t.transform.translation, t.transform.rotation
        values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        if not all(math.isfinite(v) for v in values):
            raise ValueError('Nonfinite pose')
        store('pose', values, stamp(t.header) or None)

    @guarded
    def joints(msg):
        positions = joint_map(msg.name, msg.position, fk.required)
        velocities = joint_map(msg.name, msg.velocity, fk.required)
        efforts = joint_map(msg.name, msg.effort, fk.required) if msg.effort else None
        check_applied_efforts(efforts, fk.required)
        source = stamp(msg.header)
        if source <= 0:
            if stage == 'startup':
                return
            raise ValueError('Unstamped joints')
        store('joints', {'positions': positions, 'velocities': velocities, 'efforts': efforts}, source)

    def contact_callback(key, wheel):
        @guarded
        def receive(msg):
            source = stamp(msg.header)
            if source <= 0:
                if stage == 'startup':
                    return
                raise ValueError('Actual contact heartbeat requires source timestamp')
            grounded = wheel_contact(msg, wheel, args.ground_token)
            contact_ground_seen[key] = contact_ground_seen[key] or grounded
            contact_air_seen[key] = contact_air_seen[key] or not grounded
            store(key, grounded, source)
        return receive

    @guarded
    def control(msg):
        fields = msg.data.split('|')
        data = {'mode': fields[0], 'raw': msg.data}
        for part in fields[1:]:
            if '=' in part:
                k, v = part.split('=', 1)
                try:
                    data[k] = float(v)
                except ValueError:
                    data[k] = v
        store('control', data)

    @guarded
    def jump(msg):
        fields = dict(p.split('=', 1) for p in msg.data.split('|') if '=' in p)
        store('jump', {'phase': phase_value(msg.data, args.phase_key), 'raw': msg.data,
                       'ready': float(fields['ready']) if 'ready' in fields else None})

    def command_callback(key, length):
        @guarded
        def receive(msg):
            data = list(msg.data)
            if len(data) != length or not all(math.isfinite(x) for x in data):
                raise ValueError(f'Invalid {key} commands')
            store(key, data)
            if key == 'legs' and now is not None:
                command_rows.append({'sim': now, 'values': data, 'phase':
                                     cache.get('jump', {}).get('data', {}).get('phase', 'unknown')})
        return receive

    subscriptions = [node.create_subscription(Clock, '/clock', clock, qos_profile_sensor_data),
                     node.create_subscription(Bool, '/actuator_enable',
                                              lambda m: event('actuator_enable', bool(m.data)), 100),
                     node.create_subscription(TFMessage, args.pose_topic, pose, qos_profile_sensor_data),
                     node.create_subscription(JointState, '/joint_states', joints, qos_profile_sensor_data),
                     node.create_subscription(Contacts, args.left_contact_topic,
                         contact_callback('left', args.left_wheel_token), qos_profile_sensor_data),
                     node.create_subscription(Contacts, args.right_contact_topic,
                         contact_callback('right', args.right_wheel_token), qos_profile_sensor_data),
                     node.create_subscription(String, '/control_mode', control, 100),
                     node.create_subscription(String, args.jump_status_topic, jump, 100),
                     node.create_subscription(Float64MultiArray, '/leg_effort_controller/commands',
                                              command_callback('legs', 4), 100),
                     node.create_subscription(Float64MultiArray, '/wheel_effort_controller/commands',
                                              command_callback('wheels', 2), 100)]
    drive = node.create_publisher(Twist, '/cmd_vel', 10)
    request = node.create_publisher(Bool, args.request_topic, 10)
    stop = node.create_publisher(Bool, '/emergency_stop', 10)

    try:
        until = time.monotonic()+1.0
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.05)
        if cache or now is not None:
            raise RuntimeError('Domain already has traffic; refusing to launch or actuate')
        launch = sim_launch_command('gui:=false', 'auto_enable:=true',
                  f'controller_config:={args.config.resolve()}', *model_args, *args.launch_arg)
        event('launch', launch)
        launch_wall = time.monotonic()
        process = start_owned_process(launch, cwd=ROOT, stdout=launch_log,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        print(f'TRIAL LAUNCHED pid={process.pid} domain={args.domain} binary={hashes["binary"]}', flush=True)
        deadline = time.monotonic()+180
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.002)
            if callback_error:
                raise RuntimeError(callback_error)
            if process.poll() is not None:
                raise RuntimeError('Owned launch exited')
            clock_progress_guard(stage, time.monotonic(), clock_wall, launch_wall, args.startup_wall_timeout)
            if now is None:
                continue
            if 'pose' in cache:
                p = cache['pose']['data']
                x, y, z, w = p[3:]
                pitch = math.asin(max(-1., min(1., 2*(w*y-z*x))))
                hard_pose_guard(p[2], pitch)
                if stage == 'startup' and now > 5.:
                    raise RuntimeError('Startup did not reach coherent grounded balance within 5 sim seconds')
            if last_command is None or now-last_command >= .05-1e-9:
                drive.publish(Twist())
                request.publish(Bool(data=request_level(now, request_time, args.baseline_only)))
                last_command = now
            required = ('pose', 'joints', 'left', 'right', 'control', 'jump', 'legs', 'wheels')
            if any(k not in cache or cache[k]['stamp'] is None for k in required):
                if stage != 'startup':
                    raise RuntimeError('Missing required data')
                continue
            ages = {k: now-cache[k]['stamp'] for k in required}
            stale = stale_sources(cache, now, time.monotonic(), args.freshness, args.contact_freshness)
            if stale:
                if stage == 'startup':
                    continue
                raise RuntimeError(f'Stale inputs: {stale}, ages={ages}')
            stamps = [cache[k]['stamp'] for k in ('pose', 'joints', 'left', 'right')]
            # Never use future contact state to label an earlier COM observation.
            contact_ahead = any(cache[k]['stamp'] > cache['pose']['stamp'] for k in ('left', 'right'))
            if min(ages.values()) < -.002 or max(stamps)-min(stamps) > args.coherence or contact_ahead:
                incoherent_since = incoherent_since if incoherent_since is not None else now
                if now-incoherent_since > args.freshness and stage != 'startup':
                    raise RuntimeError('Persistent incoherent source timestamps')
                continue
            incoherent_since = None
            source_time = cache['pose']['stamp']
            if last_sample is not None and source_time <= last_sample:
                continue
            last_sample = source_time
            p, j = cache['pose']['data'], cache['joints']['data']
            c, phase = cache['control']['data'], cache['jump']['data']['phase']
            x, y, z, w = p[3:]
            pitch = math.asin(max(-1., min(1., 2*(w*y-z*x))))
            row = {'sim': source_time, 'stage': stage, 'com': fk.evaluate(p[:3], p[3:], j['positions']),
                   'body_position': p[:3], 'quaternion': p[3:], 'pitch': pitch,
                   'joints': j, 'contacts': [cache[k]['data'] for k in ('left', 'right')],
                   'contact_stamps': [cache[k]['stamp'] for k in ('left', 'right')],
                   'contact_ground_since_previous': [contact_ground_seen[k] for k in ('left', 'right')],
                   'contact_air_since_previous': [contact_air_seen[k] for k in ('left', 'right')],
                   'source_stamps': {k: cache[k]['stamp'] for k in required},
                   'measurement_stamp_spread_s': max(stamps)-min(stamps),
                   'phase': phase, 'jump_ready': cache['jump']['data']['ready'],
                   'control': c, 'leg_commands': cache['legs']['data'],
                   'wheel_commands': cache['wheels']['data']}
            rows.append(row)
            contact_ground_seen.update(left=False, right=False)
            contact_air_seen.update(left=False, right=False)
            if stage == 'startup':
                if c['mode'] in ('Balance', 'Drive') and c.get('fresh') == 1:
                    stage = 'settle'
                    settling.update(row)
                continue
            if c['mode'] in ('Fallen', 'EmergencyStop', 'Disabled') or phase in faults:
                raise RuntimeError(f'Unsafe control/phase: {c["mode"]}/{phase}')
            if c.get('fresh') != 1 or c.get('cmd_v') != 0 or c.get('cmd_w') != 0:
                raise RuntimeError('Controller stale or operator command nonzero')
            if abs(pitch) > args.flight_pitch_stop:
                raise RuntimeError('Flight attitude safety stop')
            if stage == 'settle':
                if settling.update(row):
                    if args.baseline_only:
                        anchor, recovery_start, stage = row, source_time, 'baseline'
                        recovery_rows.append(row)
                        continue
                    anchor, request_time, stage = row, now, 'jump'
                    flight.update(row)
                    request.publish(Bool(data=True))
                    event('jump_rising_edge', {'sim': now})
                continue
            if stage == 'baseline':
                if c['mode'] not in ('Balance', 'Drive') or not all(row['contacts']):
                    raise RuntimeError('Bounded-actuator baseline lost balance/contact')
                recovery_rows.append(row)
                if source_time-recovery_start >= args.baseline_hold:
                    recovery = recovery_metrics(recovery_rows, anchor, args.max_xy, args.max_residual)
                    if not recovery['passed']:
                        raise RuntimeError('Bounded-actuator baseline drift/pitch failed')
                    break
                continue
            flight.update(row)
            if now-request_time > args.timeout:
                raise RuntimeError('Jump/landing/recovery timeout')
            if flight.landing is not None:
                if all(row['contacts']) and phase in recovered and c['mode'] in ('Balance', 'Drive'):
                    if recovery_start is None:
                        recovery_start, stage = source_time, 'recovery'
                    recovery_rows.append(row)
                    if source_time-recovery_start >= args.recovery_hold:
                        recovery = recovery_metrics(recovery_rows, anchor, args.max_xy, args.max_residual)
                        if not recovery['passed']:
                            raise RuntimeError('Landing recovery bounds failed')
                        if not flight.height_pass:
                            raise RuntimeError('COM rise did not strictly exceed 0.10 m')
                        break
                elif recovery_start is not None:
                    raise RuntimeError('Lost contact/balance during recovery hold')
        else:
            raise RuntimeError('Wall timeout waiting for integrated jump contract')
    except KeyboardInterrupt:
        failure = 'Interrupted by operator; incomplete trial, not a pass'
    except Exception as exc:
        failure = str(exc)
    finally:
        if process is not None:
            stage = 'estop'
            start = time.monotonic()
            zero_since = None
            try:
                while time.monotonic()-start < 3:
                    stop.publish(Bool(data=True))
                    rclpy.spin_once(node, timeout_sec=.02)
                    fresh = all(k in cache and cache[k]['wall'] >= start and
                                time.monotonic()-cache[k]['wall'] < .2 for k in ('legs', 'wheels', 'joints'))
                    zero = fresh and all(abs(v) < 1e-9 for k in ('legs', 'wheels') for v in cache[k]['data'])
                    zero = zero and all(abs(v) < 1e-9 for v in cache['joints']['data']['efforts'].values())
                    zero_since = (zero_since if zero_since is not None else time.monotonic()) if zero else None
                    if zero_since is not None and time.monotonic()-zero_since >= .2:
                        estop = {'passed': True, 'sustained_zero_wall_s': .2,
                                 'checks': 'fresh command targets AND applied actuator efforts'}
                        break
            except Exception as exc:
                estop['reason'] = str(exc)
            if not estop['passed']:
                failure = failure or 'E-stop did not sustain fresh zero commands'
        try:
            cleanup_result = cleanup_group(process)
        except Exception as exc:
            failure = failure or f'Cleanup failed: {exc}'
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        trace.close()
        launch_log.close()
        # Offline only: scanning entire traces during flight can starve callbacks.
        with (out/'events.jsonl').open() as source:
            startup = startup_diagnostics(json.loads(line) for line in source)
        limits = effective['balance_controller_node']['ros__parameters']
        applied_by_phase = {}
        applied_samples = [{'sim': r['sim'], 'phase': r['phase'],
                            'values': [r['joints']['efforts'][n] for n in fk.required]} for r in rows]
        applied_saturation = saturation_metrics(applied_samples,
            [.13 if n in ('cad_left_wheel', 'cad_right_wheel') else 5. for n in fk.required])
        for phase in sorted({r['phase'] for r in rows}):
            selected = [r for r in rows if r['phase'] == phase]
            applied_by_phase[phase] = {
                'peak_applied_nm': {n: max(abs(r['joints']['efforts'][n]) for r in selected) for n in fk.required},
                'physical_cap_saturation': {n: applied_saturation[str(i)]['phases'].get(phase,
                    {'duration_s': 0., 'saturated_s': 0.}) for i, n in enumerate(fk.required)}}
        impact = None
        if flight.landing is not None:
            t = flight.landing['sim']
            velocities = [(b['sim'], (b['com'][2]-a['com'][2])/(b['sim']-a['sim']))
                          for a, b in zip(rows, rows[1:]) if b['sim'] > a['sim']]
            before = [v for ts, v in velocities if t-.04 <= ts < t]
            after = [v for ts, v in velocities if t <= ts <= t+.04]
            if before and after:
                impact = {'pre_vz_m_s': min(before), 'post_vz_m_s': max(after),
                          'estimated_vertical_impulse_ns': geometry.mass*(max(after)-min(before)),
                          'method': 'COM finite differences within +/-40ms; not contact force measurement'}
        summary = {'failure': failure, 'hashes': hashes, 'arguments': {k: str(v) if isinstance(v, Path) else v
                   for k, v in vars(args).items()}, 'partition': os.environ['GZ_PARTITION'],
                   'flight': flight.summary(), 'recovery': recovery, 'estop': estop, 'cleanup': cleanup_result,
                   'startup_diagnostics': startup,
                   'settling': {'duration_s': args.settle, 'stable_duration_s': 1.,
                                'passed': settling.passed, 'transients': settling.transients,
                                'no_jump_height_credit': True},
                   'landing_pass': recovery['passed'] and not args.baseline_only, 'impact_estimate': impact,
                   'recovery_criteria': {'max_xy_m': args.max_xy, 'max_residual_m': args.max_residual,
                                         'max_absolute_pitch_rad': .2, 'both_contacts_required': True,
                                         'anchor': 'fixed pre-request whole-robot world COM XY'},
                   'baseline_only': args.baseline_only,
                   'mass_kg': geometry.mass, 'effective_controller_parameters': limits,
                   'effective_parameter_basis': 'Canonical dotted YAML names, base then override; not a live service dump',
                   'urdf_joint_limits': physical_limits, 'leg_command_order': leg_order,
                   'applied_effort_semantics': 'Bounded plugin applied torque, excludes passive/contact forces',
                   'wheel_command_units': 'rad/s (velocity target), NOT Nm',
                   'applied_effort_joint_order': fk.required, 'applied_effort_by_phase': applied_by_phase,
                   'applied_effort_saturation': applied_saturation,
                   'max_measurement_stamp_spread_s': max((r['measurement_stamp_spread_s'] for r in rows), default=None),
                   'saturation': saturation_metrics(command_rows, leg_limits),
                   'joint_measurements': {n: {
                       'max_absolute_velocity_rad_s': max((abs(r['joints']['velocities'][n]) for r in rows), default=0.),
                       'max_absolute_reported_effort_nm': max((abs(r['joints']['efforts'][n]) for r in rows
                                                              if r['joints']['efforts'] is not None), default=None)}
                       for n in fk.required},
                   'samples': len(rows), 'peak_leg_command_nm': max((max(map(abs, r['leg_commands']))
                                                                               for r in rows), default=0),
                   'pose_zero_stamp_policy': 'receipt /clock; source stamps otherwise required',
                   'limits': 'Simulation only; no hardware, reset, external force or body-height flight inference'}
        (out/'samples.json').write_text(json.dumps(rows, allow_nan=False))
        (out/'summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False))
        print(json.dumps(summary, indent=2), flush=True)
        print('Evidence:', out, flush=True)
    return int(failure is not None)


if __name__ == '__main__':
    raise SystemExit(main())
