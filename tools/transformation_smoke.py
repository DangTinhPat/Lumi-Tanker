#!/usr/bin/env python3
"""Fixed-anchor repeated transformation QA; runs only when explicitly invoked.

Source TAP's ROS environment first. No wheel travel command is intentional.
Measurements use /clock and independent model pose, not controller dt/odometry.
"""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid
from copy import deepcopy

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / 'tools') not in sys.path:
    sys.path.insert(0, str(ROOT / 'tools'))
from ros_parameters import canonical_ros_parameters

HARNESS_BYTES = Path(__file__).read_bytes()
COMMAND_PERIOD_S = 0.05


def model_launch_arguments(arguments):
    model, seen = [], set()
    for argument in arguments:
        name, separator, value = argument.partition(':=')
        if not separator or not name or not value or name.strip() != name:
            raise ValueError('Launch overrides require name:=value')
        if name in ('gui', 'auto_enable', 'controller_config') or name in seen:
            raise ValueError(f'Reserved or duplicate launch override: {name}')
        seen.add(name)
        if name in ('jump_model', 'jump_leg_speed', 'jump_leg_velocity_damping'):
            model.append(argument)
    return model


def append_geometry_sample(sim_time, previous_sim, source_pair, previous_pair):
    return (previous_sim is None or sim_time > previous_sim) and source_pair != previous_pair


def estop_observation_zero(cache, start_wall, wall, bounded=False,
                           start_sim=None, sim=None, freshness=.15):
    for key, count in (('legs', 4), ('wheels', 2)):
        item = cache.get(key)
        if item is None or item['wall'] < start_wall or not 0 <= wall-item['wall'] < .2:
            return False
        values = item['data']
        if len(values) != count or not all(math.isfinite(v) and abs(v) < 1e-9 for v in values):
            return False
    if not bounded:
        return True
    item = cache.get('joints')
    if item is None or item['wall'] < start_wall or not 0 <= wall-item['wall'] < .2:
        return False
    stamp = item.get('stamp')
    if stamp is None or start_sim is None or sim is None or not start_sim < stamp <= sim:
        return False
    if sim-stamp > freshness:
        return False
    efforts = item['data'].get('efforts')
    return efforts is not None and len(efforts) == 6 and all(
        math.isfinite(v) and abs(v) < 1e-9 for v in efforts)


def motion_curves(selection='all'):
    return {'all': ('squat', 'roll_left', 'roll_right'),
            'squat': ('squat',)}[selection]


def validate_deep_request(depth, motions, squat_amplitude):
    if depth is None:
        return
    if not math.isfinite(depth) or not 0 < depth <= 1.6:
        raise ValueError('Deep trial depth must be positive and <=1.6 rad; larger depths are not authorized')
    if motions != 'squat':
        raise ValueError('--deep-squat-knee requires --motions squat')
    if squat_amplitude is not None and squat_amplitude != 1:
        raise ValueError('Deep knee depth is absolute; --squat-amplitude must be omitted or 1')


def check_absolute_pitch(pitch, limit):
    if limit is not None and (not math.isfinite(pitch) or abs(pitch) > limit):
        raise RuntimeError(f'Absolute pitch guard failed: pitch={pitch}, limit={limit}')


def command_due(sim_time, last_time):
    return sim_time is not None and (last_time is None or
                                    sim_time - last_time >= COMMAND_PERIOD_S - 1e-9)


def smooth5(t):
    t = min(1.0, max(0.0, t))
    return t * t * t * (10 + t * (-15 + 6 * t))


def finite(values):
    return all(math.isfinite(v) for v in values)


def merge_config(base, override):
    result = deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = merge_config(result[key], value)
        else:
            result[key] = deepcopy(value)
    return result


def angle_delta(a, b):
    return math.remainder(a - b, 2 * math.pi)


def metrics(rows, anchor):
    if not rows or anchor is None:
        return {}
    xy = lambda a, b: math.hypot(a['x'] - b['x'], a['y'] - b['y'])
    return {
        'duration_s': rows[-1]['sim'] - rows[0]['sim'],
        'peak_xy_m': max(xy(r, anchor) for r in rows),
        'residual_xy_m': xy(rows[-1], anchor),
        'residual_dx_m': rows[-1]['x'] - anchor['x'],
        'residual_dy_m': rows[-1]['y'] - anchor['y'],
        'path_length_m': sum(xy(a, b) for a, b in zip(rows, rows[1:])),
        'peak_yaw_error_rad': max(abs(r['yaw_unwrapped'] - anchor['yaw_unwrapped']) for r in rows),
        'residual_yaw_rad': rows[-1]['yaw_unwrapped'] - anchor['yaw_unwrapped'],
        'peak_pitch_error_rad': max(abs(r['pitch_error']) for r in rows),
        'peak_absolute_pitch_rad': max(abs(r['pitch']) for r in rows),
        'peak_roll_rad': max(abs(r['roll']) for r in rows),
        'min_height_m': min(r['z'] for r in rows),
        'peak_world_speed_m_s': max(r.get('world_speed', 0) for r in rows),
        'peak_joint_error_rad': max(r['joint_error'] for r in rows),
        'final_joint_error_rad': rows[-1]['joint_error'],
        'peak_leg_command_nm': max(max(map(abs, r['leg_commands'])) for r in rows),
        'measured_effort_available': all(r['joint_efforts'] is not None for r in rows),
    }


def main():
    from owned_process import cleanup_group, start_owned_process
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain', type=int, default=113)
    parser.add_argument('--amplitude', type=float, default=.25)
    parser.add_argument('--squat-amplitude', type=float,
                        help='Separate squat amount; defaults to --amplitude')
    parser.add_argument('--cycles', type=int, default=2)
    parser.add_argument('--motions', choices=('all', 'squat'), default='all',
                        help='Select squat-only cycles or the unchanged full sequence')
    parser.add_argument('--deep-squat-knee', type=float,
                        help='Absolute symmetric knee depth in rad (<=1.6); dedicated squat-only table')
    parser.add_argument('--config', type=Path, help='Passed as controller_config:=PATH')
    parser.add_argument('--launch-arg', action='append', default=[], help='Additional name:=value launch override')
    parser.add_argument('--settle', type=float, default=8.0)
    parser.add_argument('--transition', type=float, default=3.0)
    parser.add_argument('--dwell', type=float, default=2.0)
    parser.add_argument('--max-xy', type=float, default=.04)
    parser.add_argument('--max-residual', type=float, default=.015)
    parser.add_argument('--max-yaw', type=float, default=.06)
    parser.add_argument('--max-pitch-error', type=float, default=.12)
    parser.add_argument('--max-absolute-pitch', type=float, default=None)
    parser.add_argument('--max-roll', type=float, default=.3)
    parser.add_argument('--min-height', type=float, default=.14)
    parser.add_argument('--max-joint-error', type=float, default=.08)
    parser.add_argument('--freshness', type=float, default=.15)
    parser.add_argument('--coherence', type=float, default=.05)
    args = parser.parse_args()
    try:
        model_args = model_launch_arguments(args.launch_arg)
        validate_deep_request(args.deep_squat_knee, args.motions, args.squat_amplitude)
    except ValueError as exc:
        parser.error(str(exc))
    bounded_actuators = 'jump_model:=true' in model_args
    estop_checks = 'commands+applied_effort' if bounded_actuators else 'commands_only_legacy'
    if args.deep_squat_knee is not None:
        args.squat_amplitude = 1.0
    if args.squat_amplitude is None:
        args.squat_amplitude = args.amplitude
    positive = [args.settle, args.transition, args.dwell, args.max_xy,
                args.max_residual, args.max_yaw, args.max_pitch_error,
                args.max_roll, args.min_height, args.max_joint_error,
                args.freshness, args.coherence]
    if args.max_absolute_pitch is not None:
        positive.append(args.max_absolute_pitch)
    if not finite(positive + [args.amplitude, args.squat_amplitude]) or min(positive) <= 0:
        parser.error('Times and bounds must be finite and positive')
    if not 0 <= args.amplitude <= 1 or not 0 <= args.squat_amplitude <= 1 or args.cycles < 1 or not 1 <= args.domain <= 232:
        parser.error('Require amplitude [0,1], cycles >=1, domain 1..232')

    import rclpy
    import yaml
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, Float64MultiArray, String
    from tf2_msgs.msg import TFMessage
    sys.path.insert(0, str(ROOT / 'src/tap_teleop'))
    from tap_teleop.pose_curves import sample, JOINTS

    base_config = ROOT / 'src/tap_control/config/balance_controller.yaml'
    config = (args.config or base_config).resolve()
    binary = ROOT / 'install/tap_control/lib/tap_control/balance_controller_node'
    curves_path = ROOT / 'src/tap_teleop/config/pose_curves.json'
    config_bytes = config.read_bytes()
    base_config_bytes = base_config.read_bytes()
    effective_config = canonical_ros_parameters(yaml.safe_load(base_config_bytes), yaml.safe_load(config_bytes))
    binary_hash = hashlib.sha256(binary.read_bytes()).hexdigest()
    curves_bytes = curves_path.read_bytes()
    curves = json.loads(curves_bytes)
    if list(JOINTS) != curves['joint_order']:
        raise ValueError('Unexpected pose curve joint order')
    neutral = sample(curves, 'squat', 0)
    deep_curve = None
    if args.deep_squat_knee is not None:
        from tap_teleop.deep_squat import generate_deep_squat
        parameters = effective_config['balance_controller_node']['ros__parameters']
        if parameters['pose_knee_min_rad'] > -args.deep_squat_knee:
            raise ValueError('Controller knee envelope would clip the requested deep curve')
        hips = (parameters['pose_hip_min_rad'], parameters['pose_hip_max_rad'])
        deep_curve = generate_deep_squat(
            ROOT / 'src/tap_robot_description/description/cad_robot/robot.urdf.xacro',
            knee_max=args.deep_squat_knee, hip_bounds=hips, samples=101)
        if deep_curve['joint_order'] != list(JOINTS):
            raise ValueError('Unexpected deep squat joint order')
        neutral = deep_curve['rest_pose'][:]

    def sample_pose(curve, amount):
        if deep_curve is not None and curve == 'squat':
            return sample({'tables': {'squat': deep_curve['table']}}, 'squat', amount)
        return sample(curves, curve, amount)

    partition = f'tap_transform_{os.getpid()}_{uuid.uuid4().hex[:10]}'
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    os.environ['GZ_PARTITION'] = partition
    out = ROOT / 'log' / f'transformation_{time.time_ns()}'
    out.mkdir(parents=True)
    (out / 'transformation_smoke.py').write_bytes(HARNESS_BYTES)
    helper_bytes = (ROOT / 'tools/owned_process.py').read_bytes()
    (out / 'owned_process.py').write_bytes(helper_bytes)
    parameter_helper_bytes = (ROOT / 'tools/ros_parameters.py').read_bytes()
    (out / 'ros_parameters.py').write_bytes(parameter_helper_bytes)
    (out / 'balance_controller.yaml').write_bytes(config_bytes)
    (out / 'base_balance_controller.yaml').write_bytes(base_config_bytes)
    if args.config is not None:
        (out / 'controller_override.yaml').write_bytes(config_bytes)
    effective_bytes = yaml.safe_dump(effective_config, sort_keys=False).encode()
    (out / 'effective_controller.yaml').write_bytes(effective_bytes)
    (out / 'pose_curves.json').write_bytes(curves_bytes)
    deep_bytes = None
    if deep_curve is not None:
        deep_bytes = json.dumps(deep_curve, indent=2, allow_nan=False).encode()
        (out / 'deep_squat_curve.json').write_bytes(deep_bytes)
    for name in ('robot.urdf.xacro', 'simulation.urdf.xacro', 'masses.yaml', 'mesh_inertials.yaml'):
        path = ROOT / 'src/tap_robot_description/description/cad_robot' / name
        if path.exists():
            (out / name).write_bytes(path.read_bytes())
    expanded = subprocess.run(['xacro', str(ROOT / 'src/tap_robot_description/description/cad_robot/simulation.urdf.xacro'),
                               *model_args], check=True, capture_output=True).stdout
    (out / 'expanded_simulation.urdf').write_bytes(expanded)
    (out / 'sim.launch.py').write_bytes((ROOT / 'src/tap_sim/launch/sim.launch.py').read_bytes())
    plugin_hashes = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                     for p in (ROOT / 'install/tap_sim/lib').glob('*.so')}

    process = node = None
    cleanup_result = None
    failure = None
    violations = {}
    estop_result = {'passed': False, 'reason': 'not attempted', 'checks': estop_checks}
    rows, cycles, cycle_ranges = [], [], []
    cache = {}
    now_sim = None
    clock_wall = time.monotonic()
    callback_fault = None
    anchor = None
    target = neutral[:]
    published_target = neutral[:]
    last_publish_sim = None
    publish_count = 0
    stage, cycle = 'startup', 0
    previous = None
    previous_source_pair = None
    previous_pose = None
    sync_waits = 0
    cumulative_residual = 0.0
    trace = (out / 'events.jsonl').open('w')
    launch_log = (out / 'launch.log').open('w')

    def event(kind, payload):
        trace.write(json.dumps({'kind': kind, 'sim': now_sim,
                                'wall': time.monotonic(), 'stage': stage,
                                'cycle': cycle, 'payload': payload}, allow_nan=False) + '\n')

    def store(kind, payload, stamp=None):
        cache[kind] = {'sim': now_sim, 'wall': time.monotonic(),
                       'stamp': stamp, 'data': payload}
        event(kind, payload)

    def guarded(fn):
        def callback(msg):
            nonlocal callback_fault
            try:
                fn(msg)
            except (ValueError, KeyError, TypeError, OverflowError) as exc:
                callback_fault = f'{fn.__name__}: {exc}'
                event('invalid_input', callback_fault)
        return callback

    def timestamp(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    @guarded
    def clock(msg):
        nonlocal now_sim, clock_wall, callback_fault
        value = timestamp(msg.clock)
        if value == now_sim:
            return
        if now_sim is not None and value < now_sim - 1e-6:
            callback_fault = '/clock moved backwards'
        if now_sim != value:
            clock_wall = time.monotonic()
        now_sim = value
        event('clock', value)

    @guarded
    def pose(msg):
        nonlocal previous_pose
        if len(msg.transforms) != 1:
            raise ValueError('Expected dedicated single-model pose')
        t = msg.transforms[0]
        if t.child_frame_id not in ('', 'tap_robot'):
            raise ValueError('Unexpected model frame')
        p, q = t.transform.translation, t.transform.rotation
        if not finite([p.x, p.y, p.z, q.x, q.y, q.z, q.w]):
            raise ValueError('Nonfinite model pose')
        if abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1) > .01:
            raise ValueError('Invalid quaternion norm')
        data = {'x': p.x, 'y': p.y, 'z': p.z,
                'roll': math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y)),
                'pitch': math.asin(max(-1., min(1., 2*(q.w*q.y-q.z*q.x)))),
                'yaw': math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))}
        # Pose_V bridge can emit zero stamps: retain explicit receipt-time basis.
        stamp = timestamp(t.header.stamp) or None
        sample_time = stamp if stamp is not None else now_sim
        if previous_pose is not None and sample_time is not None:
            dt = sample_time - previous_pose['time']
            if dt > 0:
                data['world_speed'] = math.hypot(data['x']-previous_pose['x'], data['y']-previous_pose['y'])/dt
                data['yaw_rate'] = angle_delta(data['yaw'], previous_pose['yaw'])/dt
        if sample_time is not None:
            previous_pose = dict(data, time=sample_time)
        store('pose', data, stamp)

    @guarded
    def joints(msg):
        names = list(msg.name)
        required = list(JOINTS) + ['cad_left_wheel', 'cad_right_wheel']
        if len(set(names)) != len(names) or any(n not in names for n in required):
            raise ValueError('Duplicate or missing joint names')
        indices = [names.index(n) for n in required]
        if any(i >= len(msg.position) or i >= len(msg.velocity) for i in indices):
            raise ValueError('Incomplete joint position/velocity')
        pos = [msg.position[i] for i in indices]
        vel = [msg.velocity[i] for i in indices]
        effort = [msg.effort[i] for i in indices] if all(i < len(msg.effort) for i in indices) else None
        if not finite(pos + vel + (effort or [])):
            raise ValueError('Nonfinite joint measurement')
        stamp = timestamp(msg.header.stamp)
        if stamp <= 0:
            if stage == 'startup':
                return
            raise ValueError('Unstamped joints')
        store('joints', {'positions': pos, 'velocities': vel, 'efforts': effort}, stamp)

    @guarded
    def status(msg):
        parts = msg.data.split('|')
        data = {'mode': parts[0]}
        for field in parts[1:]:
            if '=' in field:
                key, value = field.split('=', 1)
                try:
                    data[key] = float(value)
                except ValueError:
                    data[key] = value
        if not finite([data[k] for k in ('pitch', 'pitch_ref', 'pitch_rate', 'fresh', 'cmd_v', 'cmd_w')]):
            raise ValueError('Invalid status values')
        store('status', data)

    def command_callback(kind, count):
        @guarded
        def receive(msg):
            if len(msg.data) != count or not finite(msg.data):
                raise ValueError(f'Invalid {kind}')
            store(kind, list(msg.data))
        return receive

    def snapshot(enforce=True):
        nonlocal previous, previous_source_pair
        if now_sim is None:
            raise RuntimeError('No /clock')
        times = []
        for key in ('pose', 'joints', 'status', 'legs', 'wheels'):
            item = cache.get(key)
            if item is None or item['sim'] is None:
                raise RuntimeError(f'Missing {key}')
            stamp = item['stamp'] if item['stamp'] is not None else item['sim']
            if now_sim - stamp < -.02:
                raise RuntimeError(f'Clock delivery behind {key}')
            if now_sim - stamp > args.freshness:
                raise RuntimeError(f'Stale {key}: age={now_sim-stamp:.4f}')
            if time.monotonic() - item['wall'] > 2:
                raise RuntimeError(f'Wall-stale {key}')
            times.append(stamp)
        if max(times) - min(times) > args.coherence:
            raise RuntimeError('Incoherent measurement timestamps')
        s, p, j = (cache[k]['data'] for k in ('status', 'pose', 'joints'))
        if enforce:
            check_absolute_pitch(p['pitch'], args.max_absolute_pitch)
            if s['mode'] not in ('Balance', 'Drive', 'PoseChange'):
                raise RuntimeError(f'Unsafe mode: {s["mode"]}')
            if abs(p['roll']) > args.max_roll or p['z'] < args.min_height or abs(angle_delta(p['pitch'], s['pitch_ref'])) > args.max_pitch_error:
                raise RuntimeError('Attitude/height guard failed')
            if s['fresh'] != 1:
                ages = [s.get(k, float('inf'))/1000 for k in ('imu_age_ms', 'ori_age_ms', 'js_age_ms')]
                if finite(ages) and max(ages) <= args.freshness and min(ages) < 0:
                    raise RuntimeError('Controller clock delivery skew')
                raise RuntimeError('Controller reports stale sensors')
        row = dict(p, sim=now_sim, stage=stage, cycle=cycle,
                   mode=s['mode'], controller_pitch=s['pitch'], pitch_ref=s['pitch_ref'],
                   pitch_error=angle_delta(p['pitch'], s['pitch_ref']),
                   joint_positions=j['positions'], joint_velocities=j['velocities'],
                   joint_efforts=j['efforts'], target=published_target[:],
                   trajectory_target=target[:],
                   joint_error=max(abs(q-t) for q, t in zip(j['positions'], published_target)),
                   leg_commands=cache['legs']['data'], wheel_commands=cache['wheels']['data'],
                   stamps={k: cache[k]['stamp'] for k in cache},
                   yaw_unwrapped=p['yaw'])
        if previous is not None:
            row['yaw_unwrapped'] = previous['yaw_unwrapped'] + angle_delta(p['yaw'], previous['yaw'])
        source_pair = tuple(cache[k]['stamp'] if cache[k]['stamp'] is not None else cache[k]['sim']
                            for k in ('pose', 'joints'))
        row['geometry_source_pair'] = source_pair
        # Only trace storage is deduplicated; all guards below still run each tick.
        if append_geometry_sample(now_sim, previous['sim'] if previous is not None else None,
                                  source_pair, previous_source_pair):
            rows.append(row)
            previous = row
            previous_source_pair = source_pair
        if enforce:
            if s['mode'] not in ('Balance', 'Drive', 'PoseChange') or s['fresh'] != 1:
                raise RuntimeError(f'Unsafe mode/freshness: {s["mode"]}/{s["fresh"]}')
            if abs(s['cmd_v']) > 1e-9 or abs(s['cmd_w']) > 1e-9:
                raise RuntimeError('Nonzero raw operator command during stationary trial')
            if abs(row['pitch_error']) > args.max_pitch_error or abs(p['roll']) > args.max_roll or p['z'] < args.min_height:
                raise RuntimeError('Attitude/height guard failed')
            if anchor is not None:
                if math.hypot(p['x']-anchor['x'], p['y']-anchor['y']) > args.max_xy:
                    violations.setdefault('peak_xy', {'sim': now_sim, 'stage': stage})
                if abs(row['yaw_unwrapped']-anchor['yaw_unwrapped']) > args.max_yaw:
                    violations.setdefault('peak_yaw', {'sim': now_sim, 'stage': stage})
        return row

    def tick(check=True):
        nonlocal sync_waits
        if process.poll() is not None:
            raise RuntimeError('Simulation launcher exited')
        rclpy.spin_once(node, timeout_sec=.01)
        if callback_fault:
            raise RuntimeError(callback_fault)
        if check:
            if time.monotonic() - clock_wall > 3:
                raise RuntimeError('/clock stopped advancing')
            deadline = time.monotonic() + .5
            while True:
                try:
                    return snapshot()
                except RuntimeError as exc:
                    transient = str(exc) in ('Incoherent measurement timestamps', 'Controller clock delivery skew') or str(exc).startswith('Clock delivery behind')
                    if not transient or time.monotonic() >= deadline:
                        raise
                    sync_waits += 1
                    # Independent subscribers can be delivered in different orders.
                    # Wait for coherence, but never extend the source freshness bound.
                    rclpy.spin_once(node, timeout_sec=.005)
                    if callback_fault:
                        raise RuntimeError(callback_fault)

    def publish():
        nonlocal last_publish_sim, published_target, publish_count
        if not command_due(now_sim, last_publish_sim):
            return
        # Do not burst missed commands after a delayed clock delivery or pause.
        last_publish_sim = now_sim
        published_target = target[:]
        publish_count += 1
        drive.publish(Twist())
        pose_pub.publish(Float64MultiArray(data=published_target))
        event('target', published_target)

    def run_stage(duration, curve=None, start_amount=0., end_amount=0.):
        nonlocal target
        start = now_sim
        deadline = time.monotonic() + max(40., duration * 20)
        while now_sim - start < duration:
            if time.monotonic() > deadline:
                raise RuntimeError(f'{stage}: wall-time budget exceeded')
            if curve is not None:
                ratio = smooth5((now_sim-start)/duration)
                target = sample_pose(curve, start_amount+(end_amount-start_amount)*ratio)
            publish()
            tick()
        if curve is not None:
            target = sample_pose(curve, end_amount)

    try:
        rclpy.init(args=[])
        node = rclpy.create_node('tap_transformation_smoke')
        subscriptions = []
        for topic, kind, cb in (('/clock', Clock, clock), ('/tap_robot/pose', TFMessage, pose),
                                ('/joint_states', JointState, joints), ('/control_mode', String, status),
                                ('/leg_effort_controller/commands', Float64MultiArray, command_callback('legs', 4)),
                                ('/wheel_effort_controller/commands', Float64MultiArray, command_callback('wheels', 2))):
            subscriptions.append(node.create_subscription(kind, topic, cb, qos_profile_sensor_data))
        drive = node.create_publisher(Twist, '/cmd_vel', 10)
        pose_pub = node.create_publisher(Float64MultiArray, '/target_pose', 10)
        stop = node.create_publisher(Bool, '/emergency_stop', 10)
        # Refuse to join an existing controller/simulation on the requested domain.
        end = time.monotonic() + 1.5
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)
        if cache or now_sim is not None:
            raise RuntimeError('Requested ROS domain already has simulation/control traffic')
        argv = [str(ROOT / 'run_sim.sh'), 'gui:=false', 'auto_enable:=true']
        if args.config is not None:
            argv.append(f'controller_config:={config}')
        argv.extend(args.launch_arg)
        (out / 'launch_arguments.json').write_text(json.dumps(argv, indent=2))
        process = start_owned_process(argv, cwd=ROOT, env=os.environ.copy(),
                                   stdout=launch_log, stderr=subprocess.STDOUT, start_new_session=True)
        print(f'TRIAL LAUNCHED pid={process.pid} domain={args.domain} binary={binary_hash}', flush=True)
        deadline = time.monotonic() + 90
        while time.monotonic() < deadline:
            tick(False)
            if cache.get('status', {}).get('data', {}).get('mode') == 'Balance':
                try:
                    snapshot()
                    break
                except RuntimeError as exc:
                    if str(exc).startswith(('Missing', 'Stale', 'Incoherent')):
                        continue
                    raise
        else:
            raise RuntimeError('Startup did not produce coherent Balance observations')
        stage = 'settle'
        run_stage(args.settle)
        anchor = tick().copy()
        event('anchor', anchor)
        last_return = anchor
        for cycle in range(1, args.cycles + 1):
            first = len(rows)
            stage = 'neutral'
            target = neutral[:]
            run_stage(args.dwell)
            for curve in motion_curves(args.motions):
                amplitude = args.squat_amplitude if curve == 'squat' else args.amplitude
                for returning in (False, True):
                    stage = f'{curve}_' + ('return' if returning else 'out')
                    a, b = (amplitude, 0.) if returning else (0., amplitude)
                    run_stage(args.transition, curve, a, b)
                    stage += '_dwell'
                    run_stage(args.dwell)
                    row = tick()
                    if row['joint_error'] > args.max_joint_error:
                        violations.setdefault(f'cycle{cycle}_{stage}_joint', {'sim': now_sim, 'error': row['joint_error']})
                    if returning and math.hypot(row['x']-anchor['x'], row['y']-anchor['y']) > args.max_residual:
                        violations.setdefault(f'cycle{cycle}_{stage}_residual', {'sim': now_sim})
            row = tick()
            cumulative_residual += math.hypot(row['x']-last_return['x'], row['y']-last_return['y'])
            last_return = row
            # Scan traces only after shutdown; long summaries starve subscriptions.
            cycle_ranges.append((cycle, first, len(rows), cumulative_residual))
        if violations:
            failure = 'Transformation bounds exceeded (full cycles collected): ' + ', '.join(violations)
    except Exception as exc:
        failure = str(exc)
    finally:
        if process is not None and node is not None:
            stage = 'estop'
            start_wall = time.monotonic()
            start_sim = now_sim
            try:
                zero_since = None
                while time.monotonic() - start_wall < 3:
                    stop.publish(Bool(data=True))
                    rclpy.spin_once(node, timeout_sec=.02)
                    zero = estop_observation_zero(cache, start_wall, time.monotonic(), bounded_actuators,
                                                  start_sim, now_sim, args.freshness)
                    zero_since = (zero_since or time.monotonic()) if zero else None
                    if zero_since is not None and time.monotonic() - zero_since >= .2:
                        estop_result = {'passed': True, 'zero_commands_wall_s': .2, 'checks': estop_checks}
                        if bounded_actuators:
                            estop_result['zero_applied_effort_wall_s'] = .2
                        break
                if not estop_result['passed']:
                    estop_result['reason'] = f'No fresh sustained zero {estop_checks}'
                    failure = failure or estop_result['reason']
            except Exception as exc:
                estop_result['reason'] = str(exc)
                failure = failure or f'E-stop check: {exc}'
        try:
            cleanup_result = cleanup_group(process)
        except Exception as exc:
            failure = failure or f'Cleanup: {exc}'
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        trace.close()
        launch_log.close()
        for number, first, end, residual in cycle_ranges:
            result = metrics(rows[first:end], anchor)
            result.update(cycle=number, cumulative_cycle_residual_m=residual)
            cycles.append(result)
        active = [r for r in rows if r['cycle'] > 0]
        summary = {'failure': failure, 'binary_sha256': binary_hash,
                   'cleanup': cleanup_result,
                   'expanded_simulation_urdf_sha256': hashlib.sha256(expanded).hexdigest(),
                   'owned_process_sha256': hashlib.sha256(helper_bytes).hexdigest(),
                   'ros_parameters_sha256': hashlib.sha256(parameter_helper_bytes).hexdigest(),
                   'model_xacro_arguments': model_args, 'plugin_hashes': plugin_hashes,
                   'geometry_sampling': 'unique coherent (pose,joints) source-time pairs; guards run every tick',
                   'harness_sha256': hashlib.sha256(HARNESS_BYTES).hexdigest(),
                   'deep_curve_sha256': hashlib.sha256(deep_bytes).hexdigest() if deep_bytes else None,
                   'bound_violations': violations,
                   'config_sha256': hashlib.sha256(config_bytes).hexdigest(),
                   'base_config_sha256': hashlib.sha256(base_config_bytes).hexdigest(),
                   'override_config_sha256': hashlib.sha256(config_bytes).hexdigest() if args.config else None,
                   'effective_config_sha256': hashlib.sha256(effective_bytes).hexdigest(),
                   'effective_parameter_basis': 'Canonical dotted YAML names, base then override; not a live service dump',
                   'arguments': {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()},
                   'partition': partition, 'anchor': anchor, 'cycles': cycles,
                   'metrics': metrics(active, anchor), 'estop': estop_result,
                   'synchronization_waits': sync_waits,
                   'publication': {'rate_hz': 20, 'time_basis': '/clock',
                                   'policy': 'at most one command pair per 50 ms; no catch-up burst',
                                   'command_pairs': publish_count},
                   'samples': len(rows), 'pose_time_basis': 'source stamp or receipt /clock for zero-stamp bridge',
                   'limits': 'Moderate transformations only; no jump, hardware or torque calibration claim'}
        (out / 'samples.json').write_text(json.dumps(rows, indent=2, allow_nan=False))
        (out / 'summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False))
        print(json.dumps(summary, indent=2))
        print('Evidence:', out)
    return 1 if failure else 0


if __name__ == '__main__':
    raise SystemExit(main())
