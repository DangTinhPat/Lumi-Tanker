#!/usr/bin/env python3
"""Run an isolated TAP Gazebo balance trial and retain quantitative evidence."""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, String


ROOT = Path(__file__).resolve().parents[1]


def main():
    from owned_process import cleanup_group, start_owned_process
    from tap_launch import sim_launch_command
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=15.0)
    parser.add_argument('--exercise', action='store_true')
    parser.add_argument('--poses', action='store_true')
    parser.add_argument('--gui', action='store_true',
                        help='keep Gazebo GUI visible during the motion trial')
    parser.add_argument('--domain', type=int, default=87)
    parser.add_argument('--max-pitch-error', type=float, default=0.30)
    parser.add_argument('--controller-config', type=str, default=None,
                        help='extra controller_config:= YAML layered over the base config')
    args = parser.parse_args()
    if args.duration <= 0 or args.max_pitch_error <= 0:
        parser.error('Duration and pitch-error bound must be positive')
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    os.environ['GZ_PARTITION'] = f'tap_smoke_{os.getpid()}'
    out = ROOT / 'log' / f'smoke_{int(time.time())}'
    out.mkdir(parents=True, exist_ok=True)
    samples, wheels, legs = [], [], []
    actual_joints = {}
    stage = 'startup'

    def joint_state(msg):
        actual_joints.update(zip(msg.name, msg.position))

    def status(msg):
        fields = msg.data.split('|')
        row = {'mode': fields[0], 'stage': stage, 'wall': time.monotonic()}
        row['joint_positions'] = dict(actual_joints)
        for field in fields[1:]:
            if '=' in field:
                key, value = field.split('=', 1)
                try:
                    row[key] = float(value)
                except ValueError:
                    row[key] = value
        samples.append(row)

    log = (out / 'launch.log').open('w')
    process = None
    cleanup_result = None
    rclpy.init()
    node = rclpy.create_node('tap_smoke')
    subscriptions = [node.create_subscription(JointState, '/joint_states', joint_state, 100),
                     node.create_subscription(String, '/control_mode', status, 100),
                     node.create_subscription(Float64MultiArray, '/wheel_effort_controller/commands',
                                              lambda m: wheels.append(list(m.data)), 10),
                     node.create_subscription(Float64MultiArray, '/leg_effort_controller/commands',
                                              lambda m: legs.append(list(m.data)), 10)]
    drive = node.create_publisher(Twist, '/cmd_vel', 10)
    estop = node.create_publisher(Bool, '/emergency_stop', 10)
    pose = node.create_publisher(Float64MultiArray, '/target_pose', 10)
    failure = None
    config = ROOT / 'src/tap_control/config/balance_controller.yaml'
    (out / 'balance_controller.yaml').write_bytes(config.read_bytes())
    model = ROOT / 'src/tap_robot_description/description/cad_robot/robot.urdf.xacro'
    (out / 'robot.urdf.xacro').write_bytes(model.read_bytes())
    binary = ROOT / 'install/tap_control/lib/tap_control/balance_controller_node'
    binary_hash = hashlib.sha256(binary.read_bytes()).hexdigest()
    try:
        launch_args = sim_launch_command(
            f'gui:={str(args.gui).lower()}', 'auto_enable:=true')
        if args.controller_config:
            launch_args.append(f'controller_config:={args.controller_config}')
        process = start_owned_process(launch_args,
                                      cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
        deadline = time.monotonic() + 70
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if process.poll() is not None:
                raise RuntimeError('Simulation launch exited during startup')
            if samples and samples[-1]['mode'] == 'Balance':
                break
        else:
            raise RuntimeError(f'Balance not reached; latest: {samples[-1:]!r}')

        stages = [('hold', args.duration, 0.0, 0.0)]
        if args.exercise:
            stages += [('forward', 3.0, 0.06, 0.0),
                       ('stop_after_forward', 2.0, 0.0, 0.0),
                       ('backward', 3.0, -0.06, 0.0),
                       ('stop_after_backward', 2.0, 0.0, 0.0)]
        pose_targets = {}
        if args.poses:
            curves = json.loads((ROOT / 'src/tap_teleop/config/pose_curves.json').read_text())
            for name, curve, amount in [('squat', 'squat', .25), ('stand', 'squat', 0),
                                        ('roll_left', 'roll_left', .20),
                                        ('roll_right', 'roll_right', .20),
                                        ('stand_after_roll', 'squat', 0)]:
                table = curves['tables'][curve]
                pose_targets[name] = table[round(amount * (len(table) - 1))]
                stages.append((name, 5.0, 0.0, 0.0))
        for stage_name, duration, speed, yaw in stages:
            stage = stage_name
            elapsed = 0.0
            last_count = len(samples)
            deadline = time.monotonic() + max(30, duration * 5)
            while elapsed < duration and time.monotonic() < deadline:
                command = Twist()
                command.linear.x = speed
                command.angular.z = yaw
                drive.publish(command)
                if stage in pose_targets:
                    pose.publish(Float64MultiArray(data=pose_targets[stage]))
                rclpy.spin_once(node, timeout_sec=0.05)
                for row in samples[last_count:]:
                    elapsed += max(0, min(0.1, row.get('dt_ms', 0) * 0.001))
                    if row['mode'] not in ('Balance', 'Drive', 'PoseChange'):
                        raise RuntimeError(f'{stage}: mode {row["mode"]}, pitch={row.get("pitch")}')
                    error = abs(row['pitch'] - row['pitch_ref'])
                    if not math.isfinite(error) or error > args.max_pitch_error:
                        raise RuntimeError(f'{stage}: pitch error {error:.3f} exceeds bound')
                last_count = len(samples)
            if elapsed < duration:
                raise RuntimeError(f'{stage}: insufficient simulated observation ({elapsed:.2f}s)')
            rows = [row for row in samples if row['stage'] == stage]
            if not all('obs_x' in row and 'obs_y' in row for row in rows):
                raise RuntimeError(f'{stage}: missing encoder trajectory observer')
            dx = rows[-1]['obs_x'] - rows[0]['obs_x']
            dy = rows[-1]['obs_y'] - rows[0]['obs_y']
            if stage == 'hold' and math.hypot(dx, dy) > .10:
                raise RuntimeError(f'Hold drift exceeds 0.10 m: {math.hypot(dx, dy):.3f}')
            if stage in ('forward', 'stop_after_forward', 'backward',
                         'stop_after_backward'):
                tail = rows[-min(100, len(rows)):]
                error = sum(abs(row['wheel_vel'] - speed) for row in tail) / len(tail)
                if error > .025:
                    raise RuntimeError(f'{stage}: final speed MAE {error:.3f} exceeds 0.025 m/s')
                valid_rows = [row for row in rows if row.get('obs_valid', 0) > 0.5]
                if len(valid_rows) < max(5, len(rows) // 4):
                    raise RuntimeError(f'{stage}: observer did not produce valid samples')
                observed_dx = valid_rows[-1]['obs_x'] - valid_rows[0]['obs_x']
                if stage == 'forward' and observed_dx < .03:
                    raise RuntimeError(f'{stage}: observer displacement is not forward: {observed_dx:.3f} m')
                if stage == 'backward' and observed_dx > -.03:
                    raise RuntimeError(f'{stage}: observer displacement is not backward: {observed_dx:.3f} m')
                if stage.startswith('stop_'):
                    if abs(valid_rows[-1]['obs_v']) > .025:
                        raise RuntimeError(f'{stage}: observer did not settle: {valid_rows[-1]["obs_v"]:.3f} m/s')
                if any(row.get('flight_airborne', 0) > 0.5 for row in rows):
                    raise RuntimeError(f'{stage}: flat-ground drive falsely entered flight')
            if stage in pose_targets:
                names = curves['joint_order']
                measured = rows[-1]['joint_positions']
                if any(name not in measured for name in names):
                    raise RuntimeError(f'{stage}: missing measured joint positions')
                if max(abs(measured[name] - target) for name, target in
                       zip(names, pose_targets[stage])) > .08:
                    raise RuntimeError(f'{stage}: final joint tracking error exceeds 0.08 rad')
                if math.hypot(dx, dy) > .15:
                    raise RuntimeError(f'{stage}: pose-transition drift exceeds 0.15 m')

        stage = 'estop'
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            estop.publish(Bool(data=True))
            rclpy.spin_once(node, timeout_sec=0.05)
        if not wheels or not legs or any(abs(v) > 1e-9 for v in wheels[-1] + legs[-1]):
            raise RuntimeError('Emergency stop did not zero actuator commands')
        print('PASS: requested balance, tracking, pose bounds and emergency-stop commands')
    except Exception as exc:
        failure = str(exc)
        print('FAIL:', failure)
    finally:
        try:
            cleanup_result = cleanup_group(process)
        except Exception as exc:
            failure = failure or f'Cleanup: {exc}'
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        log.close()
        (out / 'samples.json').write_text(json.dumps(samples, indent=2))
        summary = {'failure': failure, 'samples': len(samples),
                   'cleanup': cleanup_result,
                   'binary_sha256': binary_hash, 'arguments': vars(args), 'stages': {}}
        for name in dict.fromkeys(row['stage'] for row in samples):
            rows = [r for r in samples if r['stage'] == name]
            summary['stages'][name] = {
                'simulated_s': sum(max(0, min(.1, r.get('dt_ms', 0) * .001)) for r in rows),
                'max_pitch_error_rad': max(abs(r.get('pitch', 0) - r.get('pitch_ref', 0)) for r in rows),
                'wheel_displacement_m': rows[-1].get('wheel_pos', 0) - rows[0].get('wheel_pos', 0),
                'modes': sorted({r['mode'] for r in rows}),
                'observer_dx_m': rows[-1].get('obs_x', 0) - rows[0].get('obs_x', 0),
                'observer_dy_m': rows[-1].get('obs_y', 0) - rows[0].get('obs_y', 0),
                'yaw_change_rad': math.remainder(rows[-1].get('yaw', 0) - rows[0].get('yaw', 0),
                                                2 * math.pi),
                'final_wheel_velocity_m_s': rows[-1].get('wheel_vel'),
                'final_joint_positions': rows[-1].get('joint_positions'),
                'roll_change_rad': rows[-1].get('roll', 0) - rows[0].get('roll', 0),
                'observer_final_velocity_m_s': rows[-1].get('obs_v'),
                'observer_confidence': rows[-1].get('obs_conf'),
            }
        (out / 'summary.json').write_text(json.dumps(summary, indent=2))
        print(json.dumps(summary, indent=2))
        print('Evidence:', out)
    return 1 if failure else 0


if __name__ == '__main__':
    raise SystemExit(main())
