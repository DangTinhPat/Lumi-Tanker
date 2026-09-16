#!/usr/bin/env python3
"""Landing/touchdown-balance recovery test swept across ledge heights.

Same isolated-world methodology as clean_ledge_smoke.py (ground_plane plus
one static ledge_pad, nothing else nearby - see that tool's docstring for
why an isolated world matters), generalized to an arbitrary height via
tools/generate_ledge_world.py instead of the hardcoded 20 cm
ledge_test.sdf. One fresh owned sim launch per trial.
"""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(__file__).resolve().parent))
from generate_ledge_world import generate, spawn_z_for  # noqa: E402

PAD_CENTER_X = 0.0
PAD_EDGE_X = 0.45  # matches generate_ledge_world.py's PAD_HALF_FOOTPRINT
STOP_DRIVE_X = PAD_EDGE_X + 0.10
DRIVE_SPEED_M_S = 1.0
RECOVER_PITCH_RAD = .15
RECOVER_PITCH_RATE_RAD_S = .5
RECOVER_HOLD_S = 2.0


def run_trial(trial_id, height_m, world_path, controller_config, out_dir, domain_id):
    os.environ['ROS_DOMAIN_ID'] = str(domain_id)
    os.environ['GZ_PARTITION'] = f'tap_ledge_h_{trial_id}_{os.getpid()}_{time.time_ns()}'
    from owned_process import start_owned_process, cleanup_group
    from tap_launch import sim_launch_command
    import rclpy
    from geometry_msgs.msg import Twist
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage

    log_path = out_dir / f'launch_{trial_id}.log'
    stream = log_path.open('w')
    pose, state = {}, {}
    sim_time = 0.0

    def clock_cb(msg):
        nonlocal sim_time
        sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def pose_cb(msg):
        if len(msg.transforms) != 1:
            return
        t = msg.transforms[0].transform
        q = t.rotation
        pose.update(x=t.translation.x, z=t.translation.z,
                    pitch=math.asin(max(-1, min(1, 2*(q.w*q.y-q.z*q.x)))),
                    roll=math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y)))

    def status_cb(msg):
        fields = msg.data.split('|')
        state.clear()
        state['mode'] = fields[0]
        for field in fields[1:]:
            if '=' in field:
                key, value = field.split('=', 1)
                try:
                    state[key] = float(value)
                except ValueError:
                    state[key] = value

    rclpy.init()
    node = rclpy.create_node(f'tap_ledge_h_{trial_id}'.replace('.', '_').replace('-', 'n'))
    node.create_subscription(Clock, '/clock', clock_cb, 100)
    node.create_subscription(TFMessage, '/tap_robot/pose', pose_cb, 100)
    node.create_subscription(String, '/control_mode', status_cb, 100)
    drive_pub = node.create_publisher(Twist, '/cmd_vel', 10)

    process = None
    cleanup_result = None
    failure = None
    trial_result = None

    def spin():
        if process.poll() is not None:
            raise RuntimeError('Launch exited')
        rclpy.spin_once(node, timeout_sec=.05)

    def observe(duration, wall_limit=30):
        start, end = sim_time, time.monotonic() + wall_limit
        while sim_time - start < duration and time.monotonic() < end:
            spin()
        if sim_time - start < duration:
            raise RuntimeError('Sim-time observation timed out')

    def wait_for_mode(mode, timeout_s):
        deadline = time.monotonic() + timeout_s
        while state.get('mode') != mode and time.monotonic() < deadline:
            spin()
        return state.get('mode') == mode

    try:
        spawn_z = spawn_z_for(height_m)
        args = sim_launch_command('gui:=false', 'auto_enable:=true',
                                   f'ledge_world:={world_path}', f'spawn_z:={spawn_z}')
        if controller_config:
            args.append(f'controller_config:={controller_config}')
        process = start_owned_process(args, cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT)
        if not wait_for_mode('Balance', 90):
            raise RuntimeError(f'Startup did not reach Balance: mode={state.get("mode")}')
        observe(1.0)
        start_x = pose.get('x', PAD_CENTER_X)
        if abs(start_x - PAD_CENTER_X) > 0.2:
            raise RuntimeError(f'Unexpected spawn x={start_x}, expected near {PAD_CENTER_X}')

        crossed_edge = False
        drive_deadline = time.monotonic() + 30
        drive_cmd = Twist()
        drive_cmd.linear.x = DRIVE_SPEED_M_S
        while time.monotonic() < drive_deadline:
            drive_pub.publish(drive_cmd)
            spin()
            if pose.get('x', start_x) >= STOP_DRIVE_X:
                crossed_edge = True
                break
        drive_pub.publish(Twist())
        if not crossed_edge:
            raise RuntimeError(f'Never reached STOP_DRIVE_X={STOP_DRIVE_X}; '
                              f'final x={pose.get("x")}')

        drop_start = sim_time
        peak_pitch, peak_pitch_rate = 0.0, 0.0
        peak_roll = 0.0
        min_z = pose.get('z', 9.0)
        stable = 0.0
        recovered = False
        previous = sim_time
        # Higher drops take longer to fall and to settle; scale the wall
        # budget with height instead of using clean_ledge_smoke.py's fixed
        # 25s (tuned for 0.20 m only).
        wall_budget = max(25.0, 10.0 * math.sqrt(2.0 * max(height_m, 0.01) / 9.81) + 15.0)
        wall_deadline = time.monotonic() + wall_budget
        while time.monotonic() < wall_deadline:
            spin()
            dt = max(0.0, sim_time - previous)
            previous = sim_time
            p = abs(state.get('pitch', 9.0))
            pr = abs(state.get('pitch_rate', 9.0))
            peak_pitch = max(peak_pitch, p)
            peak_pitch_rate = max(peak_pitch_rate, pr)
            peak_roll = max(peak_roll, abs(pose.get('roll', 0.0)))
            min_z = min(min_z, pose.get('z', min_z))
            upright = (state.get('mode') in ('Balance', 'Drive') and
                       p < RECOVER_PITCH_RAD and pr < RECOVER_PITCH_RATE_RAD_S)
            stable = stable + dt if upright else 0.0
            elapsed = sim_time - drop_start
            if stable >= RECOVER_HOLD_S and elapsed > 0.3:
                recovered = True
                break
            if elapsed > wall_budget - 10.0:
                break
        trial_result = dict(
            trial=trial_id, height_m=height_m, launched=True, crossed_edge=crossed_edge,
            recovered=recovered,
            peak_pitch_rad=round(peak_pitch, 4),
            peak_pitch_rate_rad_s=round(peak_pitch_rate, 4),
            peak_roll_rad=round(peak_roll, 4),
            min_z_m=round(min_z, 4),
            final_mode=state.get('mode'), final_pitch_rad=state.get('pitch'),
            final_x=pose.get('x'), final_z=pose.get('z'),
            elapsed_sim_s=round(sim_time - drop_start, 3))
        print(f'[{trial_id}] h={height_m:.2f}m crossed_edge={crossed_edge} '
              f'recovered={recovered} peak_pitch={peak_pitch:.3f} peak_roll={peak_roll:.3f} '
              f'min_z={min_z:.3f} final_mode={state.get("mode")}', flush=True)
    except Exception as exc:
        failure = str(exc)
        trial_result = dict(trial=trial_id, height_m=height_m, launched=False, reason=failure)
        print(f'[{trial_id}] h={height_m:.2f}m FAILED: {failure}', flush=True)
    finally:
        try:
            cleanup_result = cleanup_group(process) if process is not None else None
        except Exception as exc:
            failure = failure or f'Cleanup: {exc}'
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        stream.close()

    trial_result['cleanup'] = cleanup_result
    trial_result['launch_log'] = str(log_path)
    return trial_result


def summarize(trials):
    entry = {'launched': 0, 'crossed_edge': 0, 'recovered': 0, 'peak_pitch_max': 0.0}
    for row in trials:
        if not row.get('launched'):
            continue
        entry['launched'] += 1
        entry['crossed_edge'] += 1 if row.get('crossed_edge') else 0
        entry['recovered'] += 1 if row.get('recovered') else 0
        entry['peak_pitch_max'] = max(entry['peak_pitch_max'], row.get('peak_pitch_rad', 0.0))
    entry['recovery_rate'] = entry['recovered'] / entry['launched'] if entry['launched'] else 0.0
    return entry


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--heights', type=str, default='0.10,0.20,0.30,0.50,0.70,1.00',
                        help='comma-separated ledge heights in meters')
    parser.add_argument('--trials', type=int, default=10,
                        help='fresh sim launches per height')
    parser.add_argument('--controller-config', type=str, default=None,
                        help='override balance_controller.yaml (default: stock production)')
    args = parser.parse_args()

    heights = [float(h) for h in args.heights.split(',')]
    out_dir = ROOT / 'log' / f'ledge_height_{int(time.time())}'
    out_dir.mkdir(parents=True)
    worlds_dir = out_dir / 'worlds'

    conditions = {}
    # Fast-RTPS's default port formula overflows past domain ID ~232; trials
    # run strictly sequentially (one owned sim at a time, fully torn down via
    # cleanup_group before the next starts), so wrapping the id is safe.
    domain_base = 150
    domain_span = 60
    trial_counter = 0
    for height_m in heights:
        world_path = worlds_dir / f'ledge_h{height_m:.2f}.sdf'
        world_path.parent.mkdir(parents=True, exist_ok=True)
        world_path.write_text(generate(height_m))
        label = f'h{height_m:.2f}'
        print(f'=== {label} (world={world_path}) ===', flush=True)
        trials = []
        for trial in range(args.trials):
            trial_id = f'{label}_{trial}'
            domain = domain_base + (trial_counter % domain_span)
            trial_counter += 1
            result = run_trial(trial_id, height_m, world_path, args.controller_config,
                                out_dir, domain)
            trials.append(result)
            (out_dir / 'result.json').write_text(json.dumps(
                {'trials_per_condition': args.trials,
                 'conditions': conditions | {label: {'height_m': height_m, 'trials': trials}}},
                indent=2))
        conditions[label] = {'height_m': height_m, 'trials': trials}

    summary = {label: summarize(c['trials']) for label, c in conditions.items()}
    result = {'trials_per_condition': args.trials, 'conditions': conditions, 'summary': summary}
    (out_dir / 'result.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(summary, indent=2))
    print('Evidence:', out_dir)

    any_launch_failure = any(
        not row.get('launched') for c in conditions.values() for row in c['trials'])
    return 1 if any_launch_failure else 0


if __name__ == '__main__':
    raise SystemExit(main())
