#!/usr/bin/env python3
"""Landing/touchdown-balance recovery test: drive forward off the REAL
rough_arena_spawn_pad edge in src/tap_sim/worlds/rough_training.sdf (terrain:=
rough), the exact scenario the user reported and an earlier session
confirmed-failing (pitch ~3.12 rad, flips regardless of joint-damping tuning).

Unlike tools/landing_ledge_smoke.py (a synthetic vertical teleport-drop with
zero horizontal velocity -- confirmed too easy a disturbance: baseline
recovers it trivially up to 0.50 m, see that run's evidence), this tool
reproduces the actual failure mechanism: the robot is driven forward via
/cmd_vel from the pad center (x=-4.6, spawn per sim.launch.py's terrain=rough
default) across the pad's real edge (half-width 0.45 m -> edge at x=-4.15),
carrying real forward wheel/ground momentum at the moment contact is lost,
matching "Driving +X from here crosses onto the heightfield" in the world
file's own authoring comment.

One fresh owned sim launch per trial (recovery_smoke.py's proven pattern).
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
PAD_CENTER_X = -4.6
PAD_EDGE_X = -4.6 + 0.45  # -4.15, pad half-width from rough_training.sdf
STOP_DRIVE_X = PAD_EDGE_X + 0.10  # commit past the edge before cutting drive
DRIVE_SPEED_M_S = 1.0
RECOVER_PITCH_RAD = .15
RECOVER_PITCH_RATE_RAD_S = .5
RECOVER_HOLD_S = 2.0
LANDING_CONFIG = ROOT / 'src/tap_control/config/landing_recovery_test.yaml'


def run_trial(trial_id, controller_config, out_dir, domain_id):
    os.environ['ROS_DOMAIN_ID'] = str(domain_id)
    os.environ['GZ_PARTITION'] = f'tap_ledge_{trial_id}_{os.getpid()}_{time.time_ns()}'
    sys.path.insert(0, str(Path(__file__).resolve().parent))
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
    node = rclpy.create_node(f'tap_ledge_smoke_{trial_id}'.replace('.', '_').replace('-', 'n'))
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
        args = sim_launch_command('gui:=false', 'auto_enable:=true', 'terrain:=rough')
        if controller_config:
            args.append(f'controller_config:={controller_config}')
        process = start_owned_process(args, cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT)
        if not wait_for_mode('Balance', 90):
            raise RuntimeError(f'Startup did not reach Balance: mode={state.get("mode")}')
        observe(1.0)
        start_x = pose.get('x', PAD_CENTER_X)
        if abs(start_x - PAD_CENTER_X) > 0.2:
            raise RuntimeError(f'Unexpected spawn x={start_x}, expected near {PAD_CENTER_X}')

        # Drive forward (+X, per rough_training.sdf's own "Driving +X from here
        # crosses onto the heightfield" comment) until committed past the real
        # pad edge, carrying real forward momentum into the drop -- the
        # mechanism a pure vertical teleport-drop cannot reproduce.
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
        drive_pub.publish(Twist())  # explicit stop; /cmd_vel also auto-expires after 0.5s
        if not crossed_edge:
            raise RuntimeError(f'Never reached STOP_DRIVE_X={STOP_DRIVE_X}; '
                              f'final x={pose.get("x")}')

        drop_start = sim_time
        peak_pitch, peak_pitch_rate = 0.0, 0.0
        stable = 0.0
        recovered = False
        previous = sim_time
        wall_deadline = time.monotonic() + 25
        while time.monotonic() < wall_deadline:
            spin()
            dt = max(0.0, sim_time - previous)
            previous = sim_time
            p = abs(state.get('pitch', 9.0))
            pr = abs(state.get('pitch_rate', 9.0))
            peak_pitch = max(peak_pitch, p)
            peak_pitch_rate = max(peak_pitch_rate, pr)
            upright = (state.get('mode') in ('Balance', 'Drive') and
                       p < RECOVER_PITCH_RAD and pr < RECOVER_PITCH_RATE_RAD_S)
            stable = stable + dt if upright else 0.0
            elapsed = sim_time - drop_start
            if stable >= RECOVER_HOLD_S and elapsed > 0.3:
                recovered = True
                break
            if elapsed > 10.0:
                break
        trial_result = dict(
            trial=trial_id, launched=True, crossed_edge=crossed_edge, recovered=recovered,
            peak_pitch_rad=round(peak_pitch, 4),
            peak_pitch_rate_rad_s=round(peak_pitch_rate, 4),
            final_mode=state.get('mode'), final_pitch_rad=state.get('pitch'),
            final_x=pose.get('x'), final_z=pose.get('z'),
            elapsed_sim_s=round(sim_time - drop_start, 3))
        print(f'[{trial_id}] crossed_edge={crossed_edge} recovered={recovered} '
              f'peak_pitch={peak_pitch:.3f} final_mode={state.get("mode")}', flush=True)
    except Exception as exc:
        failure = str(exc)
        trial_result = dict(trial=trial_id, launched=False, reason=failure)
        print(f'[{trial_id}] FAILED: {failure}', flush=True)
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
    return entry


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trials', type=int, default=2,
                        help='fresh sim launches per condition (each is a full startup)')
    parser.add_argument('--skip-baseline', action='store_true')
    parser.add_argument('--skip-enabled', action='store_true')
    args = parser.parse_args()

    out_dir = ROOT / 'log' / f'ledge_driveoff_{int(time.time())}'
    out_dir.mkdir(parents=True)

    conditions = {}
    domain = 171
    plan = []
    if not args.skip_baseline:
        plan.append(('baseline', None))
    if not args.skip_enabled:
        plan.append(('enabled', LANDING_CONFIG))

    for label, cfg in plan:
        print(f'=== {label} ({cfg or "stock balance_controller.yaml"}) ===', flush=True)
        trials = []
        for trial in range(args.trials):
            trial_id = f'{label}_{trial}'
            domain += 1
            result = run_trial(trial_id, cfg, out_dir, domain)
            trials.append(result)
            (out_dir / 'result.json').write_text(json.dumps(
                {'trials_per_condition': args.trials,
                 'conditions': conditions | {label: {'trials': trials}}}, indent=2))
        conditions[label] = {'controller_config': str(cfg) if cfg else None, 'trials': trials}

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
