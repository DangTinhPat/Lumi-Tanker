#!/usr/bin/env python3
"""Landing/touchdown-balance recovery test: baseline vs. the phase-gated
landing-recovery boost (MpcBalancer::Config::landing_*, see
src/tap_control/core/src/MpcBalancer.cpp), at several drop heights.

This is a synthetic vertical teleport-drop test, not a literal drive-forward-
off-the-rough_arena_spawn_pad maneuver: after startup reaches Balance, the
robot is teleported straight up by the target drop height above its normal
flat-ground standing pose (spawn_z=0.2372, see src/tap_sim/CONTRACT.md) and
gravity does the rest, landing back on flat ground (terrain:=none/empty.sdf).
That reproduces the same drop-height/touchdown-transient physics as stepping
off a ledge of that height -- the whole-body pitch/velocity impulse the
landing-recovery feature targets -- while being far simpler than driving off
src/tap_sim/worlds/rough_training.sdf's real pad edge. It does not exercise
the edge-of-contact-loss-while-driving transient specifically, only the
free-fall-then-touchdown one.

Heights: 0.05 m and 0.10 m bracket the rough_training.sdf step_band_N range
(individual bands only differ from their neighbors by 1-2 cm, too little to
be a meaningful single-ledge drop, so these are synthetic heights in that
range rather than an actual band-to-band drop); 0.20 m reproduces the
rough_arena_spawn_pad edge already characterized (a separate session, same
repo) as flipping the robot regardless of leg-damping tuning.

One fresh owned sim launch per trial (like tools/recovery_smoke.py), not a
reset-and-repeat loop within one session: this robot is a statically unstable
wheeled inverted pendulum, so /emergency_stop zeroes leg torque too and makes
it collapse rather than pause, and gz_set_pose only resets the base link pose
(not joint angles/velocities) -- an earlier version of this tool tried an
estop+teleport+reset_controller reset between trials in one session and every
single "reset" itself triggered an uncontrolled collapse before any real drop
trial ran. A fresh launch sidesteps both problems at the cost of more wall
time per trial.
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
STAND_Z = 0.2372  # CONTRACT.md flat-ground wheel-ground offset
RECOVER_PITCH_RAD = .15
RECOVER_PITCH_RATE_RAD_S = .5
RECOVER_HOLD_S = 2.0
LANDING_CONFIG = ROOT / 'src/tap_control/config/landing_recovery_test.yaml'


def gz_set_pose(x, y, z, w=1., qx=0., qy=0., qz=0.):
    req = (f'name: "tap_robot" position {{x: {x} y: {y} z: {z}}} '
           f'orientation {{w: {w} x: {qx} y: {qy} z: {qz}}}')
    result = subprocess.run(
        ['gz', 'service', '-s', '/world/empty/set_pose', '--reqtype', 'gz.msgs.Pose',
         '--reptype', 'gz.msgs.Boolean', '--timeout', '3000', '--req', req],
        capture_output=True, text=True, timeout=5)
    if result.returncode or 'data: true' not in result.stdout:
        raise RuntimeError('set_pose failed: ' + result.stdout + result.stderr)


def run_trial(trial_id, controller_config, height, out_dir, domain_id):
    os.environ['ROS_DOMAIN_ID'] = str(domain_id)
    os.environ['GZ_PARTITION'] = f'tap_landing_{trial_id}_{os.getpid()}_{time.time_ns()}'
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from owned_process import start_owned_process, cleanup_group
    from tap_launch import sim_launch_command
    import rclpy
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
        pose.update(z=t.translation.z,
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
    node = rclpy.create_node(f'tap_landing_smoke_{trial_id}'.replace('.', '_'))
    node.create_subscription(Clock, '/clock', clock_cb, 100)
    node.create_subscription(TFMessage, '/tap_robot/pose', pose_cb, 100)
    node.create_subscription(String, '/control_mode', status_cb, 100)

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
        args = sim_launch_command('gui:=false', 'auto_enable:=true')
        if controller_config:
            args.append(f'controller_config:={controller_config}')
        process = start_owned_process(args, cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT)
        if not wait_for_mode('Balance', 90):
            raise RuntimeError(f'Startup did not reach Balance: mode={state.get("mode")}')
        observe(1.0)

        gz_set_pose(0.0, 0.0, STAND_Z + height)
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
                       p < RECOVER_PITCH_RAD and pr < RECOVER_PITCH_RATE_RAD_S and
                       pose.get('z', 0.0) > STAND_Z - .05)
            stable = stable + dt if upright else 0.0
            elapsed = sim_time - drop_start
            if stable >= RECOVER_HOLD_S and elapsed > 0.3:
                recovered = True
                break
            if elapsed > 8.0:
                break
        trial_result = dict(
            trial=trial_id, height_m=height, launched=True, recovered=recovered,
            peak_pitch_rad=round(peak_pitch, 4),
            peak_pitch_rate_rad_s=round(peak_pitch_rate, 4),
            final_mode=state.get('mode'), final_pitch_rad=state.get('pitch'),
            final_contact=state.get('contact'),
            elapsed_sim_s=round(sim_time - drop_start, 3))
        print(f'[{trial_id}] recovered={recovered} peak_pitch={peak_pitch:.3f} '
              f'final_mode={state.get("mode")}', flush=True)
    except Exception as exc:
        failure = str(exc)
        trial_result = dict(trial=trial_id, height_m=height, launched=False, reason=failure)
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
    by_height = {}
    for row in trials:
        h = row['height_m']
        by_height.setdefault(h, {'launched': 0, 'recovered': 0, 'peak_pitch_max': 0.0})
        if not row.get('launched'):
            continue
        entry = by_height[h]
        entry['launched'] += 1
        entry['recovered'] += 1 if row.get('recovered') else 0
        entry['peak_pitch_max'] = max(entry['peak_pitch_max'], row.get('peak_pitch_rad', 0.0))
    return by_height


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--heights', default='0.05,0.10,0.20',
                        help='comma-separated drop heights in metres')
    parser.add_argument('--trials', type=int, default=1,
                        help='fresh sim launches per height per condition (each is a full '
                             'startup, ~60-90s wall time; keep small unless you have time budget)')
    parser.add_argument('--skip-baseline', action='store_true')
    parser.add_argument('--skip-enabled', action='store_true')
    args = parser.parse_args()
    heights = [float(h) for h in args.heights.split(',') if h]

    out_dir = ROOT / 'log' / f'landing_ledge_{int(time.time())}'
    out_dir.mkdir(parents=True)

    conditions = {}
    domain = 161
    plan = []
    if not args.skip_baseline:
        plan.append(('baseline', None))
    if not args.skip_enabled:
        plan.append(('enabled', LANDING_CONFIG))

    for label, cfg in plan:
        print(f'=== {label} ({cfg or "stock balance_controller.yaml"}) ===', flush=True)
        trials = []
        for height in heights:
            for trial in range(args.trials):
                trial_id = f'{label}_h{height:.2f}_{trial}'
                domain += 1
                result = run_trial(trial_id, cfg, height, out_dir, domain)
                trials.append(result)
                (out_dir / 'result.json').write_text(json.dumps(
                    {'heights': heights, 'trials_per_height': args.trials,
                     'conditions': conditions | {label: {'trials': trials}}}, indent=2))
        conditions[label] = {'controller_config': str(cfg) if cfg else None, 'trials': trials}

    summary = {label: summarize(c['trials']) for label, c in conditions.items()}
    result = {'heights': heights, 'trials_per_height': args.trials,
              'conditions': conditions, 'summary': summary}
    (out_dir / 'result.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(summary, indent=2))
    print('Evidence:', out_dir)

    any_launch_failure = any(
        not row.get('launched') for c in conditions.values() for row in c['trials'])
    return 1 if any_launch_failure else 0


if __name__ == '__main__':
    raise SystemExit(main())
