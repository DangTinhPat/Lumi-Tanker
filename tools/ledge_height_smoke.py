#!/usr/bin/env python3
"""Landing/touchdown-balance recovery test swept across ledge heights.

Same isolated-world methodology as clean_ledge_smoke.py (ground_plane plus
one static ledge_pad, nothing else nearby - see that tool's docstring for
why an isolated world matters), generalized to an arbitrary height via
tools/generate_ledge_world.py instead of the hardcoded 20 cm
ledge_test.sdf. One fresh owned sim launch per trial. The command remains
active after the edge so a recovered trial must actually resume driving.
"""
import argparse
import csv
import json
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
DEFAULT_DRIVE_SPEED_M_S = 1.0
RECOVER_PITCH_RAD = .22
RECOVER_PITCH_RATE_RAD_S = .55
RECOVER_HOLD_S = 2.0
SUPPORT_PERSISTENCE_S = 0.10
DEFAULT_STARTUP_TIMEOUT_S = 25.0
DEFAULT_SETTLE_S = 5.0
DEFAULT_DRIVE_TIMEOUT_S = 8.0
DEFAULT_RECOVERY_TIMEOUT_S = 8.0


def run_trial(trial_id, height_m, world_path, controller_config, out_dir, domain_id,
              trace_rate_hz, drive_speed_m_s, startup_timeout_s, settle_s,
              drive_timeout_s, recovery_timeout_s, gui, stop_after_recovery,
              post_recovery_stop_s):
    os.environ['ROS_DOMAIN_ID'] = str(domain_id)
    os.environ['GZ_PARTITION'] = f'tap_ledge_h_{trial_id}_{os.getpid()}_{time.time_ns()}'
    from owned_process import start_owned_process, cleanup_group
    from tap_launch import sim_launch_command
    import rclpy
    from geometry_msgs.msg import Twist
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64MultiArray
    from std_msgs.msg import String

    log_path = out_dir / f'launch_{trial_id}.log'
    trace_path = out_dir / f'trace_{trial_id}.csv'
    stream = log_path.open('w')
    # The evaluator is deliberately driven by the same observable contract as
    # the controller.  It does not subscribe to model pose; Gazebo pose is
    # neither a control input nor an acceptance signal.
    state = {}
    joints, motors, leg_effort = {}, {}, {}
    sim_time = 0.0
    trace_last_s = -1.0
    trace_period_s = 1.0 / trace_rate_hz if trace_rate_hz > 0 else 0.0
    trace_stream = trace_path.open('w', newline='') if trace_period_s > 0 else None
    trace_fields = [
        'sim_time', 'wall_time', 'mode', 'pose_x', 'pose_z', 'pose_pitch', 'pose_roll',
        'pitch', 'pitch_rate', 'cmd_v', 'contact', 'accel_mag', 'roll_rate',
        'wheel_pos', 'wheel_vel', 'mpc_cmd', 'mpc_cmd_v', 'pitch_ref', 'wb_h',
        'flight_phase', 'flight_airborne', 'flight_ballistic', 'flight_touchdown',
        'freefall_conf', 'collision_conf', 'l_qd_cmd', 'r_qd_cmd',
        'obs_valid', 'obs_x', 'obs_y', 'obs_v', 'obs_w', 'obs_yaw', 'obs_conf',
        'landing_v', 'landing_v_observer',
        'actual_l_hip', 'actual_l_knee', 'actual_r_hip', 'actual_r_knee',
        'actual_l_hip_vel', 'actual_l_knee_vel', 'actual_r_hip_vel', 'actual_r_knee_vel',
        'target_l_hip', 'target_l_knee', 'target_r_hip', 'target_r_knee',
        'target_l_wheel_vel', 'target_r_wheel_vel',
        'l_knee_kp_scale', 'landing_phase', 'landing_ff_lh', 'landing_ff_lk',
        'landing_ff_rh', 'landing_ff_rk',
        'leg_effort_l_hip', 'leg_effort_l_knee',
        'leg_effort_r_hip', 'leg_effort_r_knee']
    trace_writer = csv.DictWriter(trace_stream, fieldnames=trace_fields) if trace_stream else None
    if trace_writer:
        trace_writer.writeheader()

    def clock_cb(msg):
        nonlocal sim_time
        sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def joint_cb(msg):
        by_name = {name: i for i, name in enumerate(msg.name)}
        def value(values, name):
            idx = by_name.get(name)
            return values[idx] if idx is not None and idx < len(values) else None
        joints.update(
            l_hip=value(msg.position, 'cad_left_hip'),
            l_knee=value(msg.position, 'cad_left_knee'),
            r_hip=value(msg.position, 'cad_right_hip'),
            r_knee=value(msg.position, 'cad_right_knee'),
            l_hip_vel=value(msg.velocity, 'cad_left_hip'),
            l_knee_vel=value(msg.velocity, 'cad_left_knee'),
            r_hip_vel=value(msg.velocity, 'cad_right_hip'),
            r_knee_vel=value(msg.velocity, 'cad_right_knee'))

    def motor_cb(msg):
        by_name = {name: i for i, name in enumerate(msg.name)}
        def value(values, name):
            idx = by_name.get(name)
            return values[idx] if idx is not None and idx < len(values) else None
        motors.update(
            l_hip=value(msg.position, 'cad_left_hip'),
            l_knee=value(msg.position, 'cad_left_knee'),
            r_hip=value(msg.position, 'cad_right_hip'),
            r_knee=value(msg.position, 'cad_right_knee'),
            l_wheel_vel=value(msg.velocity, 'cad_left_wheel'),
            r_wheel_vel=value(msg.velocity, 'cad_right_wheel'))

    def leg_effort_cb(msg):
        if len(msg.data) >= 4:
            leg_effort.update(l_hip=msg.data[0], l_knee=msg.data[1],
                              r_hip=msg.data[2], r_knee=msg.data[3])

    def write_trace():
        nonlocal trace_last_s
        if not trace_writer or sim_time <= 0.0:
            return
        if trace_last_s >= 0.0 and sim_time - trace_last_s < trace_period_s:
            return
        trace_last_s = sim_time
        trace_writer.writerow({
            'sim_time': f'{sim_time:.6f}', 'wall_time': f'{time.monotonic():.6f}',
            'mode': state.get('mode'), 'pose_x': None, 'pose_z': None,
            'pose_pitch': None, 'pose_roll': None,
            'pitch': state.get('pitch'), 'pitch_rate': state.get('pitch_rate'),
            'cmd_v': state.get('cmd_v'), 'contact': state.get('contact'),
            'accel_mag': state.get('accel_mag'), 'roll_rate': state.get('roll_rate'),
            'wheel_pos': state.get('wheel_pos'), 'wheel_vel': state.get('wheel_vel'),
            'mpc_cmd': state.get('mpc_cmd'), 'mpc_cmd_v': state.get('mpc_cmd_v'),
            'pitch_ref': state.get('pitch_ref'), 'wb_h': state.get('wb_h'),
            'flight_phase': state.get('flight_phase'),
            'flight_airborne': state.get('flight_airborne'),
            'flight_ballistic': state.get('flight_ballistic'),
            'flight_touchdown': state.get('flight_touchdown'),
            'freefall_conf': state.get('freefall_conf'),
            'collision_conf': state.get('collision_conf'),
            'obs_valid': state.get('obs_valid'), 'obs_x': state.get('obs_x'),
            'obs_y': state.get('obs_y'), 'obs_v': state.get('obs_v'),
            'obs_w': state.get('obs_w'), 'obs_yaw': state.get('obs_yaw'),
            'obs_conf': state.get('obs_conf'),
            'landing_v': state.get('landing_v'),
            'landing_v_observer': state.get('landing_v_observer'),
            'l_qd_cmd': state.get('l_qd_cmd'), 'r_qd_cmd': state.get('r_qd_cmd'),
            'actual_l_hip': joints.get('l_hip'), 'actual_l_knee': joints.get('l_knee'),
            'actual_r_hip': joints.get('r_hip'), 'actual_r_knee': joints.get('r_knee'),
            'actual_l_hip_vel': joints.get('l_hip_vel'),
            'actual_l_knee_vel': joints.get('l_knee_vel'),
            'actual_r_hip_vel': joints.get('r_hip_vel'),
            'actual_r_knee_vel': joints.get('r_knee_vel'),
            'target_l_hip': motors.get('l_hip'), 'target_l_knee': motors.get('l_knee'),
            'target_r_hip': motors.get('r_hip'), 'target_r_knee': motors.get('r_knee'),
            'target_l_wheel_vel': motors.get('l_wheel_vel'),
            'target_r_wheel_vel': motors.get('r_wheel_vel'),
            'l_knee_kp_scale': state.get('l_knee_kp_scale'),
            'landing_phase': state.get('landing_phase'),
            'landing_ff_lh': state.get('landing_ff_lh'),
            'landing_ff_lk': state.get('landing_ff_lk'),
            'landing_ff_rh': state.get('landing_ff_rh'),
            'landing_ff_rk': state.get('landing_ff_rk'),
            'leg_effort_l_hip': leg_effort.get('l_hip'),
            'leg_effort_l_knee': leg_effort.get('l_knee'),
            'leg_effort_r_hip': leg_effort.get('r_hip'),
            'leg_effort_r_knee': leg_effort.get('r_knee')})

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
        write_trace()

    rclpy.init()
    node = rclpy.create_node(f'tap_ledge_h_{trial_id}'.replace('.', '_').replace('-', 'n'))
    node.create_subscription(Clock, '/clock', clock_cb, 100)
    node.create_subscription(String, '/control_mode', status_cb, 100)
    node.create_subscription(JointState, '/joint_states', joint_cb, 100)
    node.create_subscription(JointState, '/motor_commands', motor_cb, 100)
    node.create_subscription(Float64MultiArray, '/leg_effort_controller/commands',
                             leg_effort_cb, 100)
    drive_pub = node.create_publisher(Twist, '/cmd_vel', 10)

    process = None
    cleanup_result = None
    failure = None
    failure_reason = None
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
        # The landing acceptance path is sensor-only. Keep Gazebo's physical
        # collision response, but do not publish wheel-contact ground truth;
        # FlightObserver must infer support from MPU6500 + encoders.
        args = sim_launch_command(f'gui:={str(gui).lower()}', 'auto_enable:=true',
                                   f'ledge_world:={world_path}', f'spawn_z:={spawn_z}')
        if controller_config:
            args.append(f'controller_config:={controller_config}')
        process = start_owned_process(args, cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT)
        if not wait_for_mode('Balance', startup_timeout_s):
            raise RuntimeError(f'Startup did not reach Balance: mode={state.get("mode")}')
        observe(settle_s, wall_limit=max(5.0, startup_timeout_s))
        # The ledge geometry and commanded speed define a test schedule; the
        # robot itself never receives the ledge coordinate.  Allow margin for
        # motor/controller acceleration, then begin the landing observation.
        drive_duration_s = STOP_DRIVE_X / max(abs(drive_speed_m_s), 0.1) + 0.75
        crossed_edge = False
        drive_start = sim_time
        drive_deadline = time.monotonic() + drive_timeout_s
        drive_cmd = Twist()
        drive_cmd.linear.x = drive_speed_m_s
        while time.monotonic() < drive_deadline and sim_time - drive_start < drive_duration_s:
            drive_pub.publish(drive_cmd)
            spin()
        crossed_edge = sim_time - drive_start >= drive_duration_s
        if not crossed_edge:
            raise RuntimeError('Drive schedule timed out before the landing interval')

        drop_start = sim_time
        peak_pitch, peak_pitch_rate = 0.0, 0.0
        peak_roll = 0.0
        min_z = None
        support_persisted = 0.0
        stable = 0.0
        motion_stable = 0.0
        recovered = False
        previous = sim_time
        wall_deadline = time.monotonic() + recovery_timeout_s
        while time.monotonic() < wall_deadline:
            # Keep the original command alive through flight, touchdown and
            # handoff. Publishing zero here would test "stop and recover" and
            # would also allow cmd_vel_timeout_s to silently remove the drive
            # request before the post-handoff criterion is evaluated.
            drive_pub.publish(drive_cmd)
            spin()
            dt = max(0.0, sim_time - previous)
            previous = sim_time
            if state.get('mode') == 'Fallen':
                failure_reason = 'mode_fallen'
                break
            p = abs(state.get('pitch', 9.0))
            pr = abs(state.get('pitch_rate', 9.0))
            peak_pitch = max(peak_pitch, p)
            peak_pitch_rate = max(peak_pitch_rate, pr)
            phase = state.get('flight_phase')
            sensor_support = (phase in ('Grounded', 'PartialSupport') and
                              state.get('flight_airborne') == 0)
            support_persisted = (support_persisted + dt if sensor_support else 0.0)
            wheel_balance_active = (state.get('mode') in ('Balance', 'Drive') and
                                    state.get('l_qd_cmd') is not None and
                                    state.get('r_qd_cmd') is not None)
            upright = (wheel_balance_active and
                       support_persisted >= SUPPORT_PERSISTENCE_S and
                       p <= RECOVER_PITCH_RAD and pr <= RECOVER_PITCH_RATE_RAD_S)
            stable = stable + dt if upright else 0.0
            actual_v = state.get('wheel_vel')
            velocity_tracking = (actual_v is not None and
                                 0.8 * drive_speed_m_s <= actual_v <=
                                 1.2 * drive_speed_m_s)
            motion_stable = motion_stable + dt if upright and velocity_tracking else 0.0
            elapsed = sim_time - drop_start
            if (stable >= RECOVER_HOLD_S and motion_stable >= RECOVER_HOLD_S and
                    elapsed > 0.3):
                recovered = True
                if stop_after_recovery:
                    # Observation mode: once the landing/handoff contract has
                    # already passed, remove the drive request and hold the
                    # robot still so a GUI observer can inspect the settled
                    # posture. The default canonical test keeps driving.
                    stop_cmd = Twist()
                    stop_deadline = time.monotonic() + post_recovery_stop_s
                    while time.monotonic() < stop_deadline:
                        drive_pub.publish(stop_cmd)
                        spin()
                break
        if not recovered and failure_reason is None:
            failure_reason = 'recovery_timeout'
        trial_result = dict(
            trial=trial_id, height_m=height_m, launched=True, crossed_edge=crossed_edge,
            drive_speed_m_s=drive_speed_m_s,
            recovered=recovered,
            failure_reason=failure_reason,
            support_persistence_s=round(support_persisted, 3),
            handoff_stable_s=round(stable, 3),
            velocity_tracking_s=round(motion_stable, 3),
            peak_pitch_rad=round(peak_pitch, 4),
            peak_pitch_rate_rad_s=round(peak_pitch_rate, 4),
            peak_roll_rad=round(peak_roll, 4),
            min_z_m=None,
            final_mode=state.get('mode'), final_pitch_rad=state.get('pitch'),
            final_x=None, final_z=None,
            elapsed_sim_s=round(sim_time - drop_start, 3))
        print(f'[{trial_id}] h={height_m:.2f}m crossed_edge={crossed_edge} '
              f'recovered={recovered} peak_pitch={peak_pitch:.3f} peak_roll={peak_roll:.3f} '
              f'min_z=unobserved final_mode={state.get("mode")}', flush=True)
    except Exception as exc:
        failure = str(exc)
        trial_result = dict(trial=trial_id, height_m=height_m, launched=False,
                            failure_reason=failure)
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
        if trace_stream:
            trace_stream.close()

    trial_result['cleanup'] = cleanup_result
    trial_result['launch_log'] = str(log_path)
    if trace_stream:
        trial_result['trace_csv'] = str(trace_path)
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
    parser.add_argument('--trace-rate-hz', type=float, default=100.0,
                        help='write per-trial CSV telemetry at this max rate; 0 disables')
    parser.add_argument('--gui', action='store_true',
                        help='keep Gazebo GUI visible during the landing trial')
    parser.add_argument('--stop-after-recovery', action='store_true',
                        help='stop publishing forward motion after recovery for GUI observation')
    parser.add_argument('--post-recovery-stop-s', type=float, default=3.0,
                        help='hold zero velocity after recovery in observation mode (default: 3.0 s)')
    parser.add_argument('--drive-speed', type=float, default=DEFAULT_DRIVE_SPEED_M_S,
                        help='forward speed command before the ledge, in m/s')
    parser.add_argument('--startup-timeout-s', type=float, default=DEFAULT_STARTUP_TIMEOUT_S,
                        help='fail-fast wall timeout to reach Balance (default: 25 s)')
    parser.add_argument('--settle-s', type=float, default=DEFAULT_SETTLE_S,
                        help='sim-time settle after Balance before driving (default: 5.0 s)')
    parser.add_argument('--drive-timeout-s', type=float, default=DEFAULT_DRIVE_TIMEOUT_S,
                        help='wall timeout to cross the ledge edge (default: 8 s)')
    parser.add_argument('--recovery-timeout-s', type=float, default=DEFAULT_RECOVERY_TIMEOUT_S,
                        help='wall timeout for landing/handoff (default: 8 s)')
    args = parser.parse_args()

    if (args.trials < 1 or args.trace_rate_hz < 0.0 or args.drive_speed <= 0.0 or
            args.startup_timeout_s <= 0.0 or args.settle_s < 0.0 or
            args.drive_timeout_s <= 0.0 or args.recovery_timeout_s <= 0.0 or
            args.post_recovery_stop_s < 0.0):
        parser.error('trials/timeouts must be positive, settle must be nonnegative, '
                     'trace rate must be nonnegative, and drive speed must be positive')

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
                                out_dir, domain, args.trace_rate_hz, args.drive_speed,
                                args.startup_timeout_s, args.settle_s,
                                args.drive_timeout_s, args.recovery_timeout_s, args.gui,
                                args.stop_after_recovery, args.post_recovery_stop_s)
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
