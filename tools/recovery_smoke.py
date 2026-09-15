#!/usr/bin/env python3
"""One isolated fallen-recovery trial; setup pose is never reset during recovery."""
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool, Float64MultiArray, String
from tf2_msgs.msg import TFMessage

ROOT = Path(__file__).resolve().parents[1]


def main():
    from owned_process import cleanup_group, start_owned_process
    os.environ['ROS_DOMAIN_ID'] = '109'
    os.environ['GZ_PARTITION'] = f'tap_recovery_{os.getpid()}_{time.time_ns()}'
    out = ROOT / 'log' / f'recovery_{int(time.time())}'
    out.mkdir(parents=True)
    rows, pose, state = [], {}, {}
    stage = 'startup'
    sim_time = 0.0
    failure = None
    stable = 0.0
    motor_commands = {}
    motor_times = {}

    def clock(msg):
        nonlocal sim_time
        sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def position(msg):
        if len(msg.transforms) != 1:
            return
        t = msg.transforms[0].transform
        q = t.rotation
        pose.update(z=t.translation.z,
                    actual_pitch=math.asin(max(-1, min(1, 2*(q.w*q.y-q.z*q.x)))),
                    actual_roll=math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y)))

    def status(msg):
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
        rows.append(dict(state, **pose, stage=stage, sim_time=sim_time))

    def motors(name, msg):
        motor_commands[name] = list(msg.data)
        motor_times[name] = sim_time

    log = (out / 'launch.log').open('w')
    binary = ROOT / 'install/tap_control/lib/tap_control/balance_controller_node'
    metadata = {'domain': 109, 'partition': os.environ['GZ_PARTITION'],
                'binary_sha256': hashlib.sha256(binary.read_bytes()).hexdigest(),
                'contact_note': 'contact is controller-inferred, not a contact-force sensor',
                'setup': 'estop, pose pitch=1.2 z=0.25, gravity settle; no later pose resets'}
    process = None
    cleanup_result = None
    rclpy.init()
    node = rclpy.create_node('tap_recovery_smoke')
    node.create_subscription(Clock, '/clock', clock, 100)
    node.create_subscription(TFMessage, '/tap_robot/pose', position, 100)
    node.create_subscription(String, '/control_mode', status, 100)
    node.create_subscription(Float64MultiArray, '/wheel_effort_controller/commands',
                             lambda msg: motors('wheels', msg), 100)
    node.create_subscription(Float64MultiArray, '/leg_effort_controller/commands',
                             lambda msg: motors('legs', msg), 100)
    pubs = {name: node.create_publisher(Bool, '/' + name, 10)
            for name in ('enable', 'emergency_stop', 'reset_controller', 'stand_up')}

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

    def send(name, value):
        pubs[name].publish(Bool(data=value))
        observe(.1)

    try:
        process = start_owned_process([str(ROOT / 'run_sim.sh'), 'gui:=false', 'auto_enable:=true'],
                                      cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
        deadline = time.monotonic() + 70
        while state.get('mode') != 'Balance' and time.monotonic() < deadline:
            spin()
        if state.get('mode') != 'Balance':
            raise RuntimeError('Startup did not reach Balance')
        stage = 'upright'
        observe(1)
        stage = 'estop_setup'
        send('emergency_stop', True)
        send('enable', False)
        if state.get('estop') != 1:
            raise RuntimeError('E-stop not observed')
        result = subprocess.run(['gz', 'service', '-s', '/world/empty/set_pose',
                                 '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                                 '--timeout', '3000', '--req',
                                 f'name: "tap_robot" position {{z: 0.25}} orientation {{w: {math.cos(.6)} y: {math.sin(.6)}}}'],
                                capture_output=True, text=True, timeout=5)
        if result.returncode or 'data: true' not in result.stdout:
            raise RuntimeError('Fallen setup failed: ' + result.stdout + result.stderr)
        stage = 'fallen_settle'
        observe(3)
        metadata['settled_fallen'] = dict(state, **pose)
        if abs(pose.get('actual_pitch', 0)) < .7 and abs(pose.get('actual_roll', 0)) < .7:
            raise RuntimeError('Setup did not produce a fallen robot')
        stage = 'clear_and_reset'
        send('emergency_stop', False)
        send('enable', False)
        send('reset_controller', True)
        send('reset_controller', False)
        if state.get('estop') != 0 or state.get('mode') not in ('Disabled', 'Fallen'):
            raise RuntimeError(f'Clear/reset not safely stopped: {state.get("mode")} estop={state.get("estop")}')
        for name, count in (('wheels', 2), ('legs', 4)):
            command = motor_commands.get(name, [])
            if (len(command) != count or sim_time - motor_times.get(name, -1) > .05
                    or any(not math.isfinite(v) or abs(v) > 1e-9 for v in command)):
                raise RuntimeError(f'Clear/reset motor commands not fresh and zero: {name}={command}')
        metadata['reset_gate'] = dict(mode=state['mode'], estop=state['estop'],
                                      motor_commands={k: list(v) for k, v in motor_commands.items()})
        send('stand_up', False)
        stage = 'recovery'
        send('stand_up', True)
        send('stand_up', False)
        start, previous, deadline = sim_time, sim_time, time.monotonic() + 45
        while sim_time - start < 10 and time.monotonic() < deadline:
            spin()
            dt = max(0, sim_time - previous)
            previous = sim_time
            upright = (state.get('mode') == 'Balance' and abs(pose.get('actual_pitch', 9)) < .2
                       and abs(pose.get('actual_roll', 9)) < .2 and pose.get('z', 0) > .20
                       and abs(state.get('pitch_rate', 9)) < .5 and state.get('contact') == 1)
            stable = stable + dt if upright else 0
            if stable >= 2:
                break
        if stable < 2:
            raise RuntimeError('No sustained 2s upright recovery within observation window')
    except Exception as exc:
        failure = str(exc)
    finally:
        try:
            cleanup_result = cleanup_group(process)
        except Exception as exc:
            failure = failure or f'Cleanup: {exc}'
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        log.close()
        metadata.update(failure=failure, cleanup=cleanup_result, stable_upright_s=stable, final=dict(state, **pose), stages={})
        for name in dict.fromkeys(r['stage'] for r in rows):
            subset = [r for r in rows if r['stage'] == name]
            metadata['stages'][name] = {
                'sim_duration': subset[-1]['sim_time'] - subset[0]['sim_time'],
                'modes': sorted({r['mode'] for r in subset}),
                'pitch_range': [min(r.get('actual_pitch', 0) for r in subset), max(r.get('actual_pitch', 0) for r in subset)],
                'height_range': [min(r.get('z', 0) for r in subset), max(r.get('z', 0) for r in subset)],
                'inferred_contact_values': sorted({r.get('contact', -1) for r in subset})}
        (out / 'samples.json').write_text(json.dumps(rows, indent=2))
        (out / 'summary.json').write_text(json.dumps(metadata, indent=2))
        print(json.dumps(metadata, indent=2))
        print('Evidence:', out)
    return int(failure is not None)


if __name__ == '__main__':
    raise SystemExit(main())
