#!/usr/bin/env python3
"""Exercise the real Tk/ROS dashboard against an owned Gazebo simulation."""
import argparse
import json
import math
import os
from pathlib import Path
import time
import tkinter as tk

from owned_process import start_owned_process, cleanup_group


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--cycles', type=int, default=2)
    parser.add_argument('--drag', action='store_true')
    args = parser.parse_args()
    if args.cycles < 1:
        parser.error('cycles must be positive')
    workspace = Path(__file__).resolve().parents[1]
    os.environ.update(ROS_DOMAIN_ID='127', GZ_PARTITION=f'gui_qa_{os.getpid()}',
                      GZ_IP='127.0.0.1')
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from tf2_msgs.msg import TFMessage
    from gui.dashboard import Dashboard
    from tap_teleop.pose_curves import generate
    from tap_teleop.deep_squat import generate_deep_squat

    log = workspace / 'log' / f'gui_deep_{time.time_ns()}'
    log.mkdir()
    xacro = workspace / 'src/tap_robot_description/description/cad_robot/robot.urdf.xacro'
    curves = generate(xacro)
    deep = generate_deep_squat(xacro, knee_max=1.6, hip_bounds=(-.6, 1.05))
    rclpy.init()
    root = tk.Tk()
    node = rclpy.create_node('gui_deep_qa')
    dashboard = Dashboard(root, node, curves, deep_curves=deep)
    observations = []
    latest = {}

    def on_pose(msg):
        if len(msg.transforms) != 1:
            return
        t = msg.transforms[0].transform
        p, q = t.translation, t.rotation
        latest.update(x=p.x, y=p.y, z=p.z,
                      pitch=math.asin(max(-1., min(1., 2*(q.w*q.y-q.z*q.x)))))

    node.create_subscription(TFMessage, '/tap_robot/pose', on_pose, qos_profile_sensor_data)
    stream = (log / 'sim.log').open('w')
    process = start_owned_process([str(workspace/'run_sim.sh'),
        f'gui:={str(args.gui).lower()}', 'jump_model:=true', 'jump_leg_velocity_damping:=0.6',
        f'controller_config:={workspace}/src/tap_control/config/precision_pose.yaml'],
        cwd=workspace, stdout=stream, stderr=stream)
    stage, epoch, cycle = 'startup', 0., 0
    start = time.monotonic()
    result = {'passed': False}
    anchor = None

    def finish(reason):
        result['reason'] = reason
        result['passed'] = reason == 'complete'
        dashboard.disable()
        root.quit()

    def monitor():
        nonlocal stage, epoch, cycle, anchor
        try:
            now = time.monotonic()
            sim = dashboard.sim_time
            if process.poll() is not None:
                return finish('simulator exited')
            if now-start > 240:
                return finish('wall timeout')
            if stage == 'startup':
                if 'Controllers ready; paused.' in (log/'sim.log').read_text():
                    dashboard.enable()
                    stage = 'settle'
                    epoch = sim or 0.
            elif sim is not None:
                if stage == 'settle' and sim-epoch >= 5:
                    dashboard.experimental.set(True)
                    dashboard.workflow.set('Deep squat')
                    dashboard.change_workflow()
                    if dashboard.deep_trajectory is None:
                        return finish('entry blocked: '+dashboard.deep_text)
                    if anchor is None:
                        anchor = (latest['x'], latest['y'])
                    dashboard.depth.set(1.)
                    stage, epoch = 'lower', sim
                elif stage != 'settle':
                    row = dict(latest, time=sim, stage=stage, mode=dashboard.mode_text,
                               target=list(dashboard.pose), measured=dashboard.measured_pose,
                               held=dashboard.deep_paused, detail=dashboard.deep_text)
                    observations.append(row)
                    if not dashboard.enabled or not dashboard.controller_fresh:
                        return finish('controller disarmed or stale: '+dashboard.mode_text)
                    if abs(latest['pitch']) > .3 or latest['z'] < .115:
                        return finish('posture bound')
                    if math.hypot(latest['x']-anchor[0], latest['y']-anchor[1]) > .01:
                        return finish('drift bound')
                    if dashboard.deep_paused and dashboard.deep_trajectory is not None:
                        return finish('trajectory held: '+dashboard.deep_text)
                    if stage == 'lower' and args.drag:
                        dashboard.depth.set(.25 if 4 <= sim-epoch < 6 else 1.)
                    if stage == 'lower' and sim-epoch >= (21 if args.drag else 15):
                        if max(dashboard.measured_pose[i] for i in (1, 3)) > -1.55:
                            return finish('full depth not reached')
                        dashboard.neutral()
                        stage, epoch = 'return', sim
                    elif stage == 'return' and sim-epoch >= 16:
                        if dashboard.deep_trajectory is not None:
                            return finish('return incomplete')
                        cycle += 1
                        if cycle == args.cycles:
                            if math.hypot(latest['x']-anchor[0], latest['y']-anchor[1]) > .002:
                                return finish('residual drift bound')
                            return finish('complete')
                        stage, epoch = 'settle', sim
            root.after(50, monitor)
        except Exception as exc:
            finish(f'{type(exc).__name__}: {exc}')

    root.after(100, monitor)
    print(log, flush=True)
    try:
        root.mainloop()
    finally:
        root.destroy()
        node.destroy_node()
        rclpy.shutdown()
        result['cleanup'] = cleanup_group(process)
        stream.close()
        if observations:
            result.update(samples=len(observations), cycles=cycle,
                min_z=min(r['z'] for r in observations),
                max_pitch=max(abs(r['pitch']) for r in observations),
                max_xy=max(math.hypot(r['x']-observations[0]['x'],
                                      r['y']-observations[0]['y']) for r in observations))
        (log/'trace.json').write_text(json.dumps(observations, indent=2))
        (log/'result.json').write_text(json.dumps(result, indent=2))
        print(json.dumps(result, indent=2), flush=True)
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
