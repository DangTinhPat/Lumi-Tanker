#!/usr/bin/env python3
"""Exercise the real Reset standup sequence against an owned Gazebo simulation.

Follows tools/gui_deep_smoke.py's template: an isolated ROS domain/Gazebo
partition, the actual Dashboard driven headlessly through its own Tk timer.
Forces a fallen precondition using the same literal set_pose call as
tools/recovery_smoke.py, then drives Dashboard.reset_standup() and asserts it
reaches 'done' with a sustained upright Balance — twice in the same sim
session, to prove the "no reopen needed" requirement this feature exists for.

This proves the button and gz choreography work in Gazebo; it is QA evidence
for the GUI/sim wiring, not evidence of physical recovery capability.
"""
import json
import math
import os
from pathlib import Path
import subprocess
import time
import tkinter as tk

from owned_process import start_owned_process, cleanup_group

ROOT = Path(__file__).resolve().parents[1]
# A moderate tip (0.9 rad, just over the 0.8 rad mpc.fall_pitch_rad threshold
# in balance_controller.yaml) rather than recovery_smoke.py's much harder
# 1.2 rad near-full topple: this feature targets recovering from ordinary
# testing mishaps, not the hardest fall case (which docs/ALGORITHM_MIGRATION.md
# already records as an unsolved, separate recovery problem).
FALL_ANGLE_RAD = .9
FALLEN_REQUEST = ('name: "tap_robot" position {z: 0.25} '
                  f'orientation {{w: {math.cos(FALL_ANGLE_RAD/2)} y: {math.sin(FALL_ANGLE_RAD/2)}}}')


def set_fallen_pose():
    result = subprocess.run(
        ['gz', 'service', '-s', '/world/empty/set_pose', '--reqtype', 'gz.msgs.Pose',
         '--reptype', 'gz.msgs.Boolean', '--timeout', '3000', '--req', FALLEN_REQUEST],
        capture_output=True, text=True, timeout=5)
    if result.returncode or 'data: true' not in result.stdout:
        raise RuntimeError('Fallen setup failed: ' + result.stdout + result.stderr)


def main():
    os.environ.update(ROS_DOMAIN_ID='131', GZ_PARTITION=f'tap_reset_standup_qa_{os.getpid()}',
                      GZ_IP='127.0.0.1')
    import rclpy
    from std_msgs.msg import Bool
    from gui.dashboard import Dashboard
    from tap_teleop.pose_curves import generate

    log = ROOT / 'log' / f'gui_reset_standup_{time.time_ns()}'
    log.mkdir()
    xacro = ROOT / 'src/tap_robot_description/description/cad_robot/robot.urdf.xacro'
    curves = generate(xacro)
    rclpy.init()
    root = tk.Tk()
    node = rclpy.create_node('gui_reset_standup_qa')
    dashboard = Dashboard(root, node, curves)
    dashboard.experimental.set(True)

    stream = (log / 'sim.log').open('w')
    process = start_owned_process(
        [str(ROOT / 'run_sim.sh'), 'gui:=false', 'auto_enable:=true'],
        cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT)

    result = {'passed': False, 'cycles': []}
    stage = {'name': 'startup', 'cycle': 0, 'deadline': time.monotonic() + 60}

    def finish(reason):
        result['reason'] = reason
        result['passed'] = reason == 'complete'
        dashboard.disable()
        root.quit()

    trace = {'last': None, 'last_at': 0.}

    def pump():
        try:
            if process.poll() is not None:
                return finish('simulator exited')
            if time.monotonic() > stage['deadline']:
                return finish(f"timeout in stage {stage['name']}, "
                             f"reset_standup_stage={dashboard.reset_standup_stage}, "
                             f"mode={dashboard.mode_text}, pitch={dashboard.last_pitch}, "
                             f"pose_commands_blocked={dashboard.pose_commands_blocked}")
            rclpy.spin_once(node, timeout_sec=0.)
            marker = (stage['name'], dashboard.reset_standup_stage, dashboard.mode_text)
            if marker != trace['last'] or time.monotonic() - trace['last_at'] > 5:
                print(f"[trace t={time.monotonic():.1f}] stage={stage['name']} "
                     f"reset_standup_stage={dashboard.reset_standup_stage} "
                     f"mode={dashboard.mode_text} pitch={dashboard.last_pitch} "
                     f"pose_z={dashboard.last_pose_z} "
                     f"pose_commands_blocked={dashboard.pose_commands_blocked} "
                     f"measured_pose={dashboard.measured_pose}", flush=True)
                trace['last'], trace['last_at'] = marker, time.monotonic()
            if stage['name'] == 'startup':
                if 'Enabled and running after' in (log / 'sim.log').read_text(errors='replace'):
                    stage.update(name='fall', deadline=time.monotonic() + 20)
            elif stage['name'] == 'fall':
                # Estop before the forced teleport, matching recovery_smoke.py's
                # own fallen-setup order: otherwise the still-active balance
                # controller fights the fake fall with real torque, producing a
                # far more violent/chaotic precondition than an actual fall.
                dashboard.pub['emergency_stop'].publish(Bool(data=True))
                set_fallen_pose()
                stage.update(name='settle', deadline=time.monotonic() + 20)
                stage['settle_start'] = time.monotonic()
            elif stage['name'] == 'settle':
                # Short: representative of catching a stumble quickly, not
                # letting the robot lie estopped on the ground for seconds
                # first (which lets the legs sag far from neutral under
                # passive gravity and makes any recovery much harder).
                if time.monotonic() - stage['settle_start'] > .5:
                    stage.update(name='reset_standup', deadline=time.monotonic() + 60)
                    dashboard.reset_standup()
                    if dashboard.reset_standup_stage is None:
                        return finish('reset_standup() refused to start: '
                                     + dashboard.reset_standup_status.get())
            elif stage['name'] == 'reset_standup':
                if dashboard.reset_standup_error:
                    # Check the error string itself, not reset_standup_stage ==
                    # 'failed': the dashboard's own tick() (20 ms) drives that
                    # transition independently of this 50 ms poll and clears
                    # 'failed' back to None in the very same call that sets it,
                    # so this poll can race behind it and never observe 'failed'
                    # at all. reset_standup_error stays populated regardless,
                    # since it is only ever reset to '' at the next
                    # reset_standup() call.
                    return finish('reset_standup FAILED: ' + dashboard.reset_standup_error)
                if dashboard.reset_standup_stage == 'done':
                    result['cycles'].append({
                        'cycle': stage['cycle'],
                        'status': dashboard.reset_standup_status.get(),
                        'mode': dashboard.mode_text, 'pitch': dashboard.last_pitch,
                    })
                    if stage['cycle'] == 0:
                        # Repeat once more in the SAME sim session -- this is the
                        # actual "no reopen needed" assertion this tool exists for.
                        stage.update(name='fall', cycle=1, deadline=time.monotonic() + 20)
                    else:
                        return finish('complete')
            root.after(50, pump)
        except Exception as exc:
            finish(f'{type(exc).__name__}: {exc}')

    root.after(100, pump)
    print(log, flush=True)
    try:
        root.mainloop()
    finally:
        root.destroy()
        node.destroy_node()
        rclpy.shutdown()
        result['cleanup'] = cleanup_group(process)
        stream.close()
        (log / 'result.json').write_text(json.dumps(result, indent=2))
        print(json.dumps(result, indent=2), flush=True)
        print('Evidence:', log)
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
