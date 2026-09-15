#!/usr/bin/env python3
"""Station-only ROS black-box contracts; synthetic sensors, no physics/hardware.

Uses an isolated localhost DDS domain and namespace. The transient-stale test
requires preserved anchors with zero correction, including before watchdog latch.
Run after building/sourcing TAP. Exit 2 indicates missing prerequisites.
"""
import fcntl
import hashlib
import math
import os
from pathlib import Path
import random
import signal
import subprocess
import tempfile
import time
import uuid

from test_ros_contract import discover_executable, NAMES


def main():
    try:
        executable = discover_executable()
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import TransformStamped, Twist
        from sensor_msgs.msg import Imu, JointState
        from std_msgs.msg import Bool, String, Float64MultiArray
        from tf2_msgs.msg import TFMessage
    except (ImportError, RuntimeError) as exc:
        print(f"UNAVAILABLE: {exc}")
        return 2

    domains = list(set(range(120, 200)) - {int(os.environ.get("ROS_DOMAIN_ID", "0"))})
    random.SystemRandom().shuffle(domains)
    lock = None
    for domain in domains:
        candidate = open(Path(tempfile.gettempdir())/f"tap-qa-domain-{domain}.lock", "a")
        try:
            fcntl.flock(candidate, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            candidate.close()
            continue
        os.environ.update(ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY="1")
        rclpy.init()
        probe = Node("station_probe_"+uuid.uuid4().hex[:8])
        until = time.monotonic()+1
        while time.monotonic() < until:
            rclpy.spin_once(probe, timeout_sec=.05)
        occupied = any(n != probe.get_name() for n, _ in probe.get_node_names_and_namespaces())
        probe.destroy_node()
        rclpy.shutdown()
        if not occupied:
            lock = candidate
            break
        candidate.close()
    if lock is None:
        print("UNAVAILABLE: no isolated DDS domain")
        return 2

    namespace = "/tap_station_contract_"+uuid.uuid4().hex[:10]
    process = node = None
    failures = []
    with tempfile.TemporaryFile(mode="w+") as log:
        try:
            rclpy.init()
            node = Node("driver", namespace=namespace)
            pubs = {topic: node.create_publisher(kind, topic, 10) for topic, kind in (
                ("imu", Imu), ("model_pose", TFMessage), ("joint_states", JointState),
                ("enable", Bool), ("emergency_stop", Bool), ("reset_controller", Bool), ("cmd_vel", Twist))}
            latest = {}
            def remember(msg):
                fields = msg.data.split("|")
                latest.clear()
                latest.update(mode=fields[0], **dict(f.split("=", 1) for f in fields[1:] if "=" in f))
            subscription = node.create_subscription(String, "control_mode", remember, 10)
            outputs = {}
            def remember_output(key, msg):
                values = list(msg.data) if key != "motors" else list(msg.position)+list(msg.velocity)+list(msg.effort)
                if not all(math.isfinite(v) for v in values):
                    raise RuntimeError(f"nonfinite {key} output")
                outputs[key] = (msg, time.monotonic())
            output_subscriptions = [node.create_subscription(kind, topic,
                lambda msg, key=key: remember_output(key, msg), 10)
                for topic, kind, key in (
                    ("wheel_effort_controller/commands", Float64MultiArray, "wheel"),
                    ("leg_effort_controller/commands", Float64MultiArray, "leg"),
                    ("motor_commands", JointState, "motors"))]
            args = [str(executable), "--ros-args", "-r", f"__ns:={namespace}"]
            for param in ["use_sim_time:=false", "auto_enable:=false", "pitch_source:=sim_pose",
                          "whole_body.enabled:=false", f"sim_pose_topic:={namespace}/model_pose",
                          "station_keeper.enabled:=true", "station_keeper.position_gain_hz:=0.8",
                          "station_keeper.yaw_gain_hz:=2.0", "station_keeper.max_forward_m_s:=0.08",
                          "station_keeper.max_yaw_rad_s:=0.25", "sensor_timeout_s:=0.10",
                          "sensor_stale_grace_cycles:=100", "cmd_vel_timeout_s:=0.20",
                          "mpc.nb_timesteps:=20"]:
                args += ["-p", param]
            process = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT,
                                       env=os.environ.copy(), start_new_session=True)
            print(f"Executable SHA256={hashlib.sha256(executable.read_bytes()).hexdigest()} domain={domain}")
            x = y = yaw = 0.
            pitch = pitch_rate = hip_rate = 0.
            pose_fresh = True
            driving = None
            def pump(duration, action=None):
                until = time.monotonic()+duration
                while time.monotonic() < until:
                    if process.poll() is not None:
                        raise RuntimeError(f"controller exited {process.returncode}")
                    stamp = node.get_clock().now().to_msg()
                    imu = Imu(); imu.header.stamp = stamp; imu.orientation.w = 1.; imu.linear_acceleration.z = 9.81
                    imu.angular_velocity.y = pitch_rate
                    pubs["imu"].publish(imu)
                    joints = JointState(); joints.header.stamp = stamp; joints.name = NAMES
                    joints.position = [.115483096, 0., 0., .115483096, 0., 0.]; joints.velocity = [0.]*6
                    joints.velocity[0] = joints.velocity[3] = hip_rate
                    pubs["joint_states"].publish(joints)
                    if pose_fresh:
                        pose = TransformStamped(); pose.header.stamp = stamp; pose.child_frame_id = "tap_robot"
                        pose.transform.translation.x = x; pose.transform.translation.y = y; pose.transform.translation.z = .25
                        pose.transform.rotation.w = math.cos(yaw/2)*math.cos(pitch/2)
                        pose.transform.rotation.x = -math.sin(yaw/2)*math.sin(pitch/2)
                        pose.transform.rotation.y = math.cos(yaw/2)*math.sin(pitch/2)
                        pose.transform.rotation.z = math.sin(yaw/2)*math.cos(pitch/2)
                        pubs["model_pose"].publish(TFMessage(transforms=[pose]))
                    if driving is not None:
                        twist = Twist(); twist.linear.x, twist.angular.z = driving
                        pubs["cmd_vel"].publish(twist)
                    if action: action()
                    rclpy.spin_once(node, timeout_sec=.005)
            def check(name, condition):
                print(("PASS " if condition else "FAIL ")+name)
                if not condition: failures.append((name, dict(latest)))
            def value(key): return float(latest[key])
            def zero(): return value("station_v") == 0 and value("station_w") == 0
            def motors_stopped():
                now = time.monotonic()
                if any(key not in outputs or now-outputs[key][1] > .1 for key in ("wheel", "leg", "motors")):
                    return False
                wheel, leg, motors = (outputs[key][0] for key in ("wheel", "leg", "motors"))
                return (len(wheel.data) == 2 and len(leg.data) == 4 and len(motors.velocity) == 6 and
                        len(motors.effort) == 6 and all(v == 0 for v in
                        list(wheel.data)+list(leg.data)+list(motors.velocity)+list(motors.effort)))
            def enable():
                pump(.15, lambda: pubs["enable"].publish(Bool(data=True)))
                pump(.10)
            pump(1.)
            enable()
            check("initial capture", latest["station"] == "1" and zero())
            x = .05; pump(.12)
            check("world displacement correction", abs(value("anchor_ex")+.05)<.002 and abs(value("station_v")+.04)<.002)
            pump(.15)
            check("correction never recaptures itself", abs(value("anchor_ex")+.05)<.002)
            yaw = .2; pump(.12)
            check("yaw correction bounded", abs(value("station_w")+.25)<.002)
            driving = (.06, 0.); x = .2; yaw = .3; pump(.12)
            check("operator drive disables correction", latest["station"] == "0" and zero())
            x = .3; pump(.12)
            driving = (0., 0.); pump(.1)
            x = .32; pump(.1)
            check("drive release retains last driving anchor", abs(value("anchor_ex")+.02)<.002)
            driving = (.06, 0.); x = .5; pump(.12)
            driving = None; pump(.3)
            x = .52; pump(.1)
            check("cmd_vel timeout releases into hold", abs(value("cmd_v"))<1e-6 and abs(value("anchor_ex")+.02)<.002)

            # 0.25 s silence exceeds pose timeout but is inside 0.5 s grace.
            pose_fresh = False; pump(.25)
            check("transient stale zeros correction without stop", latest["fresh"] == "0" and zero() and latest["mode"] not in ("EmergencyStop", "Fallen"))
            pose_fresh = True; x = .54; pump(.1)
            check("transient stale preserves old anchor", abs(value("anchor_ex")+.04)<.002)

            # Start AND release a drive request while pose is already stale.
            # The first fresh pose must replace the pre-drive anchor, even
            # though operator_driving is false again by that time.
            pose_fresh = False; pump(.16)
            check("drive-gap setup is inside stale grace", latest["fresh"] == "0" and latest["mode"] not in ("EmergencyStop", "Fallen"))
            driving = (.06, .1); pump(.06)
            check("drive during stale has no station correction", latest["fresh"] == "0" and value("cmd_v") > .05 and zero())
            driving = (0., 0.); pump(.06)
            check("release while stale remains correction-free", latest["fresh"] == "0" and abs(value("cmd_v")) < 1e-6 and zero())
            pose_fresh = True; x = .7; y = .1; yaw = .45; pump(.1)
            check("first fresh pose recaptures after stale drive-release", latest["station"] == "1" and
                  abs(value("anchor_ex")) < .002 and abs(value("anchor_ey")) < .002 and
                  abs(value("anchor_yaw")) < .002 and zero())
            x = .72; yaw = .47; pump(.1)
            check("post-gap correction uses new anchor", abs(value("anchor_ex")+.02) < .002 and
                  abs(value("anchor_yaw")+.02) < .002 and
                  abs(value("station_v")+.8*.02*math.cos(yaw)) < .002 and
                  abs(value("station_w")+.04) < .002)
            pose_fresh = False; pump(.2)
            pose_fresh = True; x = .74; pump(.1)
            check("stale drive recapture is consumed once", abs(value("anchor_ex")+.04) < .002)

            pose_fresh = False; pump(.8)
            check("latched stale clears anchor and stops", latest["mode"] == "EmergencyStop" and latest["station"] == "0" and zero())
            pose_fresh = True; x = .8; pump(.15)
            check("fresh sensors cannot auto-enable", latest["mode"] == "EmergencyStop" and zero())
            enable()
            check("explicit enable recaptures after stale stop", latest["station"] == "1" and abs(value("anchor_ex"))<.002)
            x = .85; pump(.12)
            pump(.08, lambda: pubs["enable"].publish(Bool(data=False)))
            check("disable clears station correction", latest["station"] == "0" and zero())
            x = 1.; enable()
            check("enable captures new location", abs(value("anchor_ex"))<.002)
            x = 1.05; pump(.12)
            pump(.03, lambda: pubs["reset_controller"].publish(Bool(data=True)))
            pump(.15)
            check("controller reset discards old anchor", abs(value("anchor_ex"))<.002)

            for axis in ("x", "y"):
                valid_x, valid_y = x, y
                if axis == "x": x = float("nan")
                else: y = float("nan")
                pump(.22)
                check(f"NaN pose {axis} rejected rather than refreshing pose", latest["fresh"] == "0" and
                      value("ori_age_ms") > 100 and latest["mode"] not in ("EmergencyStop", "Fallen") and zero())
                pump(.65)
                check(f"NaN pose {axis} eventually stops with finite zero actuation", latest["mode"] == "EmergencyStop" and
                      latest["station"] == "0" and zero() and motors_stopped())
                x, y = valid_x+.03, valid_y+.02
                pump(.15)
                check(f"valid pose after NaN {axis} cannot auto-enable", latest["fresh"] == "1" and
                      latest["mode"] == "EmergencyStop" and zero() and motors_stopped())
                enable()
                check(f"explicit enable after NaN {axis} captures fresh pose", latest["station"] == "1" and
                      abs(value("anchor_ex")) < .002 and abs(value("anchor_ey")) < .002)

            pump(.08, lambda: pubs["emergency_stop"].publish(Bool(data=True)))
            check("estop clears correction", latest["mode"] == "EmergencyStop" and latest["station"] == "0" and zero())
            os.killpg(process.pid, signal.SIGINT)
            try: process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL); process.wait(timeout=5)
            drain_until = time.monotonic()+.3
            while time.monotonic() < drain_until: rclpy.spin_once(node, timeout_sec=.01)
            latest.clear(); outputs.clear()
            x = y = yaw = pitch = pitch_rate = 0.
            hip_rate = .4; driving = None; pose_fresh = True
            dwell_args = args+["-r", "__node:=station_dwell_contract",
                               "-p", "station_keeper.acquire_dwell_s:=0.5"]
            process = subprocess.Popen(dwell_args, stdout=log, stderr=subprocess.STDOUT,
                                       env=os.environ.copy(), start_new_session=True)
            deadline = time.monotonic()+10
            while not latest and time.monotonic() < deadline: pump(.1)
            enable(); pump(.6)
            check("acquisition dwell waits while legs move", latest["station"] == "0" and zero())
            hip_rate = 0.; pump(.2)
            check("partial stable dwell cannot acquire", latest["station"] == "0" and zero())
            hip_rate = .4; pump(.1)
            hip_rate = 0.; pump(.25)
            check("leg instability restarts acquisition dwell", latest["station"] == "0" and zero())
            pump(.35)
            check("continuous stable dwell captures anchor", latest["station"] == "1" and zero())
            x = .03; pitch = .08; pump(.1)
            check("acquired anchor survives failed acquisition predicate", latest["station"] == "1" and
                  abs(value("anchor_ex")+.03)<.002 and value("station_v")<0)
            pump(.03, lambda: pubs["reset_controller"].publish(Bool(data=True)))
            pump(.6)
            check("reset requires new dwell and pitch readiness", latest["station"] == "0" and zero())
            pitch = 0.; pitch_rate = .4; pump(.6)
            check("pitch rate prevents initial acquisition", latest["station"] == "0" and zero())
            pitch_rate = 0.; pump(.65)
            check("quiet pitch rate permits acquisition after dwell", latest["station"] == "1" and zero())
            hip_rate = .4
            pump(.03, lambda: pubs["reset_controller"].publish(Bool(data=True)))
            driving = (.06, 0.); pump(.2)
            x = .08; pump(.1); driving = (0., 0.); pump(.1)
            check("drive release recaptures without initial stability dwell", latest["station"] == "1" and
                  abs(value("anchor_ex"))<.002 and zero())
            print(f"{len(failures)} failed station contracts")
            for name, fields in failures: print(name, fields)
            return 1 if failures else 0
        except (RuntimeError, KeyError) as exc:
            print(f"FAIL harness: {exc}")
            return 1
        finally:
            if process is not None and process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try: process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL); process.wait(timeout=5)
            if process is not None and process.returncode not in (0, -signal.SIGINT):
                log.seek(0); print(log.read()[-3000:])
            if node is not None: node.destroy_node()
            if rclpy.ok(): rclpy.shutdown()
            lock.close()


if __name__ == "__main__":
    raise SystemExit(main())
