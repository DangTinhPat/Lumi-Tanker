#!/usr/bin/env python3
"""Black-box TAP ROS contracts. Run in a sourced ROS environment after building.

Uses an unused local DDS domain plus a unique namespace, never the GUI domain.
No Gazebo or hardware is launched. Exit 2 means prerequisites are unavailable.
"""

import fcntl
import hashlib
import math
import os
from pathlib import Path
import random
import signal
import subprocess
import sys
import tempfile
import time
import uuid


ROOT = Path(__file__).resolve().parents[1]
NAMES = [f"cad_{side}_{joint}" for side in ("left", "right")
         for joint in ("hip", "knee", "wheel")]


def discover_executable():
    candidates = [ROOT / "install/tap_control/lib/tap_control/balance_controller_node",
                  ROOT / "install/lib/tap_control/balance_controller_node"]
    for path in candidates:
        if path.is_file() and os.access(path, os.X_OK):
            return path.absolute()
    raise RuntimeError("TAP installed balance_controller_node missing; build TAP first")


def main():
    try:
        executable = discover_executable()
        executable_hash = hashlib.sha256(executable.read_bytes()).hexdigest()
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from geometry_msgs.msg import TransformStamped, Twist
        from sensor_msgs.msg import Imu, JointState
        from std_msgs.msg import Bool, Float64MultiArray, String
        from tf2_msgs.msg import TFMessage
    except (ImportError, RuntimeError) as exc:
        print(f"UNAVAILABLE: {exc}", file=sys.stderr)
        return 2

    old_domain = int(os.environ.get("ROS_DOMAIN_ID", "0"))
    domains = list(set(range(120, 200)) - {old_domain})
    random.SystemRandom().shuffle(domains)
    lock = None
    probe = None
    # Locks exclude sibling test runners; discovery rejects occupied domains.
    for domain in domains:
        candidate = open(Path(tempfile.gettempdir()) / f"tap-qa-domain-{domain}.lock", "a")
        try:
            fcntl.flock(candidate, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            candidate.close()
            continue
        os.environ["ROS_DOMAIN_ID"] = str(domain)
        os.environ["ROS_LOCALHOST_ONLY"] = "1"
        rclpy.init()
        probe = Node("tap_contract_probe_" + uuid.uuid4().hex[:8])
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            rclpy.spin_once(probe, timeout_sec=0.05)
        others = [name for name, _ in probe.get_node_names_and_namespaces()
                  if name != probe.get_name()]
        probe.destroy_node()
        rclpy.shutdown()
        if not others:
            lock = candidate
            break
        candidate.close()
    if lock is None:
        print("UNAVAILABLE: no isolated DDS domain", file=sys.stderr)
        return 2

    namespace = "/tap_contract_" + uuid.uuid4().hex[:10]
    process = None
    node = None
    failures = []
    with tempfile.TemporaryFile(mode="w+") as log:
        try:
            rclpy.init()
            node = Node("driver", namespace=namespace)
            pubs = {}
            for topic, kind in (("imu", Imu), ("model_pose", TFMessage),
                                ("joint_states", JointState), ("cmd_vel", Twist),
                                ("enable", Bool), ("emergency_stop", Bool),
                                ("stand_up", Bool), ("reset_controller", Bool),
                                ("target_pose", Float64MultiArray)):
                pubs[topic] = node.create_publisher(kind, topic, 10)
            latest = {}

            def remember(key, msg):
                latest[key] = msg
                if key in ("wheel", "leg", "motors"):
                    values = msg.data if key != "motors" else list(msg.position) + list(msg.velocity) + list(msg.effort)
                    if not all(math.isfinite(v) for v in values):
                        raise AssertionError(f"nonfinite {key} output")

            subscriptions = []
            for topic, kind, key in (("control_mode", String, "status"),
                                     ("motor_commands", JointState, "motors"),
                                     ("wheel_effort_controller/commands", Float64MultiArray, "wheel"),
                                     ("leg_effort_controller/commands", Float64MultiArray, "leg")):
                subscriptions.append(node.create_subscription(
                    kind, topic, lambda msg, key=key: remember(key, msg), 10))

            args = [str(executable), "--ros-args", "-r", f"__ns:={namespace}"]
            for parameter in ("use_sim_time:=false", "auto_enable:=false",
                              "pitch_source:=sim_pose", "whole_body.enabled:=false",
                              f"sim_pose_topic:={namespace}/model_pose",
                              "sensor_timeout_s:=0.15", "sensor_stale_grace_cycles:=2",
                              "cmd_vel_timeout_s:=0.25", "pose_slew_rate_rad_s:=10.0",
                              "pose_hip_min_rad:=-0.6", "pose_hip_max_rad:=0.6",
                              "pose_knee_min_rad:=-0.8", "pose_knee_max_rad:=0.2"):
                args += ["-p", parameter]
            process = subprocess.Popen(args, env=os.environ.copy(), stdout=log,
                                       stderr=subprocess.STDOUT, start_new_session=True)
            print(f"Executable: {executable}\nROS_DOMAIN_ID={domain} namespace={namespace}")
            print(f"Executable SHA256: {executable_hash}")
            position = [0.0] * 6
            velocity = [0.0] * 6
            pitch = 0.0
            pose_packet = "named"
            missing = False
            invalid = False
            sensors = True

            def status():
                fields = latest["status"].data.split("|")
                return dict(mode=fields[0], **dict(f.split("=", 1) for f in fields[1:] if "=" in f))

            def pump(duration, action=None):
                deadline = time.monotonic() + duration
                while time.monotonic() < deadline:
                    if process.poll() is not None:
                        raise AssertionError(f"controller exited {process.returncode}")
                    if sensors:
                        stamp = node.get_clock().now().to_msg()
                        imu = Imu()
                        imu.header.stamp = stamp
                        imu.orientation.w = 1.0
                        imu.linear_acceleration.z = 9.81
                        imu.angular_velocity.y = 0.02
                        pubs["imu"].publish(imu)
                        pose = TransformStamped()
                        pose.header.stamp = stamp
                        pose.child_frame_id = "tap_robot" if pose_packet == "named" else ""
                        pose.transform.translation.z = 0.25
                        pose.transform.rotation.y = math.sin(pitch / 2)
                        pose.transform.rotation.w = math.cos(pitch / 2)
                        transforms = [pose]
                        if pose_packet == "unnamed_multi":
                            transforms.append(TransformStamped())
                        pubs["model_pose"].publish(TFMessage(transforms=transforms))
                        joints = JointState()
                        joints.header.stamp = stamp
                        order = [5, 0, 3, 2, 1, 4]
                        if missing:
                            order.remove(5)
                        joints.name = [NAMES[i] for i in order]
                        joints.position = [position[i] for i in order]
                        joints.velocity = [velocity[i] for i in order]
                        if invalid:
                            joints.position[0] = float("nan")
                        pubs["joint_states"].publish(joints)
                    if action:
                        action()
                    rclpy.spin_once(node, timeout_sec=0.01)

            def check(condition, message):
                if not condition:
                    raise AssertionError(message)

            def stopped():
                return all(abs(v) < 1e-6 for key in ("wheel", "leg") for v in latest[key].data)

            def recover():
                pump(0.2)
                pump(0.4, lambda: pubs["enable"].publish(Bool(data=True)))
                check(status()["mode"] in ("Balance", "Drive"), "explicit enable failed")

            def run(name, test):
                try:
                    test()
                    print(f"PASS {name}")
                except (AssertionError, KeyError) as exc:
                    failures.append(f"{name}: {exc}")
                    print(f"FAIL {name}: {exc}", flush=True)

            pump(1.2)
            run("startup remains disabled", lambda: check(stopped() and status()["mode"] == "Disabled", "startup actuated"))
            pump(.2, lambda: pubs["reset_controller"].publish(Bool(data=True)))
            run("reset preserves disabled", lambda: check(
                stopped() and status()["enable"] == "0" and status()["mode"] == "Disabled",
                f"upright disabled reset did not remain Disabled: {status()['mode']}"))
            position[2], position[5] = 2.0, 4.0
            velocity[2], velocity[5] = 1.0, 3.0
            pitch = 0.1
            pump(0.25)
            run("shuffled native odometry and pitch", lambda: check(
                abs(float(status()["wheel_pos"]) - .09) < .002 and
                abs(float(status()["wheel_vel"]) - .06) < .002 and
                abs(float(status()["pitch"]) - .1) < .002 and
                abs(float(status()["pitch_rate"]) - .02) < .002,
                f"bad native observation: {status()}"))
            pose_packet = "unnamed_multi"
            pitch = .3
            pump(.4)
            run("unnamed multi-pose rejected", lambda: check(
                abs(float(status()["pitch"]) - .1) < .002 and
                float(status()["ori_age_ms"]) > 150 and stopped(),
                "ambiguous pose updated orientation or kept watchdog fresh"))
            pose_packet = "unnamed_single"
            pitch = .12
            pump(.2)
            run("unnamed single-pose accepted", lambda: check(
                abs(float(status()["pitch"]) - .12) < .002 and status()["fresh"] == "1",
                "single model pose not accepted"))
            pose_packet = "named"
            position[:] = [0.0] * 6
            velocity[:] = [0.0] * 6
            pitch = 0.0
            run("enable", recover)
            twist = Twist()
            twist.angular.z = .2
            pump(.2, lambda: pubs["cmd_vel"].publish(twist))
            run("positive yaw native mixing", lambda: check(
                abs((latest["wheel"].data[1] - latest["wheel"].data[0]) - 2 * .1545 / .03 * .2) < .03,
                f"bad yaw wheel commands {list(latest['wheel'].data)}"))
            twist.linear.x = .12
            pump(.15, lambda: pubs["cmd_vel"].publish(twist))
            run("drive received", lambda: check(float(status()["cmd_v"]) > .1, "drive command not seen"))
            pump(.5)
            run("command timeout", lambda: check(float(status()["cmd_v"]) == 0 and float(status()["cmd_w"]) == 0,
                                                  "expired drive target retained"))
            for target, hip, knee in (([9.0] * 4, .6, .2), ([-9.0] * 4, -.6, -.8)):
                pump(.65, lambda: pubs["target_pose"].publish(Float64MultiArray(data=target)))
                def clamp_test():
                    msg = latest["motors"]
                    by_name = dict(zip(msg.name, msg.position))
                    check(all(abs(by_name[f"cad_{side}_{joint}"] - expected) < .02
                              for side in ("left", "right") for joint, expected in (("hip", hip), ("knee", knee))),
                          f"native clamp mismatch {by_name}")
                run(f"bilateral clamp {hip}/{knee}", clamp_test)
            for fault in ("missing", "nonfinite", "stale"):
                run(f"enable before {fault}", recover)
                missing = fault == "missing"
                invalid = fault == "nonfinite"
                sensors = fault != "stale"
                pump(.5)
                run(f"{fault} watchdog stop", lambda: check(stopped() and status()["mode"] == "EmergencyStop", "watchdog did not stop"))
                missing = invalid = False
                sensors = True
                pump(.25)
                run(f"{fault} stop stays latched", lambda: check(stopped() and status()["mode"] == "EmergencyStop", "automatic re-enable"))
            pump(.1, lambda: pubs["emergency_stop"].publish(Bool(data=True)))
            pump(.2, lambda: pubs["reset_controller"].publish(Bool(data=True)))
            run("reset cannot override e-stop", lambda: check(
                stopped() and status()["mode"] == "EmergencyStop", "reset bypassed e-stop"))
            pump(.15, lambda: pubs["stand_up"].publish(Bool(data=True)))
            run("stand-up cannot override e-stop", lambda: check(
                stopped() and status()["mode"] == "EmergencyStop", "stand-up bypassed e-stop"))
            pump(.1, lambda: pubs["emergency_stop"].publish(Bool(data=False)))
            pump(.15)
            run("clearing e-stop does not enable", lambda: check(stopped(), "e-stop clear enabled motors"))
            pump(.2, lambda: pubs["reset_controller"].publish(Bool(data=True)))
            run("reset stays motor-off in Disabled", lambda: check(
                stopped() and status()["enable"] == "0" and status()["mode"] == "Disabled",
                f"upright reset should clear stopped mode to Disabled: {status()['mode']}"))
            pitch = 1.2
            pump(.15, lambda: pubs["emergency_stop"].publish(Bool(data=True)))
            pump(.15, lambda: pubs["reset_controller"].publish(Bool(data=True)))
            pump(.15, lambda: pubs["stand_up"].publish(Bool(data=True)))
            run("fallen reset and stand-up blocked by physical e-stop", lambda: check(
                stopped() and status()["mode"] == "EmergencyStop", "fallen e-stop bypassed"))
            pump(.1, lambda: pubs["emergency_stop"].publish(Bool(data=False)))
            run("fallen physical e-stop clear stays stopped", lambda: check(
                stopped() and status()["mode"] == "EmergencyStop", "physical stop clear resumed recovery"))
            pump(.15, lambda: pubs["reset_controller"].publish(Bool(data=True)))
            pump(.1)
            run("disabled reset at fallen pitch enters Fallen", lambda: check(
                stopped() and status()["enable"] == "0" and status()["mode"] == "Fallen",
                f"fallen reset should enter Fallen motor-off: {status()['mode']}"))
            pump(.15, lambda: pubs["stand_up"].publish(Bool(data=True)))
            run("explicit stand-up allowed after fallen reset", lambda: check(
                status()["mode"] == "StandingUp", f"stand-up blocked: {status()['mode']}"))
            pump(.15, lambda: pubs["emergency_stop"].publish(Bool(data=True)))
            run("physical e-stop interrupts stand-up", lambda: check(
                stopped() and status()["mode"] == "EmergencyStop", "recovery ignored e-stop"))
        except Exception as exc:
            failures.append(f"harness: {exc}")
        finally:
            if process is not None and process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            lock.close()
        if hashlib.sha256(executable.read_bytes()).hexdigest() != executable_hash:
            failures.append("installed executable changed during test; rerun against a stable build")
        if failures:
            log.seek(0)
            print(log.read()[-6000:], file=sys.stderr)
            print("\n".join(failures), file=sys.stderr)
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
