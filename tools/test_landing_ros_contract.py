#!/usr/bin/env python3
"""Focused synthetic landing adapter contracts; no Gazebo or actuator plugin.

Run: python3 -u tools/test_landing_ros_contract.py in the sourced TAP overlay.
Uses TAP's installed executable, a locked unused DDS domain and owned children.
Exit 2 means missing prerequisites. Encoder positions follow debug references;
velocities are synthetic zero, so this is interface evidence, not physics.
"""

from contextlib import contextmanager
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

from test_ros_contract import NAMES, discover_executable


def terminate_owned(process):
    if process is not None and process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=3)


@contextmanager
def isolated_domain(rclpy, Node):
    # Domain 125 is reserved for the standard TAP jump harness.
    domains = list(set(range(120, 200)) - {125, int(os.environ.get("ROS_DOMAIN_ID", "0"))})
    random.SystemRandom().shuffle(domains)
    for domain in domains:
        with open(Path(tempfile.gettempdir()) / f"tap-qa-domain-{domain}.lock", "a") as lock:
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                continue
            os.environ.update(ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY="1")
            rclpy.init()
            probe = Node("landing_probe_" + uuid.uuid4().hex[:8])
            try:
                until = time.monotonic() + 1
                while time.monotonic() < until:
                    rclpy.spin_once(probe, timeout_sec=.05)
                occupied = any(name != probe.get_name()
                               for name, _ in probe.get_node_names_and_namespaces())
            finally:
                probe.destroy_node()
                rclpy.shutdown()
            if not occupied:
                yield domain
                return
    raise RuntimeError("no isolated DDS domain")


def main():
    try:
        executable = discover_executable()
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import TransformStamped
        from sensor_msgs.msg import Imu, JointState
        from std_msgs.msg import Bool, Float64MultiArray, String
        from tf2_msgs.msg import TFMessage
        from ros_gz_interfaces.msg import Contact, Contacts
    except (ImportError, RuntimeError) as exc:
        print(f"UNAVAILABLE: {exc}")
        return 2

    count = 0

    def check(label, condition):
        nonlocal count
        if not condition:
            raise AssertionError(label)
        count += 1
        print("PASS " + label, flush=True)

    def arguments(namespace, enabled):
        params = {
            "use_sim_time": "false", "auto_enable": "false",
            "pitch_source": "sim_pose", "sim_pose_topic": namespace + "/model_pose",
            "whole_body.enabled": "false", "station_keeper.enabled": "false",
            "sensor_timeout_s": "0.15", "mpc.nb_timesteps": "20",
            "pose_hip_min_rad": "-0.6", "pose_hip_max_rad": "1.05", "pose_knee_min_rad": "-1.6",
            "pose_knee_max_rad": "0.2", "jump.enabled": "true",
            "jump.crouch_duration_s": "0.4", "jump.thrust_duration_s": "0.18",
            "jump.max_thrust_s": "1.2", "jump.contact_debounce_s": "0.04",
            "jump.contact_timeout_s": "0.15", "jump.wheel_frame_compensation": "false",
            "jump.landing_world_frame": str(enabled).lower(),
            "jump.thrust_hip_offset_rad": "0.10" if enabled else "0.0",
            "jump.thrust_wheel_control": str(enabled).lower(),
            "jump.thrust_wheel_speed_m_s": "-0.15",
            "jump.landing_compression": str(enabled).lower(),
            "jump.compression_knee_rad": "-1.15", "jump.compression_duration_s": "0.30",
        }
        args = [str(executable), "--ros-args", "-r", f"__ns:={namespace}"]
        for key, value in params.items():
            args += ["-p", f"{key}:={value}"]
        return args

    class Trial:
        def __init__(self, enabled):
            self.namespace = "/tap_landing_contract_" + uuid.uuid4().hex[:10]
            self.node = Node("driver", namespace=self.namespace)
            self.process = None
            self.log = tempfile.TemporaryFile(mode="w+")
            self.latest = {}
            self.status = {}
            self.pitch = 0.0
            self.contacts = True
            self.request = False
            self.position = [.115483096, 0., 0., .115483096, 0., 0.]
            self.pubs = {topic: self.node.create_publisher(kind, topic, 10)
                         for topic, kind in (("imu", Imu), ("joint_states", JointState),
                             ("model_pose", TFMessage), ("left_wheel_contacts", Contacts),
                             ("right_wheel_contacts", Contacts), ("enable", Bool),
                             ("emergency_stop", Bool), ("jump_request", Bool))}
            self.subs = [self.node.create_subscription(String, "jump_status", self.on_status, 10)]
            for topic, kind, key in (("motor_commands", JointState, "motors"),
                    ("leg_velocity_reference", JointState, "legref"),
                    ("actuator_enable", Bool, "gate"),
                    ("wheel_effort_controller/commands", Float64MultiArray, "wheel"),
                    ("leg_effort_controller/commands", Float64MultiArray, "leg")):
                self.subs.append(self.node.create_subscription(
                    kind, topic, lambda msg, key=key: self.remember(key, msg), 10))
            self.args = arguments(self.namespace, enabled)

        def on_status(self, msg):
            self.status = dict(field.split("=", 1) for field in msg.data.split("|") if "=" in field)

        def remember(self, key, msg):
            if key == "motors":
                assert msg.name == NAMES and all(len(v) == 6 for v in
                    (msg.position, msg.velocity, msg.effort)), "malformed motor debug"
                values = list(msg.position) + list(msg.velocity) + list(msg.effort)
                # Follow only named leg positions; wheel odometry stays stationary.
                for i in (0, 1, 3, 4):
                    self.position[i] = msg.position[msg.name.index(NAMES[i])]
            elif key == "legref":
                assert msg.name == [NAMES[i] for i in (0, 1, 3, 4)], "leg reference order"
                assert len(msg.velocity) == 4 and msg.header.stamp.sec > 0, "leg reference shape/stamp"
                values = msg.velocity
            else:
                values = [] if key == "gate" else msg.data
            assert all(math.isfinite(v) for v in values), f"nonfinite {key} output"
            self.latest[key] = (msg, time.monotonic())

        def pump(self, duration, action=None):
            until = time.monotonic() + duration
            while time.monotonic() < until:
                if self.process.poll() is not None:
                    raise AssertionError(f"controller exited {self.process.returncode}")
                stamp = self.node.get_clock().now().to_msg()
                imu = Imu()
                imu.header.stamp = stamp
                imu.orientation.y, imu.orientation.w = math.sin(self.pitch / 2), math.cos(self.pitch / 2)
                imu.linear_acceleration.z = 9.81
                self.pubs["imu"].publish(imu)
                joints = JointState()
                joints.header.stamp = stamp
                joints.name, joints.position, joints.velocity = NAMES, self.position, [0.] * 6
                self.pubs["joint_states"].publish(joints)
                pose = TransformStamped()
                pose.header.stamp, pose.child_frame_id = stamp, "tap_robot"
                pose.transform.translation.z = .237
                pose.transform.rotation = imu.orientation
                self.pubs["model_pose"].publish(TFMessage(transforms=[pose]))
                for side in ("left", "right"):
                    msg = Contacts()
                    msg.header.stamp = stamp
                    if self.contacts:
                        contact = Contact()
                        contact.collision1.name = f"tap_robot::cad_{side}_wheel_link::cad_{side}_wheel_link_collision"
                        contact.collision2.name = "ground_plane::link::collision"
                        msg.contacts = [contact]
                    self.pubs[side + "_wheel_contacts"].publish(msg)
                self.pubs["jump_request"].publish(Bool(data=self.request))
                if action:
                    action()
                # Drain callbacks between sensor packets without busy publishing.
                tick_end = time.monotonic() + .005
                while time.monotonic() < tick_end:
                    rclpy.spin_once(self.node, timeout_sec=.001)

        def until(self, predicate, label, timeout=3):
            deadline = time.monotonic() + timeout
            while not predicate() and time.monotonic() < deadline:
                self.pump(.01)
            assert predicate(), f"timeout {label}: {self.status}"

        def phase(self, name, elapsed=0):
            return self.status.get("phase") == name and float(self.status.get("phase_elapsed", 0)) >= elapsed

        def msg(self, key):
            msg, received = self.latest[key]
            assert time.monotonic() - received < .15, f"stale {key} output"
            return msg

        def wheels(self, value):
            actual = self.msg("wheel").data
            debug = self.msg("motors").velocity
            return len(actual) == 2 and all(abs(v - value) < 1e-4 for v in
                                          list(actual) + [debug[2], debug[5]])

        def stop(self, label):
            self.pump(.10, lambda: self.pubs["emergency_stop"].publish(Bool(data=True)))
            check(label + " faults and gates off", self.phase("Fault") and not self.msg("gate").data)
            check(label + " zeros leg effort, wheel velocity and leg velocity reference",
                  len(self.msg("leg").data) == 4 and len(self.msg("wheel").data) == 2 and
                  all(v == 0 for key in ("leg", "wheel") for v in self.msg(key).data) and
                  all(v == 0 for v in self.msg("legref").velocity))

        def __enter__(self):
            try:
                self.process = subprocess.Popen(self.args, stdout=self.log, stderr=subprocess.STDOUT,
                                                env=os.environ.copy(), start_new_session=True)
                print(f"Trial namespace={self.namespace}", flush=True)
                self.until(lambda: bool(self.status), "discovery", timeout=10)
                self.pump(.2, lambda: self.pubs["enable"].publish(Bool(data=True)))
                self.until(lambda: self.status.get("ready") == "1", "grounded readiness")
                self.request = True
                self.until(lambda: self.phase("Thrust", .25), "tracked crouch then thrust")
                self.request = False
                return self
            except BaseException:
                self.__exit__(True, None, None)
                raise

        def __exit__(self, exc_type, exc, tb):
            terminate_owned(self.process)
            if exc_type:
                self.log.seek(0)
                print(self.log.read()[-4000:])
            self.log.close()
            self.node.destroy_node()

    digest = hashlib.sha256(executable.read_bytes()).hexdigest()
    try:
        with isolated_domain(rclpy, Node) as domain:
            print(f"Executable={executable}\nSHA256={digest}\nDDS domain={domain}", flush=True)
            rclpy.init()
            try:
                samples = {}
                for enabled in (False, True):
                    label = "flags on" if enabled else "flags off"
                    with Trial(enabled) as trial:
                        thrust = list(trial.msg("motors").position)
                        if enabled:
                            check("Thrust -0.15 m/s maps to -5 rad/s on BOTH native +Y wheels",
                                  trial.wheels(-.15 / .030) and trial.msg("gate").data)
                        else:
                            check("disabled thrust wheel flag does not apply configured speed", not trial.wheels(-5.))
                        trial.contacts = False
                        trial.until(lambda: trial.phase("Flight", .10), "measured takeoff")
                        trial.pitch = .12
                        trial.until(lambda: trial.phase("Flight", .40), "settled flight target")
                        flight = list(trial.msg("motors").position)
                        check(label + " Flight clears wheel override", trial.wheels(0.) and trial.msg("gate").data)
                        check(label + " no compression on fresh empty contact heartbeats",
                              trial.phase("Flight") and trial.status.get("contact_fresh") == "1" and
                              trial.status.get("left_contact") == trial.status.get("right_contact") == "0" and
                              all(abs(flight[i] + .6) < .015 for i in (1, 4)))
                        trial.pitch = 0.
                        trial.contacts = True
                        trial.until(lambda: trial.phase("Land", .27 if enabled else .20),
                                    "contact-driven landing")
                        landing = list(trial.msg("motors").position)
                        check(label + " touchdown enters Land", trial.status.get("reason") == "measured_landing")
                        check(label + " compression target follows configured flag and depth",
                              all(abs(landing[i] - (-1.15 if enabled else -.6)) < .04 for i in (1, 4)))
                        samples[enabled] = (thrust, flight)
                        trial.stop(label + " landing stop")
                check("thrust_hip_offset_rad adds +0.10 rad to both hip targets",
                      all(abs(samples[True][0][i] - samples[False][0][i] - .10) < .015 for i in (0, 3)))
                # Positive pitch requires negative body-frame COM offset to
                # satisfy cos(pitch)*COM_x + sin(pitch)*COM_z == 0.
                hip_delta = [samples[True][1][i] - samples[False][1][i] for i in (0, 3)]
                check(f"landing_world_frame lowers both hips at +0.12 pitch: {hip_delta}",
                      all(delta < -.05 for delta in hip_delta))
                with Trial(True) as trial:
                    check("override active before thrust stop", trial.wheels(-5.))
                    trial.stop("Thrust stop")

                # Require the relevant diagnostic, not just any nonzero exit.
                invalid = {
                    "thrust_hip_offset_rad": (["0.151", ".nan"], "Invalid thrust hip offset"),
                    "thrust_wheel_speed_m_s": (["0.351", ".inf"], "Invalid thrust wheel speed"),
                    "compression_knee_rad": (["-1.61", "-0.6", ".nan"], "Invalid landing compression"),
                    "compression_duration_s": (["0.049", "0.501", ".inf"], "Invalid landing compression"),
                    "landing_world_frame": (["1"], "invalid type"),
                    "thrust_wheel_control": (["1"], "invalid type"),
                    "landing_compression": (["1"], "invalid type"),
                }
                rejections = [("jump." + key, values, diagnostic)
                              for key, (values, diagnostic) in invalid.items()]
                rejections += [(key, [value], "full provisional pose envelope")
                               for key, value in (("pose_hip_min_rad", "-0.2"),
                                                  ("pose_knee_max_rad", "0.0"))]
                for key, values, diagnostic in rejections:
                    for value in values:
                        namespace = "/tap_landing_reject_" + uuid.uuid4().hex[:10]
                        # Replace the valid value so ROS sees one typed override;
                        # duplicate bool/integer overrides can fail in YAML parsing.
                        rejection_args = [f"{key}:={value}" if arg.startswith(f"{key}:=")
                                          else arg for arg in arguments(namespace, True)]
                        with tempfile.TemporaryFile(mode="w+") as log:
                            process = subprocess.Popen(rejection_args,
                                stdout=log, stderr=subprocess.STDOUT, env=os.environ.copy(), start_new_session=True)
                            try:
                                try:
                                    process.wait(timeout=3)
                                except subprocess.TimeoutExpired:
                                    pass
                                log.seek(0)
                                output = log.read()
                                if process.poll() is None or process.returncode == 0 or diagnostic.lower() not in output.lower():
                                    print(output[-2000:], flush=True)
                                check(f"reject {key}:={value}", process.poll() is not None and
                                      process.returncode != 0 and diagnostic.lower() in output.lower())
                            finally:
                                terminate_owned(process)
                check("installed executable unchanged during contracts",
                      hashlib.sha256(executable.read_bytes()).hexdigest() == digest)
            finally:
                rclpy.shutdown()
    except (AssertionError, RuntimeError, KeyError, OSError) as exc:
        print(f"FAIL after {count} checks: {exc}", flush=True)
        return 1
    print(f"{count} landing ROS contracts passed; synthetic interfaces only", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
