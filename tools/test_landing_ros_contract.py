#!/usr/bin/env python3
"""Black-box landing contract using only synthetic MPU6500 + encoder data.

This is interface evidence, not a physics trial. It verifies that a
free-fall/impact sequence changes the controller support hypothesis without
publishing any model pose or contact message.
"""
import hashlib
import os
import signal
import subprocess
import sys
import tempfile
import time
import uuid

from test_ros_contract import NAMES, discover_executable


def main():
    try:
        executable = discover_executable()
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Imu, JointState
        from std_msgs.msg import Bool, String
    except (ImportError, RuntimeError) as exc:
        print(f"UNAVAILABLE: {exc}")
        return 2

    os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")
    rclpy.init()
    namespace = "/tap_landing_sensor_contract_" + uuid.uuid4().hex[:8]
    node = Node("driver", namespace=namespace)
    imu_pub = node.create_publisher(Imu, "imu", 10)
    joints_pub = node.create_publisher(JointState, "joint_states", 10)
    enable_pub = node.create_publisher(Bool, "enable", 10)
    status = {}

    def on_status(msg):
        fields = msg.data.split("|")
        status.clear()
        status.update(mode=fields[0], **dict(f.split("=", 1) for f in fields[1:] if "=" in f))

    node.create_subscription(String, "control_mode", on_status, 10)
    args = [str(executable), "--ros-args", "-r", f"__ns:={namespace}",
            "-p", "use_sim_time:=false", "-p", "auto_enable:=false",
            "-p", "pitch_source:=imu", "-p", "station_keeper.enabled:=false",
            "-p", "jump.enabled:=false", "-p", "whole_body.enabled:=false",
            "-p", "sensor_timeout_s:=0.15"]
    process = None

    def publish(accel, gyro_y=0.0):
        stamp = node.get_clock().now().to_msg()
        imu = Imu()
        imu.header.stamp = stamp
        imu.linear_acceleration.x = float(accel[0])
        imu.linear_acceleration.y = float(accel[1])
        imu.linear_acceleration.z = float(accel[2])
        imu.angular_velocity.y = float(gyro_y)
        imu.angular_velocity.z = 0.0
        imu_pub.publish(imu)
        joints = JointState()
        joints.header.stamp = stamp
        joints.name = NAMES
        joints.position = [0.115483096, 0.0, 0.0, 0.115483096, 0.0, 0.0]
        joints.velocity = [0.0] * 6
        joints_pub.publish(joints)

    def pump(duration, accel=(0.0, 0.0, 9.81), gyro_y=0.0, enable=False):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError(f"controller exited {process.returncode}")
            publish(accel, gyro_y)
            if enable:
                enable_pub.publish(Bool(data=True))
            rclpy.spin_once(node, timeout_sec=.002)

    try:
        with tempfile.TemporaryFile(mode="w+") as log:
            process = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT,
                                       env=os.environ.copy(), start_new_session=True)
            pump(.8)
            if not status:
                raise RuntimeError("no controller status")
            pump(.5, enable=True)
            baseline = status.copy()
            if baseline.get("mode") not in ("Balance", "Drive"):
                raise RuntimeError(f"controller did not enable: {baseline}")

            # Near-zero specific force is the only liftoff evidence supplied.
            pump(.08, accel=(0.0, 0.0, 0.0), gyro_y=1.0)
            flight = status.copy()
            if int(float(flight.get("flight_airborne", 0))) != 1:
                raise RuntimeError(f"IMU free-fall did not arm flight: {flight}")

            # Sustained impact force plus non-ballistic force is the touchdown
            # candidate; no wheel-contact topic is involved.
            pump(.08, accel=(0.0, 0.0, 30.0), gyro_y=0.5)
            landed = status.copy()
            if int(float(landed.get("contact", 0))) != 1:
                raise RuntimeError(f"IMU impact did not restore support: {landed}")
        print("PASS: landing phase/support uses only MPU6500 and encoders")
        print(f"Executable SHA256={hashlib.sha256(executable.read_bytes()).hexdigest()}")
        return 0
    except Exception as exc:
        print(f"FAIL: {exc}")
        return 1
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
