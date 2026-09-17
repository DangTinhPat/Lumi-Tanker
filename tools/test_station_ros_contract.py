#!/usr/bin/env python3
"""Contract check: station keeping cannot be enabled without localization.

The test intentionally starts the node with only the deployable sensor
contract (MPU6500 + six encoders). A world-pose station keeper request must
be rejected at configuration time; no pose publisher is created here.
"""
import hashlib
import os
import subprocess
import sys
import tempfile

from test_ros_contract import discover_executable


def main():
    try:
        executable = discover_executable()
    except RuntimeError as exc:
        print(f"UNAVAILABLE: {exc}")
        return 2
    args = [str(executable), "--ros-args", "-p", "use_sim_time:=false",
            "-p", "pitch_source:=imu", "-p", "station_keeper.enabled:=true"]
    with tempfile.TemporaryFile(mode="w+") as log:
        process = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT,
                                   env={**os.environ, "ROS_LOCALHOST_ONLY": "1"})
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.terminate()
            process.wait(timeout=3)
            print("FAIL: station keeper was accepted without localization")
            return 1
        log.seek(0)
        output = log.read()
    expected = "station_keeper.enabled is disabled"
    if process.returncode == 0 or expected not in output:
        print("FAIL: station keeper rejection contract missing")
        print(output[-4000:])
        return 1
    print("PASS: station keeper rejects simulator/world-pose dependency")
    print(f"Executable SHA256={hashlib.sha256(executable.read_bytes()).hexdigest()}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
