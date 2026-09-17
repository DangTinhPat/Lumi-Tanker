#!/usr/bin/env python3
"""Contract check: legacy jump mode cannot bypass the sensor contract.

Jump's old sequencer required simulator/contact/world-state inputs. The
production node must reject that mode; landing remains the IMU/encoder path.
No pose, contact, or simulator-state message is constructed by this test.
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
            "-p", "pitch_source:=imu", "-p", "jump.enabled:=true"]
    with tempfile.TemporaryFile(mode="w+") as log:
        process = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT,
                                   env={**os.environ, "ROS_LOCALHOST_ONLY": "1"})
        process.wait(timeout=5)
        log.seek(0)
        output = log.read()
    expected = "jump.enabled is disabled"
    if process.returncode == 0 or expected not in output:
        print("FAIL: jump mode was accepted without deployable contact sensing")
        print(output[-4000:])
        return 1
    print("PASS: jump mode rejects simulator/contact dependency")
    print(f"Executable SHA256={hashlib.sha256(executable.read_bytes()).hexdigest()}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
