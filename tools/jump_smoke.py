#!/usr/bin/env python3
"""Report the retired jump path; landing is the supported dynamic test."""

try:
    import rclpy
except ImportError:  # The report itself is also usable outside ROS.
    rclpy = None


def _ros_contract_init():
    if rclpy is not None:
        rclpy.init(args=[])


def sensor_only_contract():
    """Return the only legal runtime configuration for this legacy entrypoint."""
    return {"jump_enabled": False, "pitch_source": "imu"}


def main():
    contract = sensor_only_contract()
    print("Jump smoke is retired: use the IMU/encoder landing smoke instead.")
    print(f"contract={contract}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
