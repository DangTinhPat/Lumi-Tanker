# Upstream Provenance

Controller algorithms and ROS adapter were ported from:

- Repository: https://github.com/DangTinhPat/wheel_leg.git
- Revision: `701cef416b3f224925ab3b2532cf1ee5d401c703`
- Original package: `balance_firmware_core` (declares Apache-2.0).
- The MPC implementation credits Upkie's MPCBalancer and qpmpc's wheeled
  inverted-pendulum model in its original source documentation.

The local `.reference/wheel_leg` checkout is read-only reference material,
excluded from colcon discovery and version control. The TAP runtime does not
load code or assets from that checkout or `/home/dvt/wheel_leg`.

TAP robot mesh assets came from the user's `source_assets/3D.rar`; they are not
Upkie assets. Existing copyright and attribution comments in ported code are
retained. Simulation actuator parameters and uniform-density inertials are
estimates, not identified motor/hardware data.
