"""Render-only MuJoCo viewer for the native runner.

The C++ runner owns the actual physics and sensor/control loop. This process
loads the same MJCF only to render the latest debug VIEW snapshot. It never
publishes data back to the runner.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import mujoco
import mujoco.viewer


def fields(line: str):
    return {part.split("=", 1)[0]: part.split("=", 1)[1]
            for part in line.split("|")[1:] if "=" in part}


def apply_view(model, data, values):
    qpos = data.qpos
    qpos[0] = float(values.get("x", qpos[0]))
    qpos[1] = float(values.get("y", qpos[1]))
    qpos[2] = float(values.get("z", qpos[2]))
    qpos[3] = float(values.get("qw", qpos[3]))
    qpos[4] = float(values.get("qx", qpos[4]))
    qpos[5] = float(values.get("qy", qpos[5]))
    qpos[6] = float(values.get("qz", qpos[6]))
    qpos[7] = float(values.get("lh", qpos[7]))
    qpos[8] = float(values.get("lk", qpos[8]))
    qpos[9] = float(values.get("lw", qpos[9]))
    qpos[10] = float(values.get("rh", qpos[10]))
    qpos[11] = float(values.get("rk", qpos[11]))
    qpos[12] = float(values.get("rw", qpos[12]))
    mujoco.mj_forward(model, data)


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=Path, required=True)
    args = parser.parse_args(argv)

    model = mujoco.MjModel.from_xml_path(str(args.model))
    data = mujoco.MjData(model)
    # The checked-in MJCF keyframe is the single source of truth for the
    # current CAD-neutral pose; do not duplicate old robot constants here.
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)

    handle = mujoco.viewer.launch_passive(
        model, data, show_left_ui=True, show_right_ui=True)
    with handle.lock():
        handle.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        handle.cam.lookat[:] = [0.0, 0.0, 0.10]
        handle.cam.distance = 0.75
        handle.cam.azimuth = model.vis.global_.azimuth
        handle.cam.elevation = model.vis.global_.elevation
    handle.sync()
    try:
        for line in sys.stdin:
            if not handle.is_running():
                break
            if not line.startswith("VIEW|"):
                continue
            try:
                with handle.lock():
                    apply_view(model, data, fields(line))
                handle.sync(state_only=True)
            except (TypeError, ValueError, RuntimeError):
                # A malformed debug snapshot must not affect the controller;
                # simply keep rendering the last valid state.
                continue
    finally:
        handle.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
