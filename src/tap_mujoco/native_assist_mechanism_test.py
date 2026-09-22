#!/usr/bin/env python3
"""Regression for the CAD knee-assist topology and changing tendon length."""

from __future__ import annotations

import argparse
from pathlib import Path

import mujoco
import numpy as np


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=Path, required=True)
    args = parser.parse_args()
    model = mujoco.MjModel.from_xml_path(str(args.model))
    data = mujoco.MjData(model)

    expected = {
        "left_assist_upper_site": "cad_left_thigh",
        "left_assist_lower_site": "cad_left_shin",
        "right_assist_upper_site": "cad_right_thigh",
        "right_assist_lower_site": "cad_right_shin",
    }
    for site_name, body_name in expected.items():
        site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        assert site_id >= 0 and model.site_bodyid[site_id] == body_id

    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    neutral = data.ten_length.copy()
    assert np.allclose(neutral, model.tendon_lengthspring[:, 0], atol=2.0e-4)

    knee_ids = [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "cad_left_knee"),
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "cad_right_knee"),
    ]
    samples = []
    for angle in (-0.8, 0.0, 0.8):
        for joint_id in knee_ids:
            data.qpos[model.jnt_qposadr[joint_id]] = angle
        mujoco.mj_forward(model, data)
        samples.append(data.ten_length.copy())
    samples = np.asarray(samples)
    assert np.ptp(samples[:, 0]) > 1.0e-2
    assert np.ptp(samples[:, 1]) > 1.0e-2
    print("PASS CAD knee assist: thigh/shin endpoints connected; length varies")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
