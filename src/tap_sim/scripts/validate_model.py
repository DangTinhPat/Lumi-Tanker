#!/usr/bin/env python3
"""Gate 1 static contract check for the canonical TAP MuJoCo model.

This check deliberately inspects only the physical model. It does not use
MuJoCo pose/contact/constraint data as a controller input and therefore cannot
silently turn simulator truth into CMF evidence.
"""
import argparse
import math
from pathlib import Path

import mujoco


EXPECTED_JOINTS = [
    "cad_left_hip", "cad_left_knee", "cad_left_wheel",
    "cad_right_hip", "cad_right_knee", "cad_right_wheel",
]
EXPECTED_BODIES = [
    "cad_body", "cad_left_thigh", "cad_left_shin", "cad_left_wheel_link",
    "cad_right_thigh", "cad_right_shin", "cad_right_wheel_link",
]
EXPECTED_MASS = 4.621
EXPECTED_SENSOR_NAMES = [
    "mpu6500_accel", "mpu6500_gyro",
    "enc_left_hip_pos", "enc_left_knee_pos", "enc_left_wheel_pos",
    "enc_right_hip_pos", "enc_right_knee_pos", "enc_right_wheel_pos",
    "enc_left_hip_vel", "enc_left_knee_vel", "enc_left_wheel_vel",
    "enc_right_hip_vel", "enc_right_knee_vel", "enc_right_wheel_vel",
]
EXPECTED_ASSIST_TENDONS = ["left_knee_assist", "right_knee_assist"]


def close(a, b, tol=2.0e-5):
    return abs(float(a) - float(b)) <= tol


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=Path, required=True)
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(str(args.model))
    failures = []

    if model.nbody - 1 != len(EXPECTED_BODIES):
        failures.append(f"expected {len(EXPECTED_BODIES)} robot bodies, got {model.nbody - 1}")
    if model.njnt != len(EXPECTED_JOINTS) + 1:
        failures.append(f"expected 1 freejoint + 6 joints, got {model.njnt}")
    if model.nq != 7 + len(EXPECTED_JOINTS):
        failures.append(f"unexpected nq={model.nq}")
    if model.nv != 6 + len(EXPECTED_JOINTS):
        failures.append(f"unexpected nv={model.nv}")

    for name in EXPECTED_BODIES:
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name) < 0:
            failures.append(f"missing body {name}")

    for name in EXPECTED_JOINTS:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0:
            failures.append(f"missing joint {name}")
            continue
        if model.jnt_type[jid] != mujoco.mjtJoint.mjJNT_HINGE:
            failures.append(f"joint {name} is not hinge")
        axis = model.jnt_axis[jid]
        if not (close(axis[0], 0.0) and close(axis[1], 1.0) and close(axis[2], 0.0)):
            failures.append(f"joint {name} axis is {axis}, expected +Y")

    # The encoder knee zero remains an interior mechanical reference, never a
    # hidden software/model stop; the natural ENABLE pose is lower than q=0.
    for name in ("cad_left_knee", "cad_right_knee"):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid >= 0 and model.jnt_limited[jid]:
            lo, hi = model.jnt_range[jid]
            if not (lo < 0.0 < hi):
                failures.append(
                    f"{name} encoder zero is outside its mechanical range "
                    f"[{lo:.6f}, {hi:.6f}]")

    robot_mass = float(sum(model.body_mass[1:]))
    if not close(robot_mass, EXPECTED_MASS, 2.0e-3):
        failures.append(f"mass sum {robot_mass:.9f} != {EXPECTED_MASS:.9f}")

    for name in EXPECTED_SENSOR_NAMES:
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, name) < 0:
            failures.append(f"missing sensor {name}")
    if model.nu != 6:
        failures.append(f"expected 6 motor actuators, got {model.nu}")

    # The CAD knee-assist must be a real cross-linking passive element, not a
    # rigid mesh accidentally left on the thigh.  Check both endpoint sites,
    # both tendons, and the natural ENABLE tendon length.
    if model.ntendon != len(EXPECTED_ASSIST_TENDONS):
        failures.append(
            f"expected {len(EXPECTED_ASSIST_TENDONS)} assist tendons, "
            f"got {model.ntendon}")
    for name in EXPECTED_ASSIST_TENDONS:
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, name) < 0:
            failures.append(f"missing assist tendon {name}")
    for name in (
        "left_assist_upper_site", "left_assist_lower_site",
        "right_assist_upper_site", "right_assist_lower_site",
    ):
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, name) < 0:
            failures.append(f"missing assist endpoint site {name}")

    # Forward dynamics at the canonical spawn must remain finite and must not
    # start with a penetrating wheel-ground contact.
    data = mujoco.MjData(model)
    # The canonical static check uses the non-penetrating CAD-natural ENABLE keyframe;
    # raw qpos zero is the mathematical free-joint origin below the robot.
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    if not all(math.isfinite(float(v)) for v in data.qpos):
        failures.append("non-finite qpos after mj_forward")
    if any(float(data.contact[i].dist) < -1.0e-6 for i in range(data.ncon)):
        failures.append(f"canonical spawn has penetrating contact(s): {data.ncon}")
    if model.ntendon == len(EXPECTED_ASSIST_TENDONS):
        for tid, name in enumerate(EXPECTED_ASSIST_TENDONS):
            if abs(float(data.ten_length[tid]) -
                   float(model.tendon_lengthspring[tid, 0])) > 2.0e-4:
                failures.append(
                    f"{name} natural ENABLE length {data.ten_length[tid]:.6f} "
                    f"does not match spring preload "
                    f"{model.tendon_lengthspring[tid, 0]:.6f}")

    # Gearbox breakaway friction should exceed gravity's joint torque at the
    # prone spawn; the 1.5x margin is a simulation estimate, not calibration.
    prone_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_KEY, "power_off_collapse_current")
    if prone_id < 0:
        failures.append("missing power_off_collapse_current keyframe")
    else:
        mujoco.mj_resetDataKeyframe(model, data, prone_id)
        mujoco.mj_forward(model, data)
        prone_joints = []
        for name in ("cad_left_hip", "cad_left_knee",
                     "cad_right_hip", "cad_right_knee"):
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            dof = model.jnt_dofadr[jid]
            qpos = model.jnt_qposadr[jid]
            prone_joints.append((name, dof, qpos, float(data.qpos[qpos])))
            gravity_torque = abs(float(data.qfrc_bias[dof]))
            friction = float(model.dof_frictionloss[dof])
            if friction < 1.5 * gravity_torque:
                failures.append(
                    f"{name} dry friction {friction:.3f} Nm is below "
                    f"1.5x spawn gravity torque {gravity_torque:.3f} Nm")
        gravity_only_duration_s = 120.0
        maximum_drift = {name: 0.0 for name, _, _, _ in prone_joints}
        for _ in range(round(gravity_only_duration_s / model.opt.timestep)):
            mujoco.mj_step(model, data)
            for name, _, qpos, initial in prone_joints:
                maximum_drift[name] = max(
                    maximum_drift[name],
                    abs(float(data.qpos[qpos]) - initial))
        for name, _, _, _ in prone_joints:
            drift = maximum_drift[name]
            limit = 0.005
            if drift > limit:
                failures.append(
                    f"{name} drifts {drift:.4f} rad in "
                    f"{gravity_only_duration_s:.0f} s gravity-only "
                    f"settling (limit {limit:.3f} rad)")

    if failures:
        for failure in failures:
            print(f"FAIL {failure}")
        return 1

    print(
        f"PASS model={args.model} bodies={model.nbody - 1} joints={model.njnt - 1} "
        f"mass={robot_mass:.6f}kg timestep={model.opt.timestep:.6f}s"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
