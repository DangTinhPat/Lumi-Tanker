#!/usr/bin/env python3
"""Offline jump diagnostics; no ROS, simulation, or artifact writes.

Usage: python3 tools/analyze_jump_trace.py log/jump_<id> [...]
COM velocities use local linear fits, not contact-force measurements. Applied
effort times relative joint velocity estimates actuator mechanical power only.
Samples are asynchronous; source-stamp spreads are reported, not hidden.
Centroidal momentum uses archived URDF spin and orbital terms in world axes.
Optional body_angular_velocity/world_angular_velocity sample vectors are rad/s;
otherwise full angular velocity is estimated from pose-source quaternions.
"""

import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


JOINTS = [f"cad_{side}_{joint}" for side in ("left", "right")
          for joint in ("hip", "knee", "wheel")]
LEGS = [0, 1, 3, 4]


def vector(value, size=3):
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f"Expected a finite {size}-vector")
    return result


def normalized_quaternion(value):
    q = vector(value, 4)
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or norm < 1e-12:
        raise ValueError("Invalid pose quaternion")
    return q / norm


def skew(v):
    x, y, z = v
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def quaternion_rotation(value):
    q = normalized_quaternion(value)
    cross = skew(q[:3])
    return np.eye(3) + 2 * q[3] * cross + 2 * cross @ cross


def axis_rotation(axis, angle):
    cross = skew(axis)
    return np.eye(3) + np.sin(angle) * cross + (1 - np.cos(angle)) * cross @ cross


def urdf_origin(element):
    origin = element.find("origin")
    if origin is None:
        return np.zeros(3), np.eye(3)
    xyz = vector([float(x) for x in origin.get("xyz", "0 0 0").split()])
    rpy = vector([float(x) for x in origin.get("rpy", "0 0 0").split()])
    rx, ry, rz = [axis_rotation(axis, angle) for axis, angle in zip(np.eye(3), rpy)]
    return xyz, rz @ ry @ rx


class MomentumModel:
    """Independent native-CAD rigid-body FK, using only an expanded URDF.

    Inertias are about link COMs. URDF inertial xyz is already link-expressed;
    only the tensor receives the inertial rpy rotation. Visuals are irrelevant.
    """

    def __init__(self, root):
        self.inertials = {}
        links = {link.get("name") for link in root.findall("link")}
        for link in root.findall("link"):
            inertial = link.find("inertial")
            if inertial is None:
                continue
            mass = float(inertial.find("mass").get("value"))
            offset, rotation = urdf_origin(inertial)
            entries = inertial.find("inertia").attrib
            tensor = np.array([[float(entries["i" + "".join(sorted(a + b))])
                                for b in "xyz"] for a in "xyz"])
            if not np.isfinite(mass) or mass <= 0 or not np.isfinite(tensor).all():
                raise ValueError("Invalid URDF mass/inertia")
            moments = np.linalg.eigvalsh(tensor)
            if moments[0] <= 0 or moments[2] > moments[0] + moments[1] + 1e-12:
                raise ValueError("Nonphysical URDF inertia")
            self.inertials[link.get("name")] = (mass, offset, rotation @ tensor @ rotation.T)
        self.joints = []
        children = set()
        for joint in root.findall("joint"):
            kind = joint.get("type")
            if kind not in ("fixed", "revolute", "continuous") or joint.find("mimic") is not None:
                raise ValueError("Unsupported URDF joint")
            offset, rotation = urdf_origin(joint)
            if kind != "fixed":
                axis = vector([float(x) for x in joint.find("axis").get("xyz").split()])
                if not np.allclose(axis, [0, 1, 0]) or not np.allclose(rotation, np.eye(3)):
                    raise ValueError("Expected native CAD +Y joints with zero origin rotation")
            parent, child = joint.find("parent").get("link"), joint.find("child").get("link")
            if child in children or parent not in links or child not in links:
                raise ValueError("Invalid URDF tree")
            children.add(child)
            self.joints.append((joint.get("name"), kind, parent, child, offset, rotation))
        roots = links - children
        if len(roots) != 1 or not self.inertials:
            raise ValueError("Expected one free-root robot with inertials")
        self.root = roots.pop()
        if self.root == "cad_ground":
            raise ValueError("Use the free-body archived URDF, not the preview mount")
        self.mass = sum(item[0] for item in self.inertials.values())

    def evaluate(self, sample, omega_world, base_velocity=(0., 0., 0.)):
        """Return H_COM = sum(I_world*w + (r-COM) x m*(v-v_COM)).

        A common base velocity cancels exactly, so no differentiated base
        translation is needed for centroidal momentum.
        """
        states = {self.root: (vector(sample["body_position"]),
                              quaternion_rotation(sample["quaternion"]),
                              vector(base_velocity), vector(omega_world))}
        pending = list(self.joints)
        while pending:
            remaining = []
            for joint in pending:
                name, kind, parent, child, offset, origin_rotation = joint
                if parent not in states:
                    remaining.append(joint)
                    continue
                p, r, v, w = states[parent]
                displacement = r @ offset
                child_r = r @ origin_rotation
                child_w = w.copy()
                if kind != "fixed":
                    angle = float(sample["joints"]["positions"][name])
                    speed = float(sample["joints"]["velocities"][name])
                    if not np.isfinite([angle, speed]).all():
                        raise ValueError("Nonfinite joint state")
                    child_w += child_r @ np.array([0., speed, 0.])
                    child_r = child_r @ axis_rotation([0., 1., 0.], angle)
                states[child] = (p + displacement, child_r,
                                 v + np.cross(w, displacement), child_w)
            if len(remaining) == len(pending):
                raise ValueError("Disconnected or cyclic URDF tree")
            pending = remaining
        bodies = []
        for name, (mass, offset, tensor) in self.inertials.items():
            p, r, v, w = states[name]
            displacement = r @ offset
            bodies.append((name, mass, p + displacement,
                           v + np.cross(w, displacement), r @ tensor @ r.T @ w))
        com = sum(m * p for _, m, p, _, _ in bodies) / self.mass
        com_v = sum(m * v for _, m, _, v, _ in bodies) / self.mass
        spin = sum(h for _, _, _, _, h in bodies)
        orbital = sum(np.cross(p - com, m * (v - com_v)) for _, m, p, v, _ in bodies)
        return dict(com_world_m=com.tolist(), spin_world_kg_m2_s=spin.tolist(),
                    orbital_world_kg_m2_s=orbital.tolist(),
                    total_world_kg_m2_s=(spin + orbital).tolist())


def body_angular_rates(samples):
    """Prefer explicit full vectors; otherwise differentiate normalized xyzw.

    Optional sample fields: body_angular_velocity or world_angular_velocity
    (rad/s), with source_stamps['angular_velocity']. Ambiguous unframed vectors
    are deliberately not interpreted. Pose stamps, never receipt/sim time,
    drive nonuniform three-point differences (one-sided at endpoints).
    """
    if not samples:
        return []
    times = np.array([s["source_stamps"]["pose"] for s in samples], dtype=float)
    if not np.isfinite(times).all() or np.any(np.diff(times) < 0):
        raise ValueError("Pose source timestamps must be finite and nondecreasing")
    quaternions = np.array([normalized_quaternion(s["quaternion"]) for s in samples])
    unique, indices, inverse = np.unique(times, return_index=True, return_inverse=True)
    q = quaternions[indices].copy()
    for i in range(1, len(q)):
        if np.dot(q[i - 1], q[i]) < 0:
            q[i] *= -1
    if len(samples) and not np.allclose(np.abs(np.sum(quaternions * q[inverse], axis=1)), 1.):
        raise ValueError("Conflicting quaternions at the same pose source stamp")
    differentiated = None
    if len(unique) >= 2:
        dq = np.gradient(q, unique, axis=0, edge_order=2 if len(unique) >= 3 else 1)
        differentiated = 2 * (q[:, 3, None] * dq[:, :3] - dq[:, 3, None] * q[:, :3]
                              + np.cross(q[:, :3], dq[:, :3]))
    result = []
    for i, sample in enumerate(samples):
        stamp = sample["source_stamps"].get("angular_velocity")
        metadata = dict(source_stamp_s=stamp)
        if "world_angular_velocity" in sample:
            omega = vector(sample["world_angular_velocity"])
            metadata["source"] = "sample_world_angular_velocity"
        elif "body_angular_velocity" in sample:
            omega = quaternion_rotation(sample["quaternion"]) @ vector(sample["body_angular_velocity"])
            metadata["source"] = "sample_body_angular_velocity"
        elif differentiated is not None:
            k = inverse[i]
            omega = differentiated[k]
            lo, hi = max(0, k - 1), min(len(unique) - 1, k + 1)
            if len(unique) >= 3:
                lo, hi = (0, 2) if k == 0 else ((k - 2, k) if k == len(unique) - 1 else (lo, hi))
            metadata.update(source="differentiated_pose_quaternion", source_stamp_s=float(times[i]),
                            stencil_source_s=unique[lo:hi + 1].tolist(),
                            one_sided=bool(k in (0, len(unique) - 1)))
        else:
            omega = None
            metadata["source"] = "unavailable_insufficient_unique_pose_stamps"
        metadata["omega_world_rad_s"] = None if omega is None else omega.tolist()
        result.append(metadata)
    return result


def momentum_trace(root, samples):
    model = MomentumModel(root)
    rates = body_angular_rates(samples)
    rows = []
    for sample, rate in zip(samples, rates):
        omega = rate["omega_world_rad_s"]
        row = dict(angular_rate=rate, pose_source_s=sample["source_stamps"]["pose"],
                   joint_source_s=sample["source_stamps"].get("joints"),
                   controller_pitch_rate_rad_s=sample.get("control", {}).get("pitch_rate"))
        row["centroidal"] = None if omega is None else model.evaluate(sample, omega)
        rows.append(row)
    return rows, dict(
        mass_kg=model.mass, inertial_links=list(model.inertials),
        frame="world", reference="instantaneous whole-robot COM reconstructed from archived URDF",
        units="kg*m^2/s", angular_rate_sources=sorted({r["source"] for r in rates}),
        limitations=[
            "Archived inertials are uniform-solid estimates with provisional motor lumping; no extra rotor inertia.",
            "Pose and joint packets are asynchronous; no resampling or timing correction is applied.",
            "Quaternion differentiation normalizes and sign-unwraps unique pose packets using source timestamps; "
            "finite-rate noise, aliasing, gaps and one-sided endpoint bias remain. No smoothing or contact exclusion.",
            "Scalar controller pitch_rate is reported for comparison, not substituted for full 3D angular velocity.",
            "Explicit full-rate vectors require a declared body/world field; missing rate source stamps remain null.",
            "Common base translation velocity cancels in centroidal momentum; no COM velocity fit is used.",
            "Momentum variation is diagnostic, not an external torque or conservation proof; no contact wrench recorded."])


def analyze(path):
    samples = json.loads((path / "samples.json").read_text())
    summary = json.loads((path / "summary.json").read_text())
    t = np.array([s["sim"] for s in samples])
    pose_t = np.array([s["source_stamps"]["pose"] for s in samples])
    com = np.array([s["com"] for s in samples])
    flight = summary["flight"]
    takeoff = flight["takeoff_bracket_s"][1]
    landing = flight["landing_sim_s"]
    root = ET.parse(path / "expanded.urdf").getroot()
    momentum, momentum_info = momentum_trace(root, samples)
    origins = {}
    for name in JOINTS:
        joint = root.find(f"joint[@name='{name}']")
        axis = np.fromstring(joint.find("axis").get("xyz"), sep=" ")
        origin = joint.find("origin")
        if not np.allclose(axis, [0, 1, 0]) or not np.allclose(
                np.fromstring(origin.get("rpy", "0 0 0"), sep=" "), 0):
            raise ValueError("Expected native CAD +Y joints with zero origin rotation")
        origins[name] = np.fromstring(origin.get("xyz"), sep=" ")

    def wheel_centers(s):
        x, y, z, w = np.array(s["quaternion"]) / np.linalg.norm(s["quaternion"])
        skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
        body_rotation = np.eye(3) + 2 * w * skew + 2 * skew @ skew
        result = []
        for side in ("left", "right"):
            rotation = body_rotation.copy()
            position = np.array(s["body_position"])
            for joint in ("hip", "knee", "wheel"):
                name = f"cad_{side}_{joint}"
                position = position + rotation @ origins[name]
                angle = s["joints"]["positions"][name]
                c, sn = np.cos(angle), np.sin(angle)
                rotation = rotation @ np.array([[c, 0, sn], [0, 1, 0], [-sn, 0, c]])
            result.append(position)
        return np.array(result)

    def velocity(i):
        mask = np.abs(pose_t - pose_t[i]) <= .0120001
        times, indices = np.unique(pose_t[mask], return_index=True)
        if len(times) < 2:
            return [None] * 3
        return np.linalg.lstsq(
            np.column_stack((times - pose_t[i], np.ones(len(times)))),
            com[mask][indices], rcond=None)[0][0].tolist()

    def row(time):
        i = int(np.argmin(np.abs(t - time)))
        s = samples[i]
        j = s["joints"]
        wheels = wheel_centers(s)
        return dict(sim=s["sim"], relative_takeoff_s=s["sim"] - takeoff,
                    phase=s["phase"], contacts=s["contacts"],
                    com_m=s["com"], com_velocity_fit_m_s=velocity(i),
                    pitch_rad=s["pitch"],
                    pitch_rate_rad_s=s["control"].get("pitch_rate"),
                    angular_momentum=momentum[i],
                    position_rad=[j["positions"][n] for n in JOINTS],
                    velocity_rad_s=[j["velocities"][n] for n in JOINTS],
                    applied_nm=[j["efforts"][n] for n in JOINTS],
                    leg_command_nm=s["leg_commands"],
                    wheel_centers_world_m=wheels.tolist(),
                    com_minus_axle_world_m=(com[i] - wheels.mean(axis=0)).tolist(),
                    source_spread_s=s["measurement_stamp_spread_s"])

    transitions = []
    previous = None
    for s in samples:
        if s["phase"] != previous:
            transitions.append(row(s["sim"]))
            previous = s["phase"]
    thrust = next(r["sim"] for r in transitions if r["phase"] == "Thrust")
    # De-duplicate joint packets: snapshots may repeat a 200 Hz JointState.
    joint_samples = {}
    for s in samples:
        joint_samples.setdefault(s["source_stamps"]["joints"], s)
    packets = sorted(joint_samples.items())
    phase_stats = {}
    for phase in ("Thrust", "Flight", "Land"):
        selected = [(stamp, s) for stamp, s in packets
                    if s["phase"] == phase and s["stage"] != "estop"]
        if not selected:
            continue
        times = np.array([stamp for stamp, _ in selected])
        qd = np.array([[s["joints"]["velocities"][n] for n in JOINTS]
                       for _, s in selected])
        tau = np.array([[s["joints"]["efforts"][n] for n in JOINTS]
                        for _, s in selected])
        power = qd * tau
        caps = np.array([5, 5, .13, 5, 5, .13])
        def work(values):
            return np.sum(.5 * (values[1:] + values[:-1]) *
                          np.diff(times)[:, None], axis=0).tolist()

        phase_stats[phase] = dict(
            packets=len(times), span_s=float(times[-1] - times[0]),
            peak_abs_speed_rad_s=np.max(np.abs(qd), axis=0).tolist(),
            peak_abs_torque_nm=np.max(np.abs(tau), axis=0).tolist(),
            torque_cap_fraction=np.mean(np.abs(tau) >= caps * .99, axis=0).tolist(),
            positive_work_above_190rpm_fraction=np.mean(
                (np.abs(qd) > 19.89675347) & (power > 0), axis=0).tolist(),
            peak_positive_power_w=np.max(np.maximum(power, 0), axis=0).tolist(),
            peak_total_positive_leg_power_w=float(np.max(
                np.maximum(power[:, LEGS], 0).sum(axis=1))),
            positive_work_j=work(np.maximum(power, 0)),
            negative_work_j=work(np.minimum(power, 0)))

    contact_edges = []
    phase_edges = []
    last_contact = {}
    last_phase = None
    with (path / "events.jsonl").open() as stream:
        for line in stream:
            event = json.loads(line)
            kind = event["kind"]
            if kind in ("left", "right"):
                data = event["data"]
                value = data["data"]
                stamp = data.get("source_stamp") or data["stamp"]
                if value != last_contact.get(kind) and thrust <= stamp <= landing + .08:
                    contact_edges.append(dict(side=kind, contact=value, source_s=stamp,
                                              receipt_sim_s=event["sim"]))
                last_contact[kind] = value
            if kind == "jump":
                data = event["data"]["data"]
                phase = data.get("phase")
                if phase != last_phase and event["sim"] >= thrust - .02:
                    phase_edges.append(dict(sim=event["sim"], phase=phase,
                                            raw=data.get("raw")))
                last_phase = phase

    airborne = (t >= takeoff + .02) & (t <= landing - .02)
    valid_momentum = [i for i in np.flatnonzero(airborne)
                      if momentum[i]["centroidal"] is not None]
    if valid_momentum:
        h = np.array([momentum[i]["centroidal"]["total_world_kg_m2_s"] for i in valid_momentum])
        momentum_info["interior_flight"] = dict(
            selection="takeoff + 20 ms through landing - 20 ms; not a contact-wrench gate",
            samples=len(h), pose_source_span_s=pose_t[valid_momentum][[0, -1]].tolist(),
            first_world_kg_m2_s=h[0].tolist(), last_world_kg_m2_s=h[-1].tolist(),
            change_world_kg_m2_s=(h[-1] - h[0]).tolist(),
            component_range_kg_m2_s=np.ptp(h, axis=0).tolist(),
            max_vector_change_from_first_kg_m2_s=float(np.max(np.linalg.norm(h - h[0], axis=1))))
    fit_t = pose_t[airborne] - takeoff
    unique, idx = np.unique(fit_t, return_index=True)
    # Fit measured curvature too, to expose departures from ballistic motion.
    fit = np.polyfit(unique, com[airborne][idx, 2], 2)
    return dict(log=str(path), joint_order=JOINTS, failure=summary["failure"],
                flight=flight, impact_estimate=summary.get("impact_estimate"),
                ballistic_fit=dict(acceleration_m_s2=float(2 * fit[0]),
                                   takeoff_vz_m_s=float(fit[1])),
                phase_transitions=transitions,
                trace=[row(x) for x in [thrust, thrust + .02, thrust + .04,
                       thrust + .06, thrust + .08, *flight["takeoff_bracket_s"],
                       takeoff + .02, flight["apex_sim_s"], landing - .02,
                       landing, landing + .012, landing + .024]],
                phase_stats=phase_stats, contact_edges=contact_edges,
                event_phase_edges=phase_edges,
                angular_momentum=momentum_info,
                note="Power/work are sampled estimates; no contact wrench or electrical power recorded.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logs", nargs="+", type=Path)
    args = parser.parse_args()
    print(json.dumps([analyze(path) for path in args.logs], indent=2))


if __name__ == "__main__":
    main()
