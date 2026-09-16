#!/usr/bin/env python3
"""Generate exact planar TAP FK constants from expanded URDF (SI units).

The URDF COM origins are already in link axes; inertial origin rotations only
rotate tensors. Refuse non-planar joint frames rather than silently fitting them.
"""
import argparse
import hashlib
import math
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET


def vector(element, key, default="0 0 0"):
    return tuple(map(float, element.get(key, default).split())) if element is not None else (0., 0., 0.)


def extract(xml):
    root = ET.fromstring(xml)
    links = {link.get("name"): link for link in root.findall("link")}
    joints = {joint.get("name"): joint for joint in root.findall("joint")}
    parts = []
    for name in ["cad_body"] + [f"cad_{s}_{p}" for s in ("left", "right") for p in ("thigh", "shin", "wheel_link")]:
        inertial = links[name].find("inertial")
        mass = float(inertial.find("mass").get("value"))
        origin = inertial.find("origin")
        xyz = vector(origin, "xyz")
        roll, _, yaw = vector(origin, "rpy")
        if abs(roll) > 1e-9 or abs(yaw) > 1e-9:
            raise ValueError("Only pitch-rotated inertial axes are supported")
        # Iyy is invariant to a rotation about Y, including body mesh -pi/2.
        iyy = float(inertial.find("inertia").get("iyy"))
        parts.append((mass, xyz[0], xyz[2], iyy))
    origins = []
    for side in ("left", "right"):
        leg = []
        for kind in ("hip", "knee", "wheel"):
            joint = joints[f"cad_{side}_{kind}"]
            if vector(joint.find("axis"), "xyz") != (0., 1., 0.):
                raise ValueError("TAP control requires all joint axes +Y")
            if any(abs(v) > 1e-9 for v in vector(joint.find("origin"), "rpy")):
                raise ValueError("Nonzero joint frame rotation is unsupported")
            leg.append(vector(joint.find("origin"), "xyz"))
        origins.append(leg)
    if any(math.hypot(p[1], p[2]) > 1e-6 for p in (parts[3], parts[6])):
        raise ValueError("Wheel COM must lie on its axle (wheel spin is omitted)")
    return parts, origins


def rotate(x, z, angle):
    c, s = math.cos(angle), math.sin(angle)
    return c*x+s*z, -s*x+c*z


def evaluate(parts, origins, hip, knee, radius):
    mass = sum(p[0] for p in parts)
    mx, mz = parts[0][0]*parts[0][1], parts[0][0]*parts[0][2]
    ax = az = 0.
    for side, offsets in enumerate(origins):
        px, _, pz = offsets[0]
        for j, angle in enumerate((hip, hip+knee, hip+knee)):
            if j:
                dx, dz = rotate(offsets[j][0], offsets[j][2], hip if j == 1 else hip+knee)
                px, pz = px+dx, pz+dz
            m, cx, cz, _ = parts[1+3*side+j]
            dx, dz = rotate(cx, cz, angle)
            mx, mz = mx+m*(px+dx), mz+m*(pz+dz)
        ax, az = ax+px/2, az+pz/2
    return mx/mass-ax, mz/mass-az+radius


def leg_inertia(parts, origins, hip, knee):
    """Rotational inertia (kg*m^2) about the hip and knee joint AXES (each
    +Y, per the URDF), i.e. the actual swinging assembly each joint's motor
    has to accelerate/decelerate: hip swings thigh+shin+wheel, knee swings
    shin+wheel only. Parallel-axis theorem per part: I_axis = Iyy_com +
    m*d^2, d = planar (XZ) distance from that part's COM to the joint axis
    at the given pose - mirrors evaluate()'s own FK loop structure. Averaged
    across both sides (identical by the CAD's left/right symmetry; averaging
    only guards against float roundoff, not a real asymmetry).
    """
    hip_total = knee_total = 0.
    for side, offsets in enumerate(origins):
        hip_x, _, hip_z = offsets[0]
        knee_dx, knee_dz = rotate(offsets[1][0], offsets[1][2], hip)
        knee_x, knee_z = hip_x+knee_dx, hip_z+knee_dz
        # Anchor for each part's own joint-origin chain, exactly mirroring
        # evaluate()'s cumulative px/pz: thigh (j=0) hangs off the hip
        # directly; shin (j=1) off the knee; wheel (j=2) off its own joint,
        # itself offset from the knee by offsets[2] (rotated by hip+knee,
        # the shin's orientation) - using knee_x/knee_z directly for the
        # wheel here (an earlier version of this function did) silently
        # dropped that last link segment, understating the wheel's real
        # lever arm from both axes by ~0.11 m (caught by
        # tests/test_leg_inertia.py's independent cross-check).
        anchor_x, anchor_z = hip_x, hip_z
        for j, angle in enumerate((hip, hip+knee, hip+knee)):
            if j:
                seg_dx, seg_dz = rotate(offsets[j][0], offsets[j][2], hip if j == 1 else hip+knee)
                anchor_x, anchor_z = anchor_x+seg_dx, anchor_z+seg_dz
            m, cx, cz, iyy = parts[1+3*side+j]
            dx, dz = rotate(cx, cz, angle)
            com_x, com_z = anchor_x+dx, anchor_z+dz
            hip_total += iyy + m*((com_x-hip_x)**2+(com_z-hip_z)**2)
            if j:  # shin, wheel: also swung by the knee joint
                knee_total += iyy + m*((com_x-knee_x)**2+(com_z-knee_z)**2)
    return hip_total/2, knee_total/2


def neutral(parts, origins, knee, radius):
    lo, hi = -1.3, 1.3
    flo = evaluate(parts, origins, lo, knee, radius)[0]
    if flo*evaluate(parts, origins, hi, knee, radius)[0] > 0:
        raise ValueError("No neutral hip root within provisional limits")
    for _ in range(70):
        mid = (lo+hi)/2
        if flo*evaluate(parts, origins, mid, knee, radius)[0] <= 0:
            hi = mid
        else:
            lo, flo = mid, evaluate(parts, origins, mid, knee, radius)[0]
    return (lo+hi)/2


def render(xml, radius):
    parts, origins = extract(xml)
    scalar = lambda x: f"{x:.12e}F"
    rows = lambda entries: ",\n".join("    {"+", ".join(map(scalar, row))+"}" for row in entries)
    knee = 0.
    hip = neutral(parts, origins, knee, radius)
    axle = abs(sum(o[1] for o in origins[0])-sum(o[1] for o in origins[1]))
    hip_inertia, knee_inertia = leg_inertia(parts, origins, hip, knee)
    result = "// Generated by tools/generate_control_model.py; do not edit.\n"
    canonical = ET.canonicalize(xml, strip_text=True, with_comments=False)
    result += "// URDF SHA256: "+hashlib.sha256(canonical.encode()).hexdigest()+"\n"
    result += "#pragma once\n#include <array>\nnamespace robot_control::geometry {\n"
    result += "struct Part { float mass, x, z, iyy; };\nstruct Origin { float x, y, z; };\n"
    result += "inline constexpr std::array<Part, 7> parts{{\n"+rows(parts)+"\n}};\n"
    result += "inline constexpr std::array<Origin, 6> origins{{\n"+rows(origins[0]+origins[1])+"\n}};\n"
    for key, value in {"total_mass_kg":sum(p[0] for p in parts), "wheel_radius_m":radius, "axle_spacing_m":axle,
                       "neutral_hip_rad":hip, "neutral_knee_rad":knee,
                       # Real mass-derived rotational inertia about each joint axis at
                       # the neutral (CAD-zero knee) pose - see leg_inertia()'s docstring.
                       # Consumed by BalanceControllerNode to compute leg_kd's default
                       # as critical damping (Kd = 2*sqrt(Kp*I)) instead of a guess.
                       "leg_hip_inertia_kgm2":hip_inertia, "leg_knee_inertia_kgm2":knee_inertia}.items():
        result += f"inline constexpr float {key} = {scalar(value)};\n"
    result += "}  // namespace robot_control::geometry\n"
    return result


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", type=Path, help="Expanded URDF, otherwise expand current description")
    parser.add_argument("--xacro", default="xacro")
    parser.add_argument("--wheel-radius", type=float, default=.03)
    parser.add_argument("--output", type=Path, default=root/"src/tap_control/core/include/robot_control/GeneratedGeometry.hpp")
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    xml = args.urdf.read_text() if args.urdf else subprocess.check_output([args.xacro, str(root/"src/tap_robot_description/description/cad_robot/robot.urdf.xacro")], text=True)
    content = render(xml, args.wheel_radius)
    if args.check:
        if args.output.read_text() != content:
            raise SystemExit("Generated control geometry is stale; regenerate it")
    else:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(content)
    parts, origins = extract(xml)
    for knee in (0., -.3, -.6, -.9, -1.2):
        hip = neutral(parts, origins, knee, args.wheel_radius)
        _, height = evaluate(parts, origins, hip, knee, args.wheel_radius)
        print(f"knee={knee:.3f} hip={hip:.9f} COM height={height:.6f} m (same signs both legs)")


if __name__ == "__main__":
    main()
