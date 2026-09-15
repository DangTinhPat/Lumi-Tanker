#!/usr/bin/env python3
"""Optimistic scalar jump screen along the existing COM-neutral squat curve.

Not inverse dynamics or a trajectory optimizer: no link rotational kinetic
energy, contact loss, losses or motor torque-speed curve. This only rejects
overoptimistic height targets on this particular upright symmetric path.
"""
import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/tap_teleop'))
from tap_teleop.deep_squat import generate_deep_squat
from tap_teleop.pose_curves import RobotGeometry


def screen(profile, mass, torque, speed):
    q = np.asarray(profile['table'], dtype=float)[::-1]
    z = np.asarray(profile['com_height_m'], dtype=float)[::-1]
    dq = np.abs(np.diff(q, axis=0))
    dz = np.diff(z)
    # Sum positive motor work on ALL four leg joints. Overestimates achievable
    # work because individual joint torques also have to regulate body pitch.
    work = np.r_[0., np.cumsum(torque * dq.sum(axis=1))]
    available = np.maximum(0., work - mass * 9.81 * (z-z[0]))
    energy_rise = available / (mass * 9.81)
    z_rate = np.gradient(z)
    q_rate = np.max(np.abs(np.gradient(q, axis=0)), axis=1)
    velocity_bound = speed * np.divide(z_rate, q_rate, out=np.zeros_like(z_rate), where=q_rate > 0)
    speed_rise = velocity_bound**2 / (2 * 9.81)
    rise = np.minimum(energy_rise, speed_rise)
    best = int(np.argmax(rise))
    return {'screen_only': True, 'mass_kg': mass, 'joint_torque_cap_nm': torque,
            'joint_speed_cap_rad_s': speed, 'upright_path_stroke_m': float(z[-1]-z[0]),
            'optimistic_ballistic_rise_bound_m': float(rise[best]),
            'candidate_pose_rad': q[best].tolist(),
            'candidate_com_height_m': float(z[best]),
            'candidate_vertical_speed_bound_m_s': float(velocity_bound[best]),
            'candidate_work_bound_j': float(work[best])}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--knee-depth', type=float, default=1.6)
    args = parser.parse_args()
    if not math.isfinite(args.knee_depth) or not 0 < args.knee_depth <= 1.6:
        parser.error('depth must be positive and at most the tested CAD envelope 1.6 rad')
    path = ROOT / 'src/tap_robot_description/description/cad_robot/robot.urdf.xacro'
    profile = generate_deep_squat(path, knee_max=args.knee_depth,
                                  hip_bounds=(-.6, 1.05), samples=151)
    mass = RobotGeometry(path).mass
    result = {'urdf_sha256': profile['urdf_sha256'],
              'assumptions': 'Four joints at 5 Nm; speed cases are separate hypothetical boxes, not a verified motor curve.',
              'rated_speed_case': screen(profile, mass, 5., 120*2*math.pi/60),
              'graph_plateau_speed_case': screen(profile, mass, 5., 190*2*math.pi/60),
              'maximum_speed_case_unverified_torque': screen(profile, mass, 5., 420*2*math.pi/60)}
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
