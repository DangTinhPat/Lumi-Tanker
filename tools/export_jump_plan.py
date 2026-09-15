#!/usr/bin/env python3
"""Export a converged offline plan to an opt-in ROS development profile."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml

from jump_dynamics import build_dynamics, model_metadata
from jump_lqr import compute_gains


def export(plan_path, urdf_path, output_path, precision_preparation=False, pose_priority=1.):
    plan = json.loads(Path(plan_path).read_text())
    if (not plan.get('solver_success') or plan.get('status') != 'Solve_Succeeded'
            or not np.isfinite(plan['max_constraint_violation'])
            or plan['max_constraint_violation'] > 1e-5):
        raise ValueError('Only a converged, constraint-checked candidate can be exported')
    if hashlib.sha256(Path(urdf_path).read_bytes()).hexdigest() != plan['urdf_sha256']:
        raise ValueError('Plan URDF differs from this model')
    q, v, tau = map(np.asarray, (plan['q'], plan['v'], plan['torque_per_side_nm']))
    counts, ds = plan['phase_intervals'], np.array(plan['phase_durations_s'])
    t = np.r_[0., np.cumsum(np.repeat(ds/counts, counts))]
    mid = (t[:-1]+t[1:])/2
    if len(t)>256 or q.shape != (len(t),6) or v.shape != q.shape or tau.shape != (len(t)-1,3):
        raise ValueError('Invalid trajectory dimensions')
    if not all(np.isfinite(x).all() for x in (q,v,tau,t)):
        raise ValueError('Nonfinite trajectory')
    model = build_dynamics(urdf_path)
    def static_torque(state):
        values = model(state, np.zeros(6))
        force = np.array([0., model_metadata(urdf_path)['total_mass_kg']*9.81])
        residual = np.array(values[1]).ravel()-np.array(values[5]).T@force
        if np.max(np.abs(residual[:3])) > .001:
            raise ValueError('Endpoint is not a stationary supported posture')
        return residual[3:]/2
    prep, final = static_torque(q[0]), static_torque(q[-1])
    node_tau = np.array([np.interp(t,mid,col) for col in tau.T]).T
    node_tau[-1] = final
    rows = np.c_[t, q[:,2], v[:,2], q[:,3:5], v[:,3:5], node_tau[:,:2],v[:,5],node_tau[:,2]]
    # Only remove solver tolerance roundoff at physical bounds, not violations.
    for col, low, high in ((1,-.3,.3),(3,-.6,1.05),(4,-1.6,.2),(5,-28.274334,28.274334),
                           (6,-28.274334,28.274334),(7,-5,5),(8,-5,5),(10,-.13,.13)):
        if np.any(rows[:,col]<low-1e-6) or np.any(rows[:,col]>high+1e-6):
            raise ValueError('Trajectory exceeds runtime sample bounds')
        rows[:,col] = np.clip(rows[:,col],low,high)
    gains = compute_gains(plan, model, pose_priority=pose_priority)
    if gains.shape != (len(t),3,12) or not np.isfinite(gains).all():
        raise ValueError('Invalid offline feedback gains')
    rows = np.c_[rows,q[:,:2],v[:,:2],q[:,5],gains.reshape(len(t),36)]
    params = {
        'jump.enabled': True, 'jump.profile': 'manual_curve_experimental',
        'jump.optimized_plan': True, 'jump.wheel_frame_compensation': True,
        'jump.plan_samples': rows.ravel().tolist(), 'jump.plan_durations_s': ds.tolist(),
        'jump.plan_touchdown_com_z_m': float(model(q[sum(counts[:2])],v[sum(counts[:2])])[2][2]),
        'jump.plan_prep_hip_torque_nm': float(prep[0]),
        'jump.plan_prep_knee_torque_nm': float(prep[1]),
        'jump.crouch_knee_rad': float(rows[0,4]), 'jump.crouch_duration_s': 3.,
        'jump.extension_knee_rad': 0., 'jump.landing_knee_rad': float(rows[-1,4]),
        'jump.settle_duration_s': 1.2,
        'jump.leg_torque_limit_nm': 5., 'jump.wheel_torque_limit_nm': .13,
        'station_keeper.enabled': True, 'mpc.min_pendulum_length': .125,
        'mpc.max_ground_accel': 2., 'mpc.max_ground_velocity': .35,
        'pose_hip_min_rad': -.6, 'pose_hip_max_rad': 1.05,
        'pose_knee_min_rad': -1.6, 'pose_knee_max_rad': .2,
    }
    if precision_preparation:
        # Match the independently tested precision_pose preparation settings.
        params.update({'jump.crouch_duration_s': 11.,
                       'jump.plan_preparation_wait_s': 8.,
                       'station_keeper.position_gain_hz': 3.,
                       'station_keeper.acquire_dwell_s': .5,
                       'station_keeper.yaw_gain_hz': 2.,
                       'station_keeper.max_forward_m_s': .08,
                       'station_keeper.max_yaw_rad_s': .25})
    header = ('# GENERATED offline candidate; NOT validated in Gazebo or on hardware.\n'
              f'# Offline leg-pose cost multiplier: {pose_priority}\n'
              f'# Source URDF SHA256: {plan["urdf_sha256"]}\n'
              '# Requires jump_model:=true, jump_leg_speed:=28.274333882308138,\n'
              '# jump_leg_velocity_damping:=0.6. Default profiles remain unchanged.\n')
    Path(output_path).write_text(header+yaml.safe_dump({'balance_controller_node':{'ros__parameters':params}}))
    print(f'Exported {len(rows)} samples to {output_path}; Gazebo validation still required')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('plan', type=Path)
    parser.add_argument('urdf', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--precision-preparation', action='store_true')
    parser.add_argument('--pose-priority', type=float, default=1.)
    args = parser.parse_args()
    export(args.plan,args.urdf,args.output,args.precision_preparation,args.pose_priority)
