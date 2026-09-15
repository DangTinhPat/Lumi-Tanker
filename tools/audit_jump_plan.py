#!/usr/bin/env python3
"""Offline phase metrics; never a physical jump acceptance certificate."""
import argparse
import json
from pathlib import Path

import numpy as np

def audit(plan, model):
    q, v, tau = (np.asarray(plan[k], dtype=float) for k in
                 ('q', 'v', 'torque_per_side_nm'))
    counts = np.asarray(plan['phase_intervals'], dtype=float)
    durations = np.asarray(plan['phase_durations_s'], dtype=float)
    if (counts.shape != (3,) or not np.isfinite(counts).all() or
            np.any(counts <= 0) or np.any(counts != np.floor(counts)) or
            durations.shape != (3,) or np.any(durations <= 0) or
            q.shape != (sum(counts)+1, 6) or v.shape != q.shape or
            tau.shape != (sum(counts), 3) or
            not all(np.isfinite(x).all() for x in (q, v, tau, durations))):
        raise ValueError('Invalid plan arrays')
    counts = counts.astype(int)
    result = {'gazebo_validated': False, 'phases': {}}
    begin = 0
    for name, count, duration in zip(('thrust', 'flight', 'landing'), counts, durations):
        end = begin+count
        rates = (v[begin:end, 3:]+v[begin+1:end+1, 3:])/2
        # Both legs/wheels: the optimizer stores torque PER SIDE.
        power = 2*tau[begin:end]*rates
        result['phases'][name] = {
            'duration_s': float(duration),
            'positive_motor_work_j': float(np.maximum(power, 0).sum()*duration/count),
            'negative_motor_work_j': float(np.minimum(power, 0).sum()*duration/count),
            'peak_leg_rate_rad_s': float(np.abs(rates[:, :2]).max()),
            'leg_torque_at_95pct_fraction': float((np.abs(tau[begin:end, :2]) >= 4.75).mean()),
            'minimum_knee_margin_rad': float(np.min(q[begin:end+1, 4])+1.6),
        }
        dt = duration/count
        acceleration_errors = []
        normal_forces = []
        friction_margins = []
        contact_defects = []
        for i in range(begin, end):
            qm = (q[i]+q[i+1])/2+dt*(v[i]-v[i+1])/8
            vm = (v[i]+v[i+1])/2
            M, bias, _, _, _, J = (np.asarray(x) for x in model(qm, vm))
            rhs = np.r_[np.zeros(3), 2*tau[i]]-bias.ravel()
            rhs -= np.r_[np.zeros(3), .04*vm[3:5], .002*vm[5]]
            if name != 'flight':
                eps = 1e-6/max(1., np.linalg.norm(vm))
                jp = np.asarray(model(qm+eps*vm, vm)[5])
                jm = np.asarray(model(qm-eps*vm, vm)[5])
                jdot_v = ((jp-jm)/(2*eps))@vm
                kkt = np.block([[M, -J.T], [J, np.zeros((2, 2))]])
                solution = np.linalg.solve(kkt, np.r_[rhs, -jdot_v])
                acceleration = solution[:6]
                normal_forces.append(float(solution[-1]))
                friction_margins.append(float(solution[-1]-abs(solution[-2])))
                contact_defects.append(float(np.linalg.norm(J@((v[i+1]-v[i])/dt)+jdot_v)))
            else:
                acceleration = np.linalg.solve(M, rhs)
            acceleration_errors.append(float(np.linalg.norm(acceleration-(v[i+1]-v[i])/dt)))
        result['phases'][name]['max_acceleration_defect_norm'] = max(acceleration_errors)
        if normal_forces:
            result['phases'][name]['minimum_reconstructed_normal_force_n'] = min(normal_forces)
            result['phases'][name]['minimum_reconstructed_mu1_margin_n'] = min(friction_margins)
            result['phases'][name]['max_contact_acceleration_defect_m_s2'] = max(contact_defects)
        begin = end
    for name, index in (('takeoff', counts[0]), ('touchdown', sum(counts[:2]))):
        M, _, com, com_v, _, J = (np.asarray(x) for x in model(q[index], v[index]))
        velocity = v[index]
        contact_v = J@velocity
        # Perfectly inelastic sticking impact is a diagnostic only. A negative
        # normal impulse or friction violation makes this assumed mode invalid.
        response = np.linalg.solve(M, J.T)
        impulse = np.linalg.solve(J@response, -contact_v)
        post = velocity+response@impulse
        result[name] = {
            'com_z_m': float(com.ravel()[2]),
            'com_velocity_m_s': com_v.ravel().tolist(),
            'contact_velocity_m_s': contact_v.tolist(),
            'body_pitch_rate_rad_s': float(velocity[2]),
            'sticking_impulse_ns': impulse.tolist(),
            'sticking_mu1_feasible': bool(impulse[1] >= -1e-9 and abs(impulse[0]) <= impulse[1]+1e-9),
            'impact_energy_loss_j': float((velocity@M@velocity-post@M@post)/2),
        }
    return result


if __name__ == '__main__':
    from jump_dynamics import build_dynamics, model_metadata
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('urdf', type=Path)
    parser.add_argument('plans', type=Path, nargs='+')
    args = parser.parse_args()
    model = build_dynamics(args.urdf)
    metadata = model_metadata(args.urdf)
    reports = {}
    for path in args.plans:
        plan = json.loads(path.read_text())
        if plan['urdf_sha256'] != metadata['urdf_sha256']:
            raise ValueError('Plan/model hash mismatch')
        reports[str(path)] = audit(plan, model)
    print(json.dumps(reports, indent=2, allow_nan=False))
