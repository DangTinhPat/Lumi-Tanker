#!/usr/bin/env python3
"""Offline constrained sagittal jump planning. Never commands ROS or hardware.

An optimizer solution is a planning candidate, not Gazebo validation. Uses the
ADAM URDF rigid-body model and CasADi/IPOPT, in a separate PC-only environment.
"""
import argparse
import hashlib
import json
from pathlib import Path
import time

import casadi as ca
import numpy as np
from scipy.optimize import brentq
from scipy.interpolate import CubicHermiteSpline

from jump_dynamics import build_dynamics


def solve(urdf, output, intervals=20, height=.105, iterations=1500, seed_path=None,
          landing_margin=False, crouch_knee=-1.6, monotonic_thrust=False,
          contact_consistent=False, pitch_rate_limit=1.5):
    if not np.isfinite(crouch_knee) or not -1.6 <= crouch_knee < -.6:
        raise ValueError('Crouch knee must be in [-1.6,-0.6) rad')
    if not np.isfinite(pitch_rate_limit) or not .5 <= pitch_rate_limit <= 10.:
        raise ValueError('Planned pitch rate bound must be in [0.5,10] rad/s')
    optimizer_hash = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    dynamics = build_dynamics(urdf)
    qs, vs = ca.MX.sym('q', 6), ca.MX.sym('v', 6)
    js = dynamics(qs, vs)[5]
    contact_bias = ca.Function('contact_bias', [qs, vs], [ca.jacobian(js@vs, qs)@vs])
    opt = ca.Opti()
    counts = [intervals, 2*intervals, 2*intervals]
    total = sum(counts)
    q = opt.variable(6, total+1)
    v = opt.variable(6, total+1)
    torque = opt.variable(3, total)
    force = opt.variable(2, total)
    durations = opt.variable(3)
    opt.subject_to(opt.bounded(ca.DM([.08, .18, .3]), durations, ca.DM([.35, .5, 1.2])))
    opt.set_initial(durations, [.15, .3, .7])

    def standing(knee):
        def offset(hip):
            values = dynamics([0, 0, 0, hip, knee, 0], np.zeros(6))
            return float(values[2][0]-values[4][0])
        hip = brentq(offset, -.6, 1.05)
        state = np.array([0, 0, 0, hip, knee, -hip-knee])
        axle = np.array(dynamics(state, np.zeros(6))[4]).ravel()
        state[0] = -axle[0]
        state[1] = .03-axle[2]
        return state

    initial, final = standing(crouch_knee), standing(-.6)
    opt.subject_to(q[:, 0] == initial)
    opt.subject_to(v[:, 0] == 0)
    opt.subject_to(q[:5, -1] == final[:5])
    opt.subject_to(v[:, -1] == 0)
    opt.subject_to(opt.bounded(-.3, q[2, :], .3))
    opt.subject_to(opt.bounded(-.6, q[3, :], 1.05))
    opt.subject_to(opt.bounded(-1.6, q[4, :], .2))
    opt.subject_to(opt.bounded(.115, q[1, :], .5))
    opt.subject_to(opt.bounded(-28.274333882308138, v[3:5, :], 28.274333882308138))
    opt.subject_to(opt.bounded(-60, v[5, :], 60))
    opt.subject_to(opt.bounded(-5, torque[:2, :], 5))
    opt.subject_to(opt.bounded(-.13, torque[2, :], .13))
    takeoff, touchdown = counts[0], counts[0]+counts[1]
    if monotonic_thrust:
        opt.subject_to(v[4, :takeoff+1] >= 0)
    if landing_margin:
        # Reserve compression stroke and reduce nominal body angular velocity.
        opt.subject_to(q[4, touchdown:] >= -1.4)
        opt.subject_to(opt.bounded(-pitch_rate_limit, v[2, :], pitch_rate_limit))
        opt.subject_to(force[1, takeoff-1] <= 5.)
    zero = np.zeros(6)
    com_start = float(dynamics(initial, zero)[2][0])
    com_takeoff = dynamics(q[:, takeoff], v[:, takeoff])
    opt.subject_to(opt.bounded(np.sqrt(2*9.81*height), com_takeoff[3][2], 1.7))
    opt.subject_to(opt.bounded(-.025, com_takeoff[3][0], .025))
    apex = dynamics(q[:, takeoff+counts[1]//2], v[:, takeoff+counts[1]//2])
    opt.subject_to(apex[2][2]-com_takeoff[2][2] >= height)
    contact_start = float(dynamics(initial, zero)[4][0])-.03*sum(initial[2:])
    objective = 0
    for i in range(total+1):
        values = dynamics(q[:, i], v[:, i])
        com, axle, J = values[2], values[4], values[5]
        opt.subject_to(opt.bounded(-.01, com[0]-com_start, .01))
        opt.subject_to(axle[2] >= .03-1e-7)
        if i and (i <= takeoff or i >= touchdown):
            # Fixed endpoint positions already enforce these height equations.
            # Duplicating them makes the equality Jacobian rank deficient.
            if i != total:
                if contact_consistent:
                    opt.subject_to(opt.bounded(.03, axle[2], .03001))
                else:
                    opt.subject_to(axle[2] == .03)
            rolling_error = axle[0]-.03*ca.sum1(q[2:, i])-contact_start
            if contact_consistent:
                # Midpoint integration has position drift of order dt^3.
                # Allow 10 um numerical drift, not physical contact separation.
                opt.subject_to(opt.bounded(-1e-5, rolling_error, 1e-5))
            else:
                opt.subject_to(rolling_error == 0)
            opt.subject_to(opt.bounded(-.015, J@v[:, i], .015))
        objective += 2*q[2, i]**2+500*(com[0]-com_start)**2
    # Touchdown is continuous and near zero foot velocity, not an unmodeled
    # velocity reset that would erase the impact energy in the optimizer.
    Jtd = dynamics(q[:, touchdown], v[:, touchdown])[5]
    opt.subject_to(opt.bounded(-.003, Jtd@v[:, touchdown], .003))

    idx = 0
    for phase, count in enumerate(counts):
        dt = durations[phase]/count
        for _ in range(count):
            i = idx
            qm = (q[:, i]+q[:, i+1])/2 + dt*(v[:, i]-v[:, i+1])/8
            vm = (v[:, i]+v[:, i+1])/2
            # Bezier convex hull bounds protect the entire Hermite segment,
            # not just collocation nodes where a fast joint can hide overshoot.
            p1, p2 = q[:,i]+dt*v[:,i]/3, q[:,i+1]-dt*v[:,i+1]/3
            for point in (p1,p2):
                opt.subject_to(opt.bounded(-.3,point[2],.3))
                opt.subject_to(opt.bounded(-.6,point[3],1.05))
                opt.subject_to(opt.bounded(-1.6,point[4],.2))
                if landing_margin and phase == 2:
                    opt.subject_to(point[4] >= -1.4)
            opt.subject_to(opt.bounded(-28.274333882308138,
                3*(p2[3:5]-p1[3:5])/dt,28.274333882308138))
            if monotonic_thrust and phase == 0:
                opt.subject_to(p2[4] >= p1[4])
            M, bias, _, _, _, J = dynamics(qm, vm)
            a = (v[:, i+1]-v[:, i])/dt
            opt.subject_to(q[:, i+1]-q[:, i] == dt*vm)
            generalized = ca.vertcat(ca.DM.zeros(3), 2*torque[:, i])
            passive = ca.vertcat(ca.DM.zeros(3), 2*.02*vm[3:5], 2*.001*vm[5])
            opt.subject_to(M@a+bias+passive == generalized+J.T@force[:, i])
            if phase == 1:
                opt.subject_to(force[:, i] == 0)
            else:
                opt.subject_to(opt.bounded(0, force[1, i], 300))
                if contact_consistent:
                    opt.subject_to(opt.bounded(-.05, J@a+contact_bias(qm, vm), .05))
                if landing_margin and phase == 0 and i < takeoff-3:
                    opt.subject_to(force[1, i] >= 20.)
                opt.subject_to(opt.bounded(-force[1, i], force[0, i], force[1, i]))
            # Exact linear facets of the motoring taper inside the speed bound.
            # Negative work still admits full braking torque, without switches.
            for j in range(2):
                rate, tau = vm[3+j], torque[j, i]
                slope = 5/(28.274333882308138-19.896753472735355)
                cap = slope*28.274333882308138
                opt.subject_to(opt.bounded(-cap, tau+slope*rate, cap))
            # Finite wheel velocity servo: not an ideal torque source.
            opt.subject_to(opt.bounded(-41.88790204786391,
                vm[5]+torque[2, i]/.02, 41.88790204786391))
            objective += dt*(.002*ca.sumsqr(torque[:, i])+.00001*ca.sumsqr(a))
            if i:
                objective += .0002*ca.sumsqr(torque[:, i]-torque[:, i-1])
            idx += 1
    opt.minimize(objective+.1*ca.sumsqr(durations))

    # A geometric seed is deliberately not exported as a feasible trajectory.
    seed = []
    for i in range(total+1):
        if i <= takeoff:
            s = i/takeoff
            guess = standing(crouch_knee+(-.5-crouch_knee)*s)
        elif i <= touchdown:
            s = (i-takeoff)/(touchdown-takeoff)
            guess = standing(-.5-.1*s)
            guess[1] += .12*4*s*(1-s)
        else:
            guess = final
        seed.append(guess)
        opt.set_initial(q[:, i], guess)
    times = np.concatenate(([0.], np.cumsum(np.repeat([.15/counts[0], .3/counts[1], .7/counts[2]], counts))))
    seed_v = np.gradient(np.asarray(seed), times, axis=0)
    seed_v[0] = seed_v[-1] = 0
    opt.set_initial(v, seed_v.T)
    opt.set_initial(torque, 0)
    opt.set_initial(force[0, :], 0)
    opt.set_initial(force[1, :], 3.938*9.81)
    if seed_path:
        previous = json.loads(Path(seed_path).read_text())
        old_counts = previous['phase_intervals']
        ds = np.asarray(previous['phase_durations_s'])
        old_t = np.r_[0, np.cumsum(np.repeat(ds/old_counts, old_counts))]
        new_t = np.r_[0, np.cumsum(np.repeat(ds/counts, counts))]
        spline = CubicHermiteSpline(old_t, previous['q'], previous['v'])
        opt.set_initial(q, spline(new_t).T)
        opt.set_initial(v, spline(new_t, 1).T)
        for variable, key in ((torque, 'torque_per_side_nm'), (force, 'total_contact_force_n')):
            samples = np.asarray(previous[key])
            opt.set_initial(variable, np.array([np.interp((new_t[:-1]+new_t[1:])/2,
                (old_t[:-1]+old_t[1:])/2, column) for column in samples.T]))
        opt.set_initial(durations, previous['phase_durations_s'])
    opt.solver('ipopt', {'expand': True}, {'max_iter': iterations, 'tol': 1e-6,
        'acceptable_tol': 1e-5, 'print_level': 5, 'max_cpu_time': 240})
    began = time.monotonic()
    success = False
    try:
        solution = opt.solve()
        success = True
    except RuntimeError:
        solution = opt.debug
    stats = opt.stats()
    values = np.asarray(solution.value(opt.g)).ravel()
    lower = np.asarray(solution.value(opt.lbg)).ravel()
    upper = np.asarray(solution.value(opt.ubg)).ravel()
    errors = np.maximum(lower-values, values-upper)
    violation = float(errors.max(initial=0))
    candidate = {
        'schema': 1, 'solver_success': success, 'gazebo_validated': False,
        'optimizer_sha256': optimizer_hash,
        'status': stats.get('return_status'), 'max_constraint_violation': violation,
        'wall_s': time.monotonic()-began, 'requested_height_m': height,
        'landing_margin': landing_margin,
        'crouch_knee_rad': crouch_knee,
        'monotonic_thrust': monotonic_thrust,
        'contact_consistent': contact_consistent,
        'pitch_rate_limit_rad_s': pitch_rate_limit if landing_margin else None,
        'urdf_sha256': hashlib.sha256(Path(urdf).read_bytes()).hexdigest(),
        'coordinate_order': ['body_x', 'body_z', 'pitch', 'hip', 'knee', 'wheel'],
        'phase_intervals': counts, 'phase_durations_s': solution.value(durations).tolist(),
        'q': solution.value(q).T.tolist(), 'v': solution.value(v).T.tolist(),
        'torque_per_side_nm': solution.value(torque).T.tolist(),
        'total_contact_force_n': solution.value(force).T.tolist(),
        'worst_constraints': [{'index': int(i), 'violation': float(errors[i]),
                               'source': opt.debug.g_describe(int(i))}
                              for i in np.argsort(errors)[-5:] if errors[i] > 1e-7],
    }
    Path(output).write_text(json.dumps(candidate, indent=2, allow_nan=False))
    print(json.dumps({k: value for k, value in candidate.items()
                      if k not in ('q', 'v', 'torque_per_side_nm', 'total_contact_force_n')}))
    return success and violation < 1e-5


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('urdf', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--intervals', type=int, default=20)
    parser.add_argument('--height', type=float, default=.105)
    parser.add_argument('--iterations', type=int, default=1500)
    parser.add_argument('--seed', type=Path)
    parser.add_argument('--landing-margin', action='store_true')
    parser.add_argument('--crouch-knee', type=float, default=-1.6)
    parser.add_argument('--monotonic-thrust', action='store_true')
    parser.add_argument('--contact-consistent', action='store_true')
    parser.add_argument('--pitch-rate-limit', type=float, default=1.5)
    args = parser.parse_args()
    if args.intervals < 8 or not np.isfinite(args.height) or not .01 <= args.height <= .15:
        parser.error('Use >=8 intervals per thrust phase and a height in [0.01,0.15] m')
    raise SystemExit(0 if solve(args.urdf, args.output, args.intervals,
                               args.height, args.iterations, args.seed,
                               args.landing_margin, args.crouch_knee,
                               args.monotonic_thrust, args.contact_consistent,
                               args.pitch_rate_limit) else 1)
