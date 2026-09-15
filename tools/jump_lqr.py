"""Offline TVLQR for the ADAM symmetric six-DOF jump model.

State is [body_x, body_z, pitch, hip, knee, wheel, followed by their rates].
All joint coordinates are native +Y; nominal inputs are PER-SIDE NET ACTIVE
torques [hip, knee, wheel] in Nm. Runtime outer input is
u = tau_nom - K @ (state - state_ref). With leg position stiffness zero and
planned joint-rate references, the plugin then applies u_leg + 0.6*(v_ref-v).
Do not cancel this local feedback in the effort command: its preclip can make
cancellation invalid. The outer TVLQR plant is A_eff = A - B*D_state, where
D_state[0,9] = D_state[1,10] = 0.6. Passive damping remains separate.

For wheels, invert using qd_cmd = qd_measured + u_wheel/0.02, retaining the
actual command saturation, torque saturation and positive-work cutoff. At the
reference v=v_ref, the nominal u is the nominal net active torque. Away from
it, net torque is tau_nom-(K+D_state)*error only while all clips are inactive.
Clipping invalidates the exact linearization, including near saturated nominal
points. This law assumes the plugin's local gain is exactly 0.6 Nm/(rad/s).

Gains assume the reference's fixed contact schedule and continuous transition
state (no impact/reset map). They neither enforce actuator/contact bounds nor
establish plan feasibility, stable landing, or hardware capability.
"""

import casadi as ca
import numpy as np
from scipy.linalg import expm


LOCAL_FEEDBACK = np.zeros((3, 12))
LOCAL_FEEDBACK[0, 9] = LOCAL_FEEDBACK[1, 10] = .6
LOCAL_FEEDBACK.setflags(write=False)


def build_linearization(dynamics_function, grounded):
    """Return a CasADi function (q, v, tau) -> (xdot, A, B).

    A differentiates the bare plant xdot with respect to [q, v]; B differentiates
    with respect to per-side net torque. compute_gains adds local feedback.
    The supplied function has the API of
    jump_dynamics.build_dynamics; bias excludes active/passive/contact forces.
    Grounded perturbations are meaningful on the no-slip constraint manifold.
    No artificial constraint stabilization or external body wrench is added.
    """
    q, v, tau = ca.MX.sym('q', 6), ca.MX.sym('v', 6), ca.MX.sym('tau', 3)
    M, bias, _, _, _, J = dynamics_function(q, v)
    actuation = ca.vertcat(ca.MX.zeros(3, 3), 2 * ca.MX.eye(3))
    passive = ca.vertcat(ca.MX.zeros(3), 2 * .02 * v[3:5], 2 * .001 * v[5])
    rhs = actuation @ tau - bias - passive
    if grounded:
        # Differentiate J(q)*v holding v fixed to obtain Jdot(q,v)*v.
        jdot_v = ca.jacobian(J @ v, q) @ v
        system = ca.blockcat([[M, -J.T], [J, ca.MX.zeros(2, 2)]])
        acceleration = ca.solve(system, ca.vertcat(rhs, -jdot_v))[:6]
    else:
        acceleration = ca.solve(M, rhs)
    xdot = ca.vertcat(v, acceleration)
    A = ca.horzcat(ca.jacobian(xdot, q), ca.jacobian(xdot, v))
    B = ca.jacobian(xdot, tau)
    return ca.Function(
        'tap_jump_grounded_linearization' if grounded else 'tap_jump_flight_linearization',
        [q, v, tau], [xdot, A, B], ['q', 'v', 'tau'], ['xdot', 'A', 'B'])


def _discretize(A, B, dt):
    """Exact ZOH transition for a frozen continuous linearization."""
    n, m = B.shape
    generator = np.zeros((n + m, n + m))
    generator[:n, :n], generator[:n, n:] = A, B
    transition = expm(dt * generator)
    return transition[:n, :n], transition[:n, n:]


def _cost_matrices(grounded, pose_priority=1.):
    # Dimensionless squared errors, integrated in seconds. Position scales
    # are m/rad; rate scales are m/s and rad/s. Pitch has highest priority.
    scales = np.array([.01 if grounded else .05, .03, .03, .15, .15, 1.,
                       .1, .5, .5, 2., 2., 10.])
    priorities = np.array([1., 1., 25., 1., 1., .01, 1., 1., 10., 1., 1., .01])
    priorities[3:5] *= pose_priority
    Q = np.diag(priorities / scales**2)
    # Sum normalized outer-input cost of BOTH sides. These scales are the
    # provisional torque caps, not a claim that outer or net torque is bounded.
    R = np.diag(2 / np.array([5., 5., .13])**2)
    return Q, R


def _plan_arrays(plan):
    try:
        counts = np.asarray(plan['phase_intervals'], dtype=float)
        durations = np.asarray(plan['phase_durations_s'], dtype=float)
        q = np.asarray(plan['q'], dtype=float)
        v = np.asarray(plan['v'], dtype=float)
        torque = np.asarray(plan['torque_per_side_nm'], dtype=float)
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError('Plan requires numeric phases, q, v and per-side torques') from exc
    if (counts.shape != (3,) or not np.isfinite(counts).all()
            or np.any(counts < 1) or np.any(counts != np.floor(counts))):
        raise ValueError('phase_intervals must contain three positive integers: stance, flight, land')
    if durations.shape != (3,) or not np.isfinite(durations).all() or np.any(durations <= 0):
        raise ValueError('phase_durations_s must contain three finite positive durations')
    total = sum(int(c) for c in counts)
    if q.shape != (total + 1, 6) or v.shape != q.shape or torque.shape != (total, 3):
        raise ValueError('Expected q/v [sum(intervals)+1,6] and torque_per_side_nm [sum(intervals),3]')
    if not all(np.isfinite(a).all() for a in (q, v, torque)):
        raise ValueError('Plan states and torques must be finite')
    order = plan.get('coordinate_order', ['body_x', 'body_z', 'pitch', 'hip', 'knee', 'wheel'])
    if list(order) not in (['body_x', 'body_z', 'pitch', 'hip', 'knee', 'wheel'],
                          ['body_x', 'body_z', 'body_pitch', 'hip', 'knee', 'wheel']):
        raise ValueError('Unsupported plan coordinate_order')
    return counts.astype(int), durations, q, v, torque


def compute_gains(plan_dict, dynamics_function, pose_priority=1.):
    """Return finite-horizon gains of shape [Nnodes, 3, 12].

    Plan fields: phase_intervals [stance, flight, land], phase_durations_s,
    q/v [Nnodes,6], torque_per_side_nm [Nnodes-1,3]. Optional coordinate_order
    accepts either 'pitch' or the ADAM model's 'body_pitch' spelling.

    K[i] acts at node i for its outgoing interval, including at phase changes.
    Each interval uses the nominal midpoint A_eff=A-B*LOCAL_FEEDBACK and ZOH;
    K is the OUTER gain, so the parent must not add LOCAL_FEEDBACK to it. Stage
    costs use Q*dt and R*dt (left-rectangle cost quadrature). The terminal cost
    is 0.5 seconds of the grounded Q. No full-state algebraic Riccati solve is
    attempted on the constrained, partly uncontrollable grounded system.
    K[-1] repeats the last landing gain for clamped node lookup; it is NOT a
    separately certified terminal hold controller. References are not mutated
    or certified here, and no affine correction for reference defects is added.
    """
    if not np.isfinite(pose_priority) or not 1. <= pose_priority <= 100.:
        raise ValueError('Pose priority must be finite and in [1,100]')
    counts, durations, q, v, torque = _plan_arrays(plan_dict)
    models = {True: build_linearization(dynamics_function, True),
              False: build_linearization(dynamics_function, False)}
    grounded = np.repeat([True, False, True], counts)
    steps = np.repeat(durations / counts, counts)
    costs = {mode: _cost_matrices(mode, pose_priority) for mode in (True, False)}
    gains = np.empty((len(q), 3, 12))
    P = .5 * costs[True][0]
    for i in range(len(torque) - 1, -1, -1):
        _, A, B = models[bool(grounded[i])]((q[i] + q[i+1]) / 2,
                                          (v[i] + v[i+1]) / 2, torque[i])
        A, B = np.asarray(A), np.asarray(B)
        if not np.isfinite(A).all() or not np.isfinite(B).all():
            raise ValueError(f'Non-finite dynamics linearization at interval {i}')
        Ad, Bd = _discretize(A - B @ LOCAL_FEEDBACK, B, steps[i])
        Q, R = costs[bool(grounded[i])]
        Qd, Rd = steps[i] * Q, steps[i] * R
        gains[i] = np.linalg.solve(Rd + Bd.T @ P @ Bd, Bd.T @ P @ Ad)
        closed_loop = Ad - Bd @ gains[i]
        # Equivalent Riccati update in a sum-of-PSD-terms form.
        P = Qd + gains[i].T @ Rd @ gains[i] + closed_loop.T @ P @ closed_loop
        P = (P + P.T) / 2
        if not np.isfinite(gains[i]).all() or not np.isfinite(P).all():
            raise ValueError(f'Non-finite Riccati recursion at interval {i}')
    gains[-1] = gains[-2]
    return gains
