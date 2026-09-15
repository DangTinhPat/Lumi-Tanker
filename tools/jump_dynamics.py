"""ADAM symbolic dynamics projected onto TAP's symmetric sagittal manifold.

This is an offline rigid-body planning model, not a simulator or evidence of a
feasible jump. Import an expanded, free-root CAD URDF; no model data is changed.
The projected equations do not establish feasibility of suppressed 3D motion.
"""

import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


JOINT_NAMES = tuple(
    f"cad_{side}_{joint}"
    for side in ("left", "right") for joint in ("hip", "knee", "wheel")
)
WHEEL_RADIUS = 0.030
GRAVITY = 9.81
Q_NAMES = ("body_x", "body_z", "body_pitch", "hip", "knee", "wheel")

# Mixed twist: world linear velocity of base origin, world angular velocity,
# then LH, LK, LW, RH, RK, RW. Both legs have the same native +Y angles.
S = np.zeros((12, 6))
S[0, 0] = S[2, 1] = S[4, 2] = 1.0
S[6:9, 3:6] = np.eye(3)
S[9:12, 3:6] = np.eye(3)
S.setflags(write=False)


def _read_model(urdf_path):
    path = Path(urdf_path)
    source = path.read_bytes()
    root = ET.fromstring(source)
    links = {link.get("name"): link for link in root.findall("link")}
    expected_links = {"cad_body"} | {
        f"cad_{side}_{part}" for side in ("left", "right")
        for part in ("thigh", "shin", "wheel_link")
    }
    joints = {joint.get("name"): joint for joint in root.findall("joint")}
    if (root.tag != "robot" or set(links) != expected_links
            or len(root.findall("link")) != 7
            or set(joints) != set(JOINT_NAMES) or len(root.findall("joint")) != 6):
        raise ValueError("Expected expanded seven-link CAD URDF, free-root cad_body, six joints")
    for side in ("left", "right"):
        chain = ("body", f"{side}_thigh", f"{side}_shin", f"{side}_wheel_link")
        for i, kind in enumerate(("hip", "knee", "wheel")):
            joint = joints[f"cad_{side}_{kind}"]
            if (joint.find("parent").get("link") != "cad_" + chain[i]
                    or joint.find("child").get("link") != "cad_" + chain[i + 1]
                    or joint.get("type") != ("continuous" if kind == "wheel" else "revolute")):
                raise ValueError("Unexpected CAD joint topology or type")
            axis = joint.find("axis")
            if axis is None or axis.get("xyz", "").split() == []:
                raise ValueError("Explicit native +Y joint axis required")
            if not np.array_equal(np.fromstring(axis.get("xyz"), sep=" "), [0., 1., 0.]):
                raise ValueError("Native +Y joint axes required on BOTH sides")
            origin = joint.find("origin")
            rpy = np.fromstring(origin.get("rpy", "0 0 0"), sep=" ") if origin is not None else np.zeros(3)
            if not np.array_equal(rpy, np.zeros(3)):
                raise ValueError("Sagittal selector requires unrotated CAD joint origins")
    masses = {}
    for name, link in links.items():
        inertial = link.find("inertial")
        if inertial is None or inertial.find("mass") is None or inertial.find("inertia") is None:
            raise ValueError(f"Missing inertial data for {name}")
        mass = float(inertial.find("mass").get("value"))
        if not np.isfinite(mass) or mass <= 0:
            raise ValueError(f"Invalid mass for {name}")
        masses[name] = mass
    metadata = {
        "total_mass_kg": sum(masses.values()),
        "link_masses_kg": masses,
        "urdf_path": str(path.resolve()),
        "urdf_sha256": hashlib.sha256(source).hexdigest(),
        "joint_names": JOINT_NAMES,
        "q_names": Q_NAMES,
        "wheel_radius_m": WHEEL_RADIUS,
        "gravity_m_s2": GRAVITY,
        "mass_basis": "URDF values unchanged; TAP inertials and motor allocation are provisional",
    }
    return source.decode("utf-8"), metadata


def model_metadata(urdf_path):
    """Return mass totals, per-link masses and source hash without loading ADAM."""
    return _read_model(urdf_path)[1]


def build_dynamics(urdf_path):
    """Return CasADi Function(q, v) -> M, bias, com, com_velocity, axle, Jcontact.

    Inputs are 6x1 in Q_NAMES order. Outputs are respectively 6x6, 6x1, 3x1,
    3x1, 3x1 and 2x6. Positions/linear velocities are world-expressed (X,Z
    sagittal); axle is the midpoint of the two wheel-link origins. Contact rows
    are the instantaneous wheel-material-point X,Z velocities on level ground.

    M*a + bias is the reduced inertial/gravity generalized force, excluding
    active torque, passive damping and contact. A symmetric per-side motor
    torque contributes TWICE its value to the hip/knee/wheel reduced equation.
    S is constant in mixed representation, so there is no M*Sdot*v term.
    ADAM's reduced RNEA supplies bias; no rigid-body algorithm is reimplemented.
    """
    import casadi as cs
    from adam import Representations
    from adam.casadi import KinDynComputations

    xml, metadata = _read_model(urdf_path)
    kindyn = KinDynComputations(
        xml, list(JOINT_NAMES), gravity=np.array([0., 0., -GRAVITY, 0., 0., 0.])
    )
    kindyn.set_frame_velocity_representation(Representations.MIXED_REPRESENTATION)
    if kindyn.NDoF != 6 or not np.isclose(
            float(kindyn.get_total_mass()), metadata["total_mass_kg"], rtol=0, atol=1e-10):
        raise ValueError("ADAM joint count or mass differs from the source URDF")
    q = cs.SX.sym("q", 6)
    v = cs.SX.sym("v", 6)
    c, s = cs.cos(q[2]), cs.sin(q[2])
    H = cs.vertcat(
        cs.horzcat(c, 0, s, q[0]), cs.horzcat(0, 1, 0, 0),
        cs.horzcat(-s, 0, c, q[1]), cs.horzcat(0, 0, 0, 1),
    )
    positions = cs.vertcat(q[3:6], q[3:6])
    selector = cs.DM(S)
    nu = selector @ v
    M = selector.T @ kindyn.mass_matrix_fun()(H, positions) @ selector
    bias = selector.T @ kindyn.bias_force_fun()(H, positions, nu[:6], nu[6:])
    com = kindyn.CoM_position_fun()(H, positions)
    com_velocity = cs.jacobian(com, q) @ v
    axles = [kindyn.forward_kinematics_fun(f"cad_{side}_wheel_link")(H, positions)[:3, 3]
             for side in ("left", "right")]
    axle = (axles[0] + axles[1]) / 2
    # The contact point changes with rolling; it is not a fixed URDF link frame.
    Jx = cs.jacobian(axle[0], q) - WHEEL_RADIUS * cs.DM([[0, 0, 1, 1, 1, 1]])
    Jz = cs.jacobian(axle[2], q)
    return cs.Function(
        "tap_jump_dynamics", [q, v],
        [M, bias, com, com_velocity, axle, cs.vertcat(Jx, Jz)],
        ["q", "v"], ["M", "bias", "com", "com_velocity", "axle", "Jcontact"],
    )
