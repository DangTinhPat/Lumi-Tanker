# Offline Jump Optimizer: Dynamics Library Handoff

Reviewed 2026-09-12. This extends [JUMP_RESEARCH.md](JUMP_RESEARCH.md), not a
repeat of its literature survey. Read the subsequent
[landing implementation audit](JUMP_LANDING_IMPLEMENTATION.md) for existing
failed trials and remaining work. No new jump, landing, height, optimization
feasibility, Gazebo or hardware result is claimed here.

Initial scope was research only. The parent subsequently requested the disjoint
`tools/jump_dynamics.py` and `tests/test_jump_dynamics.py` implementation and
created the offline environment itself. This sidecar installed nothing, ran no
simulations, changed no controller/model/actuator files, and wrote only those
two files and this note. Parent owns the constrained optimizer.

## Recommendation And Local Availability

Use **ADAM robotics 0.5.0 + CasADi 3.8.0**, consuming the expanded TAP URDF.
ADAM implements established Featherstone rigid-body algorithms and supplies
symbolic expressions; do not implement another RNEA, CRBA or massless-leg
template. CasADi supplies derivatives and the NLP interface, not robot dynamics.
IPOPT is a suitable offline local constrained solver, not a global feasibility
certificate. MuJoCo is not required by this URDF/CasADi path.

Initial read-only toolchain probes found:

| Component | Observation before parent's environment setup |
| --- | --- |
| Active Python | `/home/dvt/micromamba/envs/wheel-leg-ros/bin/python3`, 3.12.14, Linux x86_64 |
| System Python | `/usr/bin/python3`, 3.12.3 |
| NumPy / SciPy | 2.5.3 / 1.18.1 distribution metadata; SciPy from user site |
| ADAM / CasADi / Pinocchio | Not discoverable in either checked interpreter |
| Crocoddyl / mim_solvers / pydrake / dartpy / cyipopt / acados_template / SymPy | Not discoverable in initial active interpreter |
| C++ numerical tools | `g++`, `cmake`, `pkg-config`, `ipopt` executable discoverable |
| Native package metadata | `pkg-config --modversion dart ipopt eigen3`: 6.19.4 / 3.14.19 / 5.0.1 |
| DART URDF development component | `dart-utils-urdf` pkg-config lookup failed; not proof all DART URDF support is absent |
| Other tools | Xacro importable; Julia/conda/uv executables not found; micromamba environment exists |

Probes used `importlib.util.find_spec`, `importlib.metadata`, `shutil.which`,
`pkg-config`, source reads and `curl -L --fail --max-time 25`. These do not prove
compiler linking or solver operation. No C++ build or native optimizer was run.
The workspace has no Git repository; provenance below uses file hashes.

After parent's setup, this sidecar successfully imported ADAM 0.5.0, CasADi
3.8.0 and NumPy 2.5.3 through
`/home/dvt/.local/share/tap-jump-optimizer/bin/python`. Keep this environment
separate from ROS. The installed dependency set and hash lock are parent's
responsibility; do not modify the ROS environment or add the `test`, `all`,
PyTorch, JAX or MuJoCo extras to obtain this backend. The CasADi extra includes
`liecasadi`; importing `adam.casadi` also imports its IK module.

### Alternatives

| Library | Source-backed fit | Decision |
| --- | --- | --- |
| ADAM CasADi | URDF, floating base, CRBA, reduced RNEA, COM, FK, Jacobians; SX graphs callable with MX | Selected; import and TAP adapter tests now run locally |
| Pinocchio + CasADi | Official example imports `pinocchio.casadi`, casts `cpin.Model`, uses `aba` and manifold `integrate` | Strong alternative; Python module absent initially; verify CasADi bindings in the exact distribution before promising availability |
| Crocoddyl | Pinocchio-based contact-sequence optimal control, DDP/FDDP/BoxFDDP and other solvers | Established OCP library, but not installed; custom wheel rolling, impact and speed-dependent motor constraints still needed |
| Native DART + IPOPT | Native package metadata present, rigid-body dynamics and NLP building blocks | Numerical fallback to investigate, not an available symbolic AD solution; no validated linking/URDF/derivative path here |
| CasADi alone / SciPy alone | Optimization and numerical tools | Neither replaces a validated multibody model; do not hand-roll dynamics merely because SciPy is present |

## Verified ADAM API And Provenance

Primary source: https://github.com/gbionics/adam (the `ami-iit/adam` API URL
redirected there). Upstream `main` resolved to
`dce1099e6954539ba0abbe2be9715e56ebb501e7`; inspected the release `v0.5.0`
separately, because its API differs. Source inspection is not an upstream test
run or proof for TAP.

- [Release computations](https://github.com/gbionics/adam/blob/v0.5.0/src/adam/casadi/computations.py):
  `from adam.casadi import KinDynComputations`;
  constructor accepts URDF path/XML and explicit joint-name order, plus a
  six-element gravity vector. Release 0.5.0 **ignores `root_link`** and uses the
  URDF root. Current main supports re-rooting, so do not mix its examples into
  a release-based implementation. TAP input must already have root `cad_body`.
- `set_frame_velocity_representation(Representations.MIXED_REPRESENTATION)`:
  base twist order is world linear velocity of the base origin, then world
  angular velocity. It is not a world-origin spatial twist and not a
  body-expressed angular velocity. Set representation before building functions.
- `mass_matrix_fun()(H, s)` returns the `(6+n)x(6+n)` CRBA mass matrix.
  `bias_force_fun()(H, s, base_velocity, joint_velocity)` returns reduced-RNEA
  Coriolis/centrifugal/gravity terms with zero acceleration and no external
  forces. General inverse-dynamics residual is `M*a + bias - applied_forces`;
  do not mistake this bias API for a full acceleration/force-input RNEA API.
- `CoM_position_fun`, `CoM_jacobian_fun`, `forward_kinematics_fun(frame)`,
  `jacobian_fun(frame)`, `jacobian_dot_fun(frame)` and
  `centroidal_momentum_matrix_fun` are present. Prefer `*_fun` wrappers when
  the optimizer uses MX. `get_total_mass()` exposes the imported total.
- `aba_fun()` is available, but its symbolic wrapper does not accept external
  wrenches. Using explicit `M*a+bias=...` is simpler for contact-force decision
  variables and impact equations.
- [URDF joint handling](https://github.com/gbionics/adam/blob/v0.5.0/src/adam/model/std_factories/std_joint.py)
  supports revolute and continuous joints and preserves axis/origin data.
  [Link handling](https://github.com/gbionics/adam/blob/v0.5.0/src/adam/model/std_factories/std_link.py)
  retains mass, all six inertia entries and inertial-origin pose.
  [Factory](https://github.com/gbionics/adam/blob/v0.5.0/src/adam/model/factory.py)
  detects URDF without importing MuJoCo; optional MuJoCo type annotations are
  guarded by `TYPE_CHECKING` in its factory module.
- [Release tests](https://github.com/gbionics/adam/blob/v0.5.0/tests/test_casadi.py)
  compare mass, COM, FK, Jacobians and bias with iDynTree-derived quantities:
  mass/FK comparisons use absolute `1e-5`, bias comparisons `1e-4`.
  This is concrete independent-library test provenance, not just a README claim.
- [PyPI metadata](https://pypi.org/pypi/adam-robotics/json) reported release 0.5.0,
  Python `>=3.10`, `casadi>=3.6`, NumPy, SciPy, prettytable, urdfdom-py and
  array-api-compat. The published `adam_robotics-0.5.0-py3-none-any.whl` SHA256:
  `d8ae9a73b6ec2e625d2eeb0efde76439868457049b09da7cf7bfd08f63b04d5c`.
  This is registry provenance, not verification of the parent's downloaded wheel.
- ADAM declares BSD-3-Clause. Official
  [Pinocchio CasADi example](https://github.com/stack-of-tasks/pinocchio/blob/devel/examples/casadi/cartpole.py),
  [Crocoddyl README](https://github.com/loco-3d/crocoddyl/blob/master/README.md),
  and [CasADi documentation](https://web.casadi.org/docs/) were also inspected.
  CasADi documents bundled IPOPT interfaces and automatic differentiation;
  no NLP solve or performance benchmark was performed by this sidecar.

## Implemented Dynamics Boundary

`build_dynamics(urdf_path)` returns a CasADi Function, not a tuple or class.
`model_metadata(urdf_path)` separately returns `total_mass_kg`, per-link
`link_masses_kg`, source path/SHA256, joint/q order, wheel radius and gravity.

```python
from tools.jump_dynamics import build_dynamics, model_metadata

path = "log/jump_1789218763317496560/expanded.urdf"
f = build_dynamics(path)
metadata = model_metadata(path)
M, bias, com, com_velocity, axle, Jcontact = f(q, v)
# Named calls: f(q=q, v=v) -> dict with these same six output names.
```

Inputs `q,v` are 6x1, ordered `[body_x, body_z, body_pitch, hip, knee, wheel]`.
Outputs are 6x6, 6x1, 3x1, 3x1, 3x1, 2x6 respectively. All positions are world
expressed; axle is the midpoint of left/right wheel-link origins, not the ground
contact point. Continuous wheel angles are unwrapped scalar joint coordinates.

Construct `H = [Ry(pitch), (x,0,z); 0,0,0,1]`. ADAM joints are explicitly
`LH,LK,LW,RH,RK,RW`, with `s=[hip,knee,wheel,hip,knee,wheel]`.
Constant mixed selector `S` is 12x6 and maps:

```text
nu = S*v = [vx,0,vz, 0,pitch_rate,0,
            hip_rate,knee_rate,wheel_rate, hip_rate,knee_rate,wheel_rate]
M6 = S.T * M12 * S
b6 = S.T * bias12
```

No `Sdot` term exists for this selector. This is an exact projection of the
URDF rigid-body equations onto the prescribed manifold, subject to floating
point evaluation. It is **not** exact unconstrained 3D dynamics: roll, lateral
translation, yaw and asymmetric shape are suppressed. Source COM offsets and
products of inertia are retained, so the CAD model need not have an invariant
symmetry manifold. Discarded equation residuals can represent fictitious
constraint reactions, particularly in flight. A later full-3D residual check
or optimizer is required before declaring physical feasibility.

CAD zero, native +Y on both sides, all seven masses and all inertia entries
remain explicit model inputs. Body visual/inertial pitch `-pi/2` is consumed
from URDF; there is no second rotation or parallel-axis shift in the adapter.
The active native model uses a `0.040 m` wheel radius, `0.309 m` track and
`4.621 kg` total mass. The archived URDF/ADAM results below were not regenerated
for that model and remain historical only. Motor mass allocation and
uniform-density inertia estimates remain unmeasured assumptions.
Root/mount, topology, joint types and native axes are validated on load.

The level-ground material-point contact velocity uses precisely:

```text
Jx = d(axle_x)/dq - 0.040 * [0,0,1,1,1,1]
Jz = d(axle_z)/dq
Jcontact = [Jx; Jz]
gap = axle_z - 0.040
```

The wheel's absolute angular rate is pitch+hip+knee+relative-wheel rate.
Positive wheel spin therefore requires positive X rolling speed on both sides.
A rotating material contact point is not a fixed URDF frame, and its velocity
is not the derivative of the wheel-center-minus-radius position in X.

## Minimal Optimizer Design For Parent

1. Freeze input hashes, launch cutoff, travel limits, actuator law, damping and
   gravity before solving. Use states `q,v`, accelerations, actuator commands,
   contact forces and bounded phase durations. Start with known phases: grounded
   thrust, flight, impact reset, grounded compression, recovery. Use sparse
   direct collocation or multiple shooting; refine near switching events.
2. At smooth nodes impose `M6*a+b6 = Qmotor+Qpassive+Jcontact.T*f`.
   If `tau` is the THREE **per-side** symmetric motor torques,
   `Qmotor=[0,0,0,2*tau_h,2*tau_k,2*tau_w]`.
   Define `f=[Fx,Fz]` as the **total of both wheel forces**, so do NOT multiply
   its contribution by two again. Passive generalized damping is
   `[0,0,0,-2*.02*v_h,-2*.02*v_k,-2*.001*v_w]` under the current jump model.
   The dynamics Function deliberately includes neither motor nor passive terms.
3. Grounded rolling: enforce `gap=0`, `Jcontact*v=0`, normal force nonnegative
   and `abs(Fx)<=mu*Fz`. Use acceleration-level contact consistency where the
   transcription needs it. Flight: `f=0`, gap nonnegative, no no-slip constraint.
   Liftoff has continuous position/velocity and unloading; impact is distinct.
   These two-row constraints cover simultaneous symmetric level-ground contact,
   not independent left/right contact or roll. Friction is an assumption, not
   calibrated by this research. Body/leg primitive clearance needs separate
   constraints; positive wheel gap alone does not prevent body-ground contact.
4. Keep pre-impact/post-impact velocities distinct. A first rigid, plastic,
   sticking reset is `M(q)*(v_plus-v_minus)=J.T*p`, `J*v_plus=0`,
   `p_z>=0`, `abs(p_x)<=mu*p_z`, with continuous `q` and closing pre-impact
   normal velocity. Here `p` is total impulse in N*s. Check kinetic-energy loss
   and reject inadmissible impulses; if sticking is infeasible use a physically
   specified sliding mode, not unrestricted tangential impulse. Motor torque
   is finite and has no instantaneous impulse. Subsequent compression must be
   solved with bounded torque and available stroke. A rigid impulse predicts
   no finite peak contact load; tire compliance/contact duration and structural
   limits are unknown, so low impulse is only a screening objective.
5. Enforce the actual `actuator_bounds.hpp` law, not just torque boxes. Legs
   stay +/-5 Nm, wheels +/-0.13 Nm. Leg positive-work availability is 5 Nm
   through 19.896753472735355 rad/s (190 rpm), tapering to zero at
   28.274333882308138 rad/s (270 rpm); the fixed configured `leg_speed_limit`
   imposes its additional earlier positive-work cutoff. Default is
   12.566370614359172 rad/s; an archived experimental configuration may differ.
   Never choose a higher cutoff to obtain feasibility. Braking remains bounded
   and externally driven overspeed is allowed. URDF 1000 rad/s is a numerical
   guard, NOT a physical speed envelope or an impact braking mechanism.
6. Wheels accept speed commands, not independent torque commands:
   `tau_w=clip(.02*(clip(command,-wmax,wmax)-relative_wheel_rate),-.13,.13)`,
   with the same positive-work cutoff rule at `wmax=41.88790204786391`.
   Optimize the command or enforce its exact realizability; a box-constrained
   wheel torque alone can be uncommandable near the speed limit. Leg active
   effort includes optional local D feedback BEFORE clipping/taper/cutoff;
   freeze D and reference bounds, and never count it again as passive damping.
   Treat piecewise branches explicitly or use a conservative inner envelope
   followed by exact-law verification; an optimistic smooth approximation is
   not evidence of feasibility. Avoid imposing a hard state-speed clamp during
   impact; any extra planning speed bound must be labeled conservative.
7. Require finite joint travel and current controller-profile limits, feasible
   touchdown shape/rate, compression stroke and recovery, not height alone.
   Penalize launch horizontal COM velocity, unwanted angular momentum, peak
   displacement and impact energy. Preserve the >0.10 m COM-rise, <=10 mm peak
   XY and <=2 mm residual objectives from prior research; the planar model
   cannot certify Y. Pin initial state and station anchor, constrain a terminal
   settled equilibrium, then require independent closed-loop recovery evidence.
8. Export a candidate only with residuals, constraint margins, exact motor-law
   recheck, source hashes and mesh-refinement comparison. Solver failure is
   inconclusive, not proof of physical impossibility; local convergence is not
   proof of physical success. Parent owns subsequent optimizer/test design.

## Additional Wheeled-Jump Sources

The Ascento, Chen and Dinev papers already read in `JUMP_RESEARCH.md` retain their
stated applicability limits. Their full texts were not re-read here. In
particular: linkage springs are not extra TAP actuator energy; W-SLIP is not
TAP multibody dynamics; a prismatic leg is not an articulated CAD leg.

- **New code provenance for the existing Chen paper:**
  [BinghengNUS/SUSTech_NeZha](https://github.com/BinghengNUS/SUSTech_NeZha),
  tree `eb4f1d04650a7b8507066699b7fd12ae7351b33f`. README explicitly identifies
  code for *Underactuated Motion Planning and Control for Jumping With
  Wheeled-Bipedal Robots*, RA-L 6(2), 747-754 (2021),
  DOI https://doi.org/10.1109/LRA.2020.3047787. It contains a README and
  `Vrep_simulation_opensource.zip`. The archive was not downloaded or executed;
  README/tree only were inspected. GitHub reports no license. Useful next source
  for stance/flight/terminal-rate implementation provenance, but no claim about
  its exact torque-speed or impulse implementation is justified yet.
- **Additional paper-associated repository:** Zhuang et al., *Height Control
  and Optimal Torque Planning for Jumping With Wheeled-Bipedal Robots*, ICARM
  2021, pp. 477-482, DOI https://doi.org/10.1109/ICARM52023.2021.9536196;
  [silvery107/wheeled-bipedal-jumping](https://github.com/silvery107/wheeled-bipedal-jumping),
  tree `b31b98e4887894c39d5b7d98344738d438cde222`. Inspected README,
  `advisor_config/min_function.py`, `robot_specifications.txt` and LICENSE.
  README describes a wheeled-bipedal jumping model and Bayesian torque planning
  for height/energy; dependencies are Python 3.7, Webots R2021a, Advisor 0.1.6.
  The optimization wrapper starts Webots and reads metrics: this is
  simulator-in-the-loop parameter search, not symbolic URDF collocation.
  Specifications differ substantially (5 kg body, 0.05 m wheel radius,
  0.22 m segments). The wrapper also maps `jump_d` from `args.jump_c`, so it
  should not be adopted uncritically. AGPL-3.0 license; no code copied.
  Paper full text and detailed motor/contact controller were not inspected,
  so finite speed-dependent torque enforcement and impact-law fidelity are
  unverified. Useful torque-profile initialization idea, not a TAP capability
  claim or the preferred solver architecture.

The student `rbonuomo/UR_HybridWheeledJumpingRobot` implementation is already
covered in prior research; do not relabel it official Dinev code or repeat its
predicted-next-state loop as independent validation. None of these external
robots establishes TAP's torque-speed, thermal, tire or structural limits.

## Verification And Handoff Evidence

Read all four relevant skill files: robot physics, control, ROS/Gazebo and QA.
Physics handoff: unchanged CAD signs/inertias, fixed manifold, provisional mass.
Control handoff: named Function, selector, force/torque multiplicity and motor
law above. QA handoff: offline-only checks and outstanding 3D/contact evidence.
These are functional expert responsibilities, not claims of spawned agents.

Executed in parent's venv:

```bash
/home/dvt/.local/share/tap-jump-optimizer/bin/python -B -m unittest discover -s tests -p test_jump_dynamics.py -v
```

**9 tests passed.** Coverage: interface/metadata, SPD and translational mass
over 24 sampled configurations, independent URDF FK/COM and finite-difference
COM velocity/contact Jacobian, gravity as potential gradient, kinetic energy
from all seven link inertias, +Y contact/carrier-rate signs, rejection of a
mirrored axis/preview root, velocity-bias energy identity, and MX derivative
evaluation. Test fixtures expand the current Xacro with simulation/jump flags;
no simulator is launched. Default environments without ADAM/CasADi skip this
optional suite; the passing evidence uses the explicit venv above.

Also built the Function on the parent-provided
`log/jump_1789218763317496560/expanded.urdf`, SHA256
`7720f72660e5bc2c8d1cc3717ac145502ad9f1ccc942270b1b36d94fd6f50af8`.
The URDF parser warns about collision `name` attributes and archived
`ros2_control` tags; inertial/dynamics checks pass. The adapter does not alter
the XML to hide those warnings. No source or model was rewritten.

Inspected local source SHA256 values:

| Path | SHA256 |
| --- | --- |
| `src/tap_robot_description/description/company_rb/robot.urdf.xacro` | `d8163f8108d81b16723496d5843b1406d33be69486044ffed3c614c781713a90` |
| `src/tap_robot_description/description/company_rb/masses.yaml` | `8438c8c303ca535eb0f3781606292ee6216cd96c78594995b72643463332c78e` |
| `src/tap_robot_description/description/company_rb/mesh_inertials.yaml` | `d9fd19ab552558df41c265a730bbd6f8209148d68aca96e96f964e33fdcfbdc3` |
| `src/legacy/ros2_gazebo_sim/include/tap_sim/actuator_bounds.hpp` | `fa82bc2e122e33bc21f7e091f08733ec4d91fba1f2795d014fd80048bf0e1612` |

No full upstream suite, iDynTree cross-library test on TAP, constrained NLP,
3D residual audit, Gazebo test, package install or hardware action was performed
by this sidecar. Full flight momentum/contact validation, mesh convergence,
impact robustness and stable recovery remain separate evidence obligations.
