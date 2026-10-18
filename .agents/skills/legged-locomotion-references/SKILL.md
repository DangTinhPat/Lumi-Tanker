---
name: legged-locomotion-references
description: Reach for established balance/landing/fall-recovery theory and real open-source stacks (humanoid, quadruped, wheel-legged) before inventing a mechanism from scratch for TAP.
---

# Legged/Wheel-Legged Balance Control References

TAP is a wheel-legged hybrid (linear-inverted-pendulum wheel balance + 2 legs),
sitting between humanoid/biped and quadruped balance theory. Before designing
a new balance, landing, or fall-recovery mechanism, check whether an
established method already solves it - cite the real source in code comments
(as `LandingReflex.hpp`/`LandingImpedance.hpp` already do for the two sources
below), not a freehand derivation presented as novel.

## Core theory (small, load-bearing set - learn these first)

- **Capture Point / Instantaneous Capture Point (ICP)**: Pratt, Carff, Raibert,
  Neuhaus, "Capture Point: A Step Toward Humanoid Push Recovery" (IEEE
  Humanoids 2006). `x_capture = x_com + v_com/omega`, `omega = sqrt(g/z_com)`
  - the same omega TAP's `MpcBalancer` already uses for its linear-inverted-
  -pendulum model, so this applies almost verbatim (see `LandingReflex.hpp`'s
  `capture_point_gravity_m_s2`, and its 2026-09-16 regression: a capture-point
  term must be latched at liftoff, never re-derived live off noisy
  post-touchdown state - a lesson from this exact codebase, not theory alone).
- **ZMP (Zero Moment Point)**: classic biped stability criterion, largely
  superseded by capturability theory for fast/dynamic motion (ZMP needs an
  explicit support polygon and struggles with flight phases) - know it, but
  prefer capture-point/MPC framing for anything TAP-like.
- **Partial feedback linearization** for underactuated cart-pole/wheeled-
  inverted-pendulum systems: correcting a small-angle-linearized MPC's output
  against the real `sin`/`cos` dynamics rather than rebuilding the whole
  horizon model - see `MpcBalancer::Config::nonlinear_gravity_correction_gain`
  for TAP's own implementation of this technique.
- **MIT Mini Cheetah landing control**: arXiv 2110.02799, "Real-time Optimal
  Landing Control of the MIT Mini Cheetah" - active pre-touchdown leg
  extension to zero the closing velocity, not passive post-impact absorption
  alone. Already the basis for `LandingReflex::Config::
  active_extension_gravity_m_s2`.

## Real open-source stacks worth reading before reimplementing

- **OCS2** (`leggedrobotics/ocs2`) - optimal control for switched systems,
  the reference MPC framework behind ANYmal/legged-robot whole-body MPC work.
- **Pinocchio** + **Crocoddyl** - rigid-body dynamics and multi-contact
  optimal control libraries underlying most modern legged-robot research code.
- **CHAMP** (`chvmp/champ`) - open MIT-Cheetah-style hierarchical quadruped
  controller with ready URDFs (ANYmal, Mini Cheetah, Spot) - good for
  comparing joint/gait conventions.
- **A1-QP-MPC-Controller** (`ShuoYangRobotics/A1-QP-MPC-Controller`) - open
  reimplementation of the MIT Cheetah 3 QP+MPC stance controller.
- **ETH RSL wheeled-legged whole-body MPC** (Bellicoso/Hutter, "Whole-Body
  MPC and Online Gait Sequence Generation for Wheeled-Legged Robots") - the
  closest published architecture to TAP's own wheel+leg hybrid.
- Curated jump-off points: `lgkimjy/awesome-legged-locomotion`,
  `XinLang2019/awesome-wheeled-legged` (wheel-legged specific - check this
  one first for TAP), `aCodeDog/awesome-loco-manipulation`.

## How to apply

1. Before building a new mechanism, search for the specific failure mode
   (e.g. "capture point", "fall recovery quadruped", "wheeled-legged whole
   body MPC") rather than deriving from first principles alone - this
   session's biggest wins (capture point, MIT Cheetah landing extension,
   partial feedback linearization) all came from grounding in a named,
   citable method first.
2. Port the *idea*, not code verbatim - TAP's dynamics, joint conventions,
   and actuator limits (GIM6010-8, see `masses.yaml`) differ from any of the
   above robots; always re-derive constants from TAP's own CAD/MJCF model.
3. Always validate a ported method against Rule 3's canonical landing test
   (`docs/RULES.md`) with real trials before trusting it - a technique being
   "established" for a biped/quadruped is not proof it transfers cleanly to
   TAP's wheel-legged geometry (see `capture_point_hip`'s 2026-09-16
   regression: theoretically sound, still measured net-harmful until its
   liftoff-latch bug was found and fixed, and even then stayed net-harmful -
   an open, unresolved case, not a success story to copy blindly).
