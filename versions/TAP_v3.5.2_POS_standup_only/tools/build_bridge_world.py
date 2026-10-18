#!/usr/bin/env python3
"""Bridge world for the GIM6010-8 MuJoCo model (simulation only).

Writes next to `gim_dynamic.xml`:
  gim_bridge.xml   the same robot on a long bridge: flat run-up, a ramp up at --slope-deg, a flat deck, a ramp down
                   at the same angle, a flat run-out. Derived from gim_dynamic.xml by text edits, so the robot
                   definition stays in one place; do not edit by hand, rerun this script.

Layout (x forward, the robot spawns at x = 0 facing +x, y across, z up; the ground plane is z = 0):

  pad ........ run-up        -pad-back .. ramp_foot            flat, z = 0
  ramp up .... slope-deg     ramp_foot .. deck_start           rises deck-height
  deck ....... flat          deck_start .. deck_end            z = deck-height, deck-length long
  ramp down .. slope-deg     deck_end .. ramp_end              falls back to z = 0
  run-out .... flat          ramp_end .. ramp_end + exit       z = 0

The bridge is made of boxes (the ramps are tilted slabs, the deck a slab), because box-cylinder contact is what the
wheels already use well; the top surfaces meet exactly at the joints, so there is no step. The deck is --width wide
with NO side walls: a wheel that leaves the edge drops to the ground (z = 0), as on a real bridge. Rails and pillars
are visual only (contype 0), so they never touch the robot. The slabs get the same explicit wheel friction (0.8) as
the ground; nothing about the robot, its mass, gravity or friction is changed.

  python3 tools/build_bridge_world.py                        # 15 deg, deck 0.40 m high and 4 m long
  python3 tools/build_bridge_world.py --slope-deg 10         # a gentler version
  python3 tools/build_bridge_world.py --deck-height 0.6 --deck-length 6

Run the sim on it:  TAP_SIM_MODEL=src/tap_sim/model/gim_bridge.xml python3 tools/tap_native_gui.py

Physics to keep in mind (see also build_rough_terrain.py): the wheel torque cap is 0.13 N.m per wheel, so the two wheels
give at most 2 x 0.13 / 0.04 m = 6.5 N of ground force, and holding 3.02 kg on a slope of angle a needs
3.02 x 9.81 x sin(a). That is 7.7 N at 15 deg: the model as configured cannot hold, let alone climb, 15 deg (the limit
is about 12.7 deg). The script prints this check. The stand-up recovery peak (0.20 N.m) is a short transient only.
"""
from __future__ import annotations

import argparse
import math
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MODEL_DIR = ROOT / "src/tap_sim/model"
BASE_MODEL = MODEL_DIR / "gim_dynamic.xml"
OUT_MODEL = MODEL_DIR / "gim_bridge.xml"   # --out overrides (the model must stay next to its mesh assets)

MASS_KG = 3.02
WHEEL_RADIUS_M = 0.040
WHEEL_TORQUE_NM = 0.13        # regular per-wheel cap used by the runner
G = 9.81


def quat_about_y(angle_rad: float) -> str:
    """Quaternion (w x y z) for a rotation about +y. A positive angle tips +x downwards, so a ramp that rises
    towards +x uses a negative angle."""
    return f"{math.cos(angle_rad / 2):.9f} 0 {math.sin(angle_rad / 2):.9f} 0"


def slab(name: str, centre: tuple[float, float], half_len: float, half_width: float, half_thick: float,
         angle_rad: float, rgba: str, collide: bool = True) -> str:
    contact = 'contype="1" conaffinity="1"' if collide else 'contype="0" conaffinity="0"'
    return (f'    <geom name="{name}" type="box" pos="{centre[0]:.5f} 0 {centre[1]:.5f}" '
            f'quat="{quat_about_y(angle_rad)}" size="{half_len:.5f} {half_width:.5f} {half_thick:.5f}" '
            f'rgba="{rgba}" {contact}/>\n')


def build(args) -> dict:
    theta = math.radians(args.slope_deg)
    if not 0.0 < args.slope_deg < 45.0:
        raise SystemExit("--slope-deg must be between 0 and 45")
    h, t = args.deck_height, args.thickness
    run = h / math.tan(theta)                  # horizontal length of one ramp
    ramp_len = h / math.sin(theta)             # length along the slope
    x_foot = args.pad_front
    x_deck0 = x_foot + run
    x_deck1 = x_deck0 + args.deck_length
    x_end = x_deck1 + run
    hw = 0.5 * args.width

    base = BASE_MODEL.read_text()
    plane = re.search(r'    <geom name="ground" type="plane"[^\n]*/>\n', base)
    if plane is None:
        raise SystemExit("gim_dynamic.xml: ground plane line not found")

    c, s = math.cos(theta), math.sin(theta)
    geoms = ['    <geom name="ground" type="plane" pos="0 0 0" size="14 6 0.1" rgba="0.20 0.22 0.24 1"/>\n']
    # Ramp up: surface from (x_foot, 0) to (x_deck0, h); the slab centre sits half a thickness below the mid-surface,
    # along the surface normal (-sin, cos) going up and (+sin, cos) going down.
    mid_up = (0.5 * (x_foot + x_deck0), 0.5 * h)
    geoms.append(slab("bridge_ramp_up", (mid_up[0] + s * t / 2, mid_up[1] - c * t / 2), 0.5 * ramp_len, hw, t / 2,
                      -theta, "0.55 0.56 0.60 1"))
    geoms.append(slab("bridge_deck", (0.5 * (x_deck0 + x_deck1), h - t / 2), 0.5 * args.deck_length, hw, t / 2,
                      0.0, "0.62 0.63 0.67 1"))
    mid_dn = (0.5 * (x_deck1 + x_end), 0.5 * h)
    geoms.append(slab("bridge_ramp_down", (mid_dn[0] - s * t / 2, mid_dn[1] - c * t / 2), 0.5 * ramp_len, hw, t / 2,
                      +theta, "0.55 0.56 0.60 1"))
    # Visual only: low rails along both deck edges and pillars under it (contype 0, the robot never touches them).
    for side, y in (("left", hw - 0.02), ("right", -hw + 0.02)):
        geoms.append(f'    <geom name="rail_{side}" type="box" pos="{0.5 * (x_deck0 + x_deck1):.4f} {y:.4f} {h + 0.03:.4f}" '
                     f'size="{0.5 * args.deck_length:.4f} 0.01 0.03" rgba="0.90 0.55 0.10 1" contype="0" conaffinity="0"/>\n')
    n_pillars = max(2, int(args.deck_length // 1.5) + 1)
    for i in range(n_pillars):
        x = x_deck0 + (args.deck_length * i / (n_pillars - 1))
        geoms.append(f'    <geom name="pillar_{i}" type="box" pos="{x:.4f} 0 {(h - t) / 2:.4f}" '
                     f'size="0.05 {hw - 0.1:.4f} {(h - t) / 2:.4f}" rgba="0.38 0.39 0.42 1" contype="0" conaffinity="0"/>\n')
    text = base.replace(plane.group(0), "".join(geoms), 1)

    # The wheels touch every bridge slab with the same explicit friction as the ground.
    pairs = "".join(
        f'    <pair geom1="{g}" geom2="{w}_wheel_collision" condim="3" friction="0.8 0.8 0 0 0" '
        'solref="0.006 1" solimp="0.95 0.99 0.001"/>\n'
        for g in ("bridge_ramp_up", "bridge_deck", "bridge_ramp_down") for w in ("left", "right"))
    text = text.replace("  </contact>", pairs + "  </contact>", 1)
    text = text.replace('<mujoco model="gim6010_v3_dynamic">', '<mujoco model="gim6010_v3_bridge">', 1)
    text = text.replace("\n", f"\n<!-- GENERATED by tools/build_bridge_world.py from gim_dynamic.xml "
                        f"(slope {args.slope_deg} deg, deck {h} m high x {args.deck_length} m long, {args.width} m wide): "
                        "do not edit, rerun the script -->\n", 1)
    OUT_MODEL.write_text(text)
    return dict(x_foot=x_foot, x_deck0=x_deck0, x_deck1=x_deck1, x_end=x_end, run=run, ramp_len=ramp_len)


def verify(args, lay: dict) -> None:
    """Compile the model and cast vertical rays down the centre line (and both wheel tracks): the measured
    profile must match the design, with no step at any joint."""
    import mujoco
    import numpy as np

    model = mujoco.MjModel.from_xml_path(str(OUT_MODEL))
    data = mujoco.MjData(model)
    data.qpos[2] = 50.0            # lift the robot out of the way: the rays must see only the terrain
    mujoco.mj_forward(model, data)
    names = ["ground", "bridge_ramp_up", "bridge_deck", "bridge_ramp_down"]
    ids = {mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, n) for n in names}

    def height(x: float, y: float) -> float:
        geomid = np.zeros(1, dtype=np.int32)
        dist = mujoco.mj_ray(model, data, np.array([x, y, 3.0]), np.array([0.0, 0.0, -1.0]), None, 1, -1, geomid)
        if dist < 0 or int(geomid[0]) not in ids:
            raise SystemExit(f"ray at x={x:.2f} y={y:.2f} hit no terrain geom (geom {int(geomid[0])})")
        return 3.0 - dist

    xs = np.arange(-1.0, lay["x_end"] + args.exit, 0.02)
    track = 0.2654 / 2
    zs = {y: np.array([height(x, y) for x in xs]) for y in (0.0, track, -track)}
    z0 = zs[0.0]
    dz = np.diff(z0) / 0.02
    ramp_mask = (xs[:-1] > lay["x_foot"] + 0.05) & (xs[:-1] < lay["x_deck0"] - 0.05)
    down_mask = (xs[:-1] > lay["x_deck1"] + 0.05) & (xs[:-1] < lay["x_end"] - 0.05)
    slope_up = math.degrees(math.atan(dz[ramp_mask].mean()))
    slope_dn = math.degrees(math.atan(dz[down_mask].mean()))
    deck_top = z0[(xs > lay["x_deck0"] + 0.1) & (xs < lay["x_deck1"] - 0.1)]
    step = np.abs(np.diff(z0)).max()
    print(f"measured: ramp up {slope_up:+.2f} deg, ramp down {slope_dn:+.2f} deg, deck z "
          f"{deck_top.min():.4f}..{deck_top.max():.4f} m, largest 2 cm height change {step * 1000:.1f} mm "
          f"(a {args.slope_deg:g} deg slope changes {0.02 * math.tan(math.radians(args.slope_deg)) * 1000:.1f} mm), "
          f"wheel-track mismatch {max(np.abs(zs[track] - z0).max(), np.abs(zs[-track] - z0).max()) * 1000:.2f} mm")
    assert abs(slope_up - args.slope_deg) < 0.3 and abs(slope_dn + args.slope_deg) < 0.3, "slope does not match"
    assert abs(deck_top.min() - args.deck_height) < 2e-3 and abs(deck_top.max() - args.deck_height) < 2e-3
    assert step < 0.02 * math.tan(math.radians(args.slope_deg)) * 1.2 + 1e-4, "step at a joint"
    assert np.abs(zs[track] - z0).max() < 1e-3, "left/right wheel track heights differ"


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--slope-deg", type=float, default=15.0, help="ramp angle, up and down [deg]")
    p.add_argument("--deck-height", type=float, default=0.40, help="height of the deck above the ground [m]")
    p.add_argument("--deck-length", type=float, default=4.0, help="length of the flat deck [m]")
    p.add_argument("--width", type=float, default=0.9, help="width of the bridge (ramps and deck) [m]")
    p.add_argument("--thickness", type=float, default=0.10, help="slab thickness [m]")
    p.add_argument("--pad-front", type=float, default=1.5, help="flat run-up from the spawn (x = 0) to the ramp foot [m]")
    p.add_argument("--exit", type=float, default=2.0, help="flat run-out after the ramp down [m]")
    p.add_argument("--out", type=Path, default=None, help="output file name inside the model directory (default gim_bridge.xml)")
    p.add_argument("--no-verify", action="store_true", help="skip the compile + ray check")
    args = p.parse_args()
    global OUT_MODEL
    if args.out is not None:
        OUT_MODEL = MODEL_DIR / args.out.name

    lay = build(args)
    hold_n = MASS_KG * G * math.sin(math.radians(args.slope_deg))
    cap_n = 2 * WHEEL_TORQUE_NM / WHEEL_RADIUS_M
    limit_deg = math.degrees(math.asin(cap_n / (MASS_KG * G)))
    print(f"wrote {OUT_MODEL}")
    print(f"ramp foot x={lay['x_foot']:.3f}  deck x={lay['x_deck0']:.3f}..{lay['x_deck1']:.3f}  "
          f"ramp end x={lay['x_end']:.3f}  (each ramp {lay['run']:.3f} m long in x, {lay['ramp_len']:.3f} m on the slope)")
    verdict = "CANNOT hold it" if hold_n > cap_n else "can hold it (climbing needs more)"
    print(f"hold-on-slope force {hold_n:.2f} N vs wheel limit {cap_n:.2f} N ({WHEEL_TORQUE_NM} N.m x2): the robot {verdict}; "
          f"static limit {limit_deg:.1f} deg")
    if not args.no_verify:
        verify(args, lay)
    return 0


if __name__ == "__main__":
    sys.exit(main())
