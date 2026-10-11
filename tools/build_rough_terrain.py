#!/usr/bin/env python3
"""Rough "wave" terrain for the GIM6010-8 MuJoCo model (simulation only).

Writes next to `gim_dynamic.xml`:
  gim_rough.xml           the same robot on a height field (derived from gim_dynamic.xml by text edits, so the
                          robot definition stays in one place; do not edit by hand, rerun this script)
  gim_rough_terrain.bin   the height field (MuJoCo custom binary: int32 nrow, int32 ncol, float32 data)

Terrain: a flat start pad around the robot's spawn point (a plain box, not part of the height field: the belly
shell lying on hundreds of 1 cm height-field cells made the passive robot jitter), then smooth 2-D waves (several wavelengths and
directions, so the left and right wheel see different heights and the body rolls) with a little band-limited
roughness on top, then a flat run-out. No steps: the field is a sum of sinusoids and filtered noise, and the
slope of the final surface is measured on the triangles MuJoCo uses and again by casting rays into the
compiled model; the script refuses a surface steeper than 45 deg.

  python3 tools/build_rough_terrain.py                       # default: steepest point 17.7 deg
  python3 tools/build_rough_terrain.py --max-slope-deg 25    # a stress version
  python3 tools/build_rough_terrain.py --seed 3              # another random surface

The one knob is the steepest slope; the heights follow from it. Physics to keep in mind: the two wheels give at
most 2 x 0.13 N.m / 0.04 m = 6.5 N of ground force, and holding 3.1 kg on a slope of angle a needs
3.1 x 9.81 x sin(a), so the robot cannot even stand on more than ~12 deg (climbing needs more). Above that the
terrain is for jumps, hops or falls, not for driving.

Run the sim on it:  TAP_SIM_MODEL=src/tap_sim/model/gim_rough.xml python3 tools/tap_native_gui.py
Known limit: the runner's `wheel_clearance` telemetry is the wheel height above z = 0, not above the terrain.
"""
from __future__ import annotations

import argparse
import math
import re
import struct
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
MODEL_DIR = ROOT / "src/tap_sim/model"
BASE_MODEL = MODEL_DIR / "gim_dynamic.xml"
OUT_MODEL = MODEL_DIR / "gim_rough.xml"
OUT_BIN = MODEL_DIR / "gim_rough_terrain.bin"

TRACK_M = 0.2654          # wheel-centre spacing (V3Controller::Roll::track_m)
HARD_LIMIT_DEG = 45.0     # never produce a steeper surface
CLIMB_LIMIT_DEG = math.degrees(math.asin(2 * 0.13 / 0.04 / (3.12 * 9.81)))   # ~12.3 deg: the wheels cannot hold more

# Waves before the slope scaling: (wavelength m, amplitude m). The long swell makes the highs and lows, the
# mid waves tilt the body side to side (wavelength about 1.5 to 3 times the track), the roughness is texture.
SWELL = [(2.6, 0.012), (1.9, 0.014), (1.3, 0.016)]
MID = [(0.90, 0.007), (0.70, 0.007), (0.55, 0.007), (0.42, 0.006)]
ROUGH_RMS_M = 0.0012
ROUGH_WAVELENGTH_M = 0.12      # centre of the roughness band (an octave either side)


def quintic(t: np.ndarray) -> np.ndarray:
    t = np.clip(t, 0.0, 1.0)
    return t * t * t * (t * (6.0 * t - 15.0) + 10.0)


def raw_field(x, y, res, seed):
    rng = np.random.default_rng(seed)
    X, Y = np.meshgrid(x, y)                        # rows follow y, columns follow x
    waves = SWELL + MID
    # Directions spread over the half circle (jittered, shuffled): some waves run across the path, some along it.
    angles = (np.arange(len(waves)) + 0.5) * math.pi / len(waves) + rng.uniform(-0.15, 0.15, len(waves))
    rng.shuffle(angles)
    h = np.zeros_like(X)
    for (lam, amp), ang in zip(waves, angles):
        k = 2.0 * math.pi / lam
        h += amp * np.sin(k * (X * math.cos(ang) + Y * math.sin(ang)) + rng.uniform(0, 2 * math.pi))
    noise = rng.standard_normal(X.shape)
    spectrum = np.fft.rfft2(noise)
    fy = np.fft.fftfreq(X.shape[0], res)[:, None]
    fx = np.fft.rfftfreq(X.shape[1], res)[None, :]
    f = np.hypot(fx, fy)
    f0 = 1.0 / ROUGH_WAVELENGTH_M
    with np.errstate(divide="ignore"):
        window = np.where(f > 0, np.exp(-0.5 * (np.log(np.maximum(f, 1e-9) / f0) / 0.5) ** 2), 0.0)
    rough = np.fft.irfft2(spectrum * window, s=X.shape)
    rough *= ROUGH_RMS_M / rough.std()
    return X, Y, h + rough


def triangle_slopes_deg(h: np.ndarray, res: float) -> np.ndarray:
    """Slope of every triangle of the grid, for both ways of splitting a cell into two (the worst case counts)."""
    a, b, c, d = h[:-1, :-1], h[:-1, 1:], h[1:, :-1], h[1:, 1:]       # (y, x) corners: 00, 01, 10, 11
    def slope(gx, gy):
        return np.degrees(np.arctan(np.hypot(gx, gy)))
    s = [
        slope((b - a) / res, (c - a) / res),      # split along the b-c diagonal: triangle a b c
        slope((d - c) / res, (d - b) / res),      #                                 triangle d b c
        slope((b - a) / res, (d - b) / res),      # split along the a-d diagonal: triangle a b d
        slope((d - c) / res, (c - a) / res),      #                                 triangle a c d
    ]
    return np.fmax.reduce(s)


def build(args):
    res = args.res
    nx = int(round(args.length / res)) + 1
    ny = int(round(args.width / res)) + 1
    x = args.x0 + np.arange(nx) * res
    y = -0.5 * args.width + np.arange(ny) * res
    X, Y, h = raw_field(x, y, res, args.seed)
    half = 0.5 * args.width
    mask = (quintic((X - args.pad_end) / args.ramp) *                       # flat start pad, then ramp up
            quintic((args.x0 + args.length - X) / args.ramp) *              # ramp down into the run-out
            quintic((half - np.abs(Y)) / args.edge_ramp))                   # flat at the sides
    h = h * mask
    slope0 = triangle_slopes_deg(h, res).max()
    k = math.tan(math.radians(args.max_slope_deg)) / math.tan(math.radians(slope0))
    return x, y, h * k, k


def write_files(args, x, y, h):
    res = args.res
    hmin, hmax = float(h.min()), float(h.max())
    v = ((h - hmin) / (hmax - hmin)).astype(np.float32)                     # 0..1, as the format wants
    ny, nx = v.shape
    with open(OUT_BIN, "wb") as f:
        f.write(struct.pack("<ii", ny, nx))
        f.write(v.tobytes())
    sx, sy = 0.5 * nx_len(x, res), 0.5 * nx_len(y, res)
    cx = float(x[0]) + sx
    base = Path(BASE_MODEL).read_text()
    # 1) the height field and a checker material (the grid makes the relief visible in the viewer)
    assets = (
        f'    <hfield name="rough_terrain" file="{OUT_BIN.name}" size="{sx:.4f} {sy:.4f} {hmax - hmin:.6f} 0.1"/>\n'
        '    <texture name="terrain_grid" type="2d" builtin="checker" rgb1="0.30 0.32 0.35" rgb2="0.40 0.42 0.46" '
        'width="512" height="512"/>\n'
        f'    <material name="terrain_mat" texture="terrain_grid" texrepeat="{int(round(2 * sx / 0.25))} '
        f'{int(round(2 * sy / 0.25))}" texuniform="false" reflectance="0"/>\n'
        f'    <material name="terrain_pad_mat" texture="terrain_grid" texrepeat="{int(round((args.x0 + 0.1 + args.pad_back) / 0.25))} '
        f'{int(round(2 * sy / 0.25))}" texuniform="false" reflectance="0"/>\n')
    text = base.replace("  </asset>", assets + "  </asset>", 1)
    # 2) the old plane stays only as a safety floor far below the lowest valley (a plane would cut the valleys)
    plane = re.search(r'    <geom name="ground" type="plane"[^\n]*/>\n', text)
    if plane is None:
        raise SystemExit("gim_dynamic.xml: ground plane line not found")
    floor = f'    <geom name="ground" type="plane" pos="0 0 {hmin - 0.08:.4f}" size="30 30 0.1" rgba="0.12 0.13 0.15 1"/>\n'
    pad_x0, pad_x1 = -args.pad_back, args.x0 + 0.1          # overlaps the flat first 0.2 m of the height field
    terrain = (f'    <geom name="start_pad" type="box" pos="{0.5 * (pad_x0 + pad_x1):.4f} 0 -0.05" '
               f'size="{0.5 * (pad_x1 - pad_x0):.4f} {sy:.4f} 0.05" material="terrain_pad_mat" contype="1" conaffinity="1"/>\n'
               f'    <geom name="terrain" type="hfield" hfield="rough_terrain" pos="{cx:.4f} 0 {hmin:.6f}" '
               'material="terrain_mat" contype="1" conaffinity="1"/>\n')
    text = text.replace(plane.group(0), floor + terrain, 1)
    # 3) the wheels touch the terrain with the same explicit friction as the old ground
    pairs = "".join(
        f'    <pair geom1="{g}" geom2="{w}_wheel_collision" condim="3" friction="0.8 0.8 0 0 0" '
        'solref="0.006 1" solimp="0.95 0.99 0.001"/>\n' for g in ("start_pad", "terrain") for w in ("left", "right"))
    text = text.replace("  </contact>", pairs + "  </contact>", 1)
    text = text.replace('<mujoco model="gim6010_v3_dynamic">', '<mujoco model="gim6010_v3_rough">', 1)
    text = text.replace("\n", f"\n<!-- GENERATED by tools/build_rough_terrain.py from gim_dynamic.xml "
                        f"(seed {args.seed}, steepest {args.max_slope_deg} deg): do not edit, rerun the script -->\n", 1)
    OUT_MODEL.write_text(text)
    return hmin, hmax


def nx_len(axis, res):
    return float(axis[-1] - axis[0])


def check_with_mujoco(args, x, y, h):
    """Cast rays into the compiled model: the surface MuJoCo really has must match h and stay below the slope limit."""
    import mujoco
    m = mujoco.MjModel.from_xml_path(str(OUT_MODEL))
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    terrain = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "terrain")
    step = 2                                           # every 2 cm
    xs, ys = x[1:-1:step], y[1:-1:step]
    z = np.zeros((len(ys), len(xs)))
    geom = np.zeros(1, dtype=np.int32)
    for j, yy in enumerate(ys):
        for i, xx in enumerate(xs):
            dist = mujoco.mj_ray(m, d, np.array([xx, yy, 1.0]), np.array([0.0, 0.0, -1.0]), None, 1, -1, geom)
            # rays that hit the robot (it stands on the flat pad) are not the terrain: left out
            z[j, i] = 1.0 - dist if dist >= 0 and geom[0] == terrain else np.nan
    iy, ix = np.arange(1, len(y) - 1, step), np.arange(1, len(x) - 1, step)
    err = np.nanmax(np.abs(z - h[np.ix_(iy, ix)]))
    slope = np.nanmax(triangle_slopes_deg(z, args.res * step))
    return err, slope


def report(args, x, y, h, k, ray_err, ray_slope):
    res = args.res
    sl = triangle_slopes_deg(h, res)
    shift = int(round(TRACK_M / res))
    roll = np.abs(h[shift:, :] - h[:-shift, :])          # same x, one track apart in y
    lo = int(round((args.pad_end + args.ramp - args.x0) / res))
    rough_zone = slice(lo, len(x) - int(round(args.ramp / res)))
    print(f"terrain {args.length:.1f} x {args.width:.1f} m at {100 * res:.0f} cm, seed {args.seed}, scale k={k:.2f}")
    print(f"  height {1e3 * h.min():+.0f} .. {1e3 * h.max():+.0f} mm (peak to peak {1e3 * (h.max() - h.min()):.0f} mm), "
          f"rms {1e3 * h[:, rough_zone].std():.1f} mm")
    print(f"  slope: median {np.median(sl[:, rough_zone]):.1f}, 99% {np.percentile(sl[:, rough_zone], 99):.1f}, "
          f"max {sl.max():.1f} deg (limit {HARD_LIMIT_DEG:.0f}; the wheels hold ~{CLIMB_LIMIT_DEG:.0f})")
    print(f"  left/right height difference across the track: 95% {1e3 * np.percentile(roll[:, rough_zone], 95):.0f}, "
          f"max {1e3 * roll.max():.0f} mm")
    print(f"  MuJoCo ray check: height error {1e3 * ray_err:.2f} mm, steepest {ray_slope:.1f} deg")


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--max-slope-deg", type=float, default=17.7, help="steepest point of the surface (<= 45); 12 = the first version, 17.7 = 1.5 x its heights")
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--res", type=float, default=0.01, help="grid spacing [m]")
    p.add_argument("--length", type=float, default=9.5, help="size of the height field along x [m]")
    p.add_argument("--width", type=float, default=3.0, help="size along y [m]")
    p.add_argument("--x0", type=float, default=0.5, help="x of the height field's start [m] (the robot spawns at x=0 on the box pad that ends at x0+0.1)")
    p.add_argument("--pad-back", type=float, default=2.2, help="length of the flat box pad behind x=0 [m]")
    p.add_argument("--pad-end", type=float, default=0.7, help="x where the flat start pad ends and the ramp-up begins")
    p.add_argument("--ramp", type=float, default=1.0, help="length of the ramp from the pad / into the run-out [m]")
    p.add_argument("--edge-ramp", type=float, default=0.6, help="width of the flat strip fading in at each side [m]")
    args = p.parse_args()
    if not 0.0 < args.max_slope_deg <= HARD_LIMIT_DEG:
        raise SystemExit(f"--max-slope-deg must be in (0, {HARD_LIMIT_DEG:.0f}]")
    x, y, h, k = build(args)
    write_files(args, x, y, h)
    ray_err, ray_slope = check_with_mujoco(args, x, y, h)
    report(args, x, y, h, k, ray_err, ray_slope)
    if ray_err > 0.002 or ray_slope > HARD_LIMIT_DEG:
        sys.exit(f"MuJoCo surface does not match the design (error {1e3 * ray_err:.1f} mm, slope {ray_slope:.1f} deg)")
    print(f"wrote {OUT_MODEL.relative_to(ROOT)} and {OUT_BIN.relative_to(ROOT)} ({OUT_BIN.stat().st_size / 1e6:.1f} MB)")


if __name__ == "__main__":
    main()
