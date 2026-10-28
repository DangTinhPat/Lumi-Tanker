#!/usr/bin/env python3
"""Derive gim_dynamic_shell.xml: gim_dynamic.xml plus ground contact for the body shell.

gim_dynamic.xml only collides the two wheels, so the stand-up simulation lets the
body pass through the floor and cannot reproduce the shell/knee touch-down that the
robot logs show. This adds the six STEP-derived body boxes of GeneratedBodyEnvelope.hpp
(the same ones robot_control::bodyClearance uses) as ground-contact geoms. They are
conservative AABBs, not calibrated contact: a simulator aid, never a controller input.
"""
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / "simulation/model/gim_dynamic.xml"
OUTPUT = ROOT / "simulation/model/gim_dynamic_shell.xml"
ENVELOPE = ROOT / "controller/core/include/robot_control/GeneratedBodyEnvelope.hpp"


def main():
    boxes = re.findall(r"\{\{(-?[\d.]+)F, (-?[\d.]+)F, (-?[\d.]+)F\}, "
                       r"\{(-?[\d.]+)F, (-?[\d.]+)F, (-?[\d.]+)F\}\},\s*//\s*(\S+)",
                       ENVELOPE.read_text(encoding="utf-8"))
    if len(boxes) != 6:
        raise RuntimeError(f"expected 6 envelope boxes, found {len(boxes)}")
    geoms, pairs = [], []
    for i, b in enumerate(boxes):
        lo, hi, label = [float(v) for v in b[:3]], [float(v) for v in b[3:6]], b[6]
        centre = [(l + h) / 2 for l, h in zip(lo, hi)]
        half = [(h - l) / 2 for l, h in zip(lo, hi)]
        name = f"shell_{i}_{label}"
        geoms.append(f'      <geom name="{name}" type="box" pos="{centre[0]:.6f} {centre[1]:.6f} {centre[2]:.6f}" '
                     f'size="{half[0]:.6f} {half[1]:.6f} {half[2]:.6f}" contype="0" conaffinity="0" rgba="0.9 0.3 0.1 0.15"/>')
        pairs.append(f'    <pair geom1="ground" geom2="{name}" condim="3" friction="0.6 0.6 0 0 0" '
                     'solref="0.006 1" solimp="0.95 0.99 0.001"/>')
    xml = MODEL.read_text(encoding="utf-8")
    anchor = '      <site name="imu_site"'
    if xml.count(anchor) != 1 or xml.count("  </contact>") != 1:
        raise RuntimeError("gim_dynamic.xml layout changed")
    xml = xml.replace(anchor, "\n".join(geoms) + "\n" + anchor)
    xml = xml.replace("  </contact>", "\n".join(pairs) + "\n  </contact>")
    # Visual CAD meshes stop colliding: gim_dynamic.xml lets knee/link meshes (cad_11/12/42/43) rest on the
    # floor, which holds the lying body at pitch .27 whereas the robot rests at .457 on the shell envelope.
    # Floor contact is then exactly what the controller's clearance model uses: wheels + shell boxes.
    default = '<geom friction="1.0 0.01 0.001" solref="0.006 1" solimp="0.95 0.99 0.001"/>'
    if xml.count(default) != 1:
        raise RuntimeError("gim_dynamic.xml default geom changed")
    xml = xml.replace(default, default.replace("<geom ", '<geom contype="0" conaffinity="0" '))
    xml = xml.replace('<mujoco model="gim6010_v3_dynamic">', '<mujoco model="gim6010_v3_shell_contact">', 1)
    OUTPUT.write_text(xml, encoding="utf-8")
    print(f"wrote {OUTPUT} with {len(boxes)} shell contact boxes")


if __name__ == "__main__":
    main()
