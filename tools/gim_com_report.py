#!/usr/bin/env python3
"""Estimate GIM COM from the source STEP and explicitly assigned group masses.

Authoring tool only: requires cadquery-ocp. Does not alter firmware or models.
STEP supplies shapes/placements, not densities. Uniform density is assumed
within each listed mass group. The body split follows the historical
body_module_inertia.py (5ae6de9); its residual is adjusted for --body-mass.
Distal links retain the adopted slender-link / motor-at-axle approximation.
"""

import argparse
import hashlib
import json
import math
from pathlib import Path

from OCP.BRepBndLib import BRepBndLib
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepGProp import BRepGProp
from OCP.Bnd import Bnd_Box
from OCP.GProp import GProp_GProps
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_Label
from OCP.TDocStd import TDocStd_Document
from OCP.TopLoc import TopLoc_Location
from OCP.XCAFApp import XCAFApp_Application
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.collections import Sequence_TDF_Label


ROOT = Path(__file__).resolve().parents[1]
BODY = {7, 8, 13, 14, 15, 17, 18, 19, 20, 21, 22, 24,
        31, 32, 35, 36, 37, 39, 40, 44, 45, 46, 47, *range(48, 60)}
MOTORS = {13, 14, 35, 36}
RESIDUAL = {48, 18, 19, 20, 21, 24, *range(51, 55), *range(56, 60)}


def name(label):
    attr = TDataStd_Name()
    return (attr.Get().ToExtString()
            if label.FindAttribute(TDataStd_Name.GetID_s(), attr) else "")


def read_parts(source, *, with_bounds=False):
    doc = TDocStd_Document(TCollection_ExtendedString("gim_com"))
    XCAFApp_Application.GetApplication_s().NewDocument(
        TCollection_ExtendedString("MDTV-XCAF"), doc)
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    if not reader.ReadFile(str(source)).name.endswith("RetDone") or not reader.Transfer(doc):
        raise RuntimeError(f"Cannot load STEP: {source}")
    tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    parts = []

    def collect(label, parent):
        if tool.IsReference_s(label):
            referred = TDF_Label()
            if not tool.GetReferredShape_s(label, referred):
                raise RuntimeError("Unresolved STEP reference")
            location = parent.Multiplied(tool.GetLocation_s(label))
            if tool.IsAssembly_s(referred):
                collect(referred, location)
            else:
                shape = BRepBuilderAPI_Transform(
                    tool.GetShape_s(referred), location.Transformation(), True).Shape()
                parts.append((name(referred), shape))
        elif tool.IsAssembly_s(label):
            children = Sequence_TDF_Label()
            tool.GetComponents_s(label, children, False)
            for i in range(1, children.Length() + 1):
                collect(children.Value(i), parent)
        else:
            raise RuntimeError("Expected an assembly with component references")

    roots = Sequence_TDF_Label()
    tool.GetFreeShapes(roots)
    for i in range(1, roots.Length() + 1):
        collect(roots.Value(i), TopLoc_Location())
    if len(parts) != 59:
        raise RuntimeError("STEP layout changed; review component mass assignments")
    expected = {13: "GIM6010-8", 14: "GIM6010-8", 35: "GIM6010-8",
                36: "GIM6010-8", 48: "PCB", 49: "14.4V Baterry",
                50: "Body 1", 55: "Body 2"}
    for number, expected_name in expected.items():
        if parts[number - 1][0] != expected_name:
            raise RuntimeError(f"Component {number} changed; review mass assignments")
    bounds = Bnd_Box()
    for _, shape in parts:
        BRepBndLib.Add_s(shape, bounds, True)
    origin = [(bounds.GetXMin() + bounds.GetXMax()) / 2,
              (bounds.GetYMin() + bounds.GetYMax()) / 2,
              (bounds.GetZMin() + bounds.GetZMax()) / 2]
    measured = {}
    for number, (label, shape) in enumerate(parts, 1):
        props = GProp_GProps()
        BRepGProp.VolumeProperties_s(shape, props)
        volume = props.Mass()
        if not math.isfinite(volume) or volume <= 0:
            raise RuntimeError(f"Invalid solid volume for component {number}")
        center = props.CentreOfMass()
        measured[number] = {"name": label, "volume_mm3": volume,
                            "centroid_m": [(center.Coord(i + 1) - origin[i]) / 1000
                                           for i in range(3)]}
        if with_bounds:
            box = Bnd_Box()
            BRepBndLib.AddOptimal_s(shape, box, False, False)
            measured[number]["bounds_m"] = [
                [(box.GetXMin() - origin[0]) / 1000,
                 (box.GetYMin() - origin[1]) / 1000,
                 (box.GetZMin() - origin[2]) / 1000],
                [(box.GetXMax() - origin[0]) / 1000,
                 (box.GetYMax() - origin[1]) / 1000,
                 (box.GetZMax() - origin[2]) / 1000]]
    return origin, measured


def body_com(parts, body_mass, battery, battery_mass=.25, frame_mass=.35):
    allocated = 1.6 + battery_mass + frame_mass
    groups = [("four_motors", MOTORS, 1.6), ("battery", {49}, battery_mass),
              ("frame_and_mounts", BODY - MOTORS - RESIDUAL - {49}, frame_mass),
              ("pcb_wiring_fasteners_residual", RESIDUAL, body_mass - allocated)]
    if groups[-1][2] <= 0:
        raise ValueError(f"Body mass must exceed the explicitly allocated {allocated:.3f} kg")
    report = []
    for label, numbers, mass in groups:
        volume = sum(parts[n]["volume_mm3"] for n in numbers)
        center = [sum(parts[n]["volume_mm3"] * parts[n]["centroid_m"][i]
                      for n in numbers) / volume for i in range(3)]
        # Mirror the battery to the opposite slot; the hardware location was
        # confirmed by the operator. This is NOT the original STEP placement.
        if label == "battery" and battery == "front":
            center[0] = -center[0]
        report.append({"group": label, "components": sorted(numbers),
                       "mass_kg": mass, "com_m": center})
    com = [sum(g["mass_kg"] * g["com_m"][i] for g in report) / body_mass
           for i in range(3)]
    return com, report


def rotate(point, q):
    x, z = point
    return [math.cos(q) * x + math.sin(q) * z,
            -math.sin(q) * x + math.cos(q) * z]


def whole_com(body, mass, hip, knee, pitch):
    # Existing GIM joint-axis measurements from PHYSICS.md, metres, native +Y.
    k = rotate([-.09816, -.01908], hip)
    w = rotate([.09816, -.01908], hip + knee)
    axle = [k[0] + w[0], .00828 + k[1] + w[1]]
    thigh = rotate([-.04908, -.00954], hip)
    thigh[1] += .00828
    shin = rotate([.08998, -.01749], hip + knee)
    shin = [shin[0] + k[0], shin[1] + k[1] + .00828]
    com = [(mass * body[j] + 2 * (.05 * thigh[i] + .24 * shin[i] + .02 * axle[i]))
           / (mass + .62) - axle[i] for i, j in [(0, 0), (1, 2)]]
    gravity = rotate(com, pitch)
    return {"hip_rad": hip, "knee_rad": knee, "body_pitch_rad": pitch,
            "com_from_axle_body_xz_m": com, "com_from_axle_gravity_xz_m": gravity,
            "physical_lean_rad": math.atan2(gravity[0], gravity[1]),
            "equilibrium_body_pitch_rad": -math.atan2(com[0], com[1])}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=ROOT / "assets/cad/ROBOT (GIM6010-8).STEP")
    parser.add_argument("--battery", choices=["front", "rear"], required=True)
    parser.add_argument("--body-mass", type=float, default=2.612)
    # Operator-stated masses (18/10): battery 0.25 kg, frame 0.35 kg; the
    # earlier 0.210 / 0.278 kg were inherited assignments. Not weighed here.
    # The 0.412 kg PCB/wiring residual is kept, so the body is 2.612 kg.
    parser.add_argument("--battery-mass", type=float, default=.25)
    parser.add_argument("--frame-mass", type=float, default=.35)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    origin, parts = read_parts(args.source)
    body, groups = body_com(parts, args.body_mass, args.battery, args.battery_mass, args.frame_mass)
    poses = {"cad_zero_body_level": (0, 0, 0),
             "extended_body_level": (-1.9722222222222222 / 2, 1.9722222222222222, 0),
             "b4_entry_log_163441": (-.745, 0, .453),
             "b4_hip_limit_log_171313": (.458, 0, -.471)}
    result = {"source": args.source.name,
              "source_sha256": hashlib.sha256(args.source.read_bytes()).hexdigest(),
              "method": "STEP solid centroids; uniform density within each assigned body mass group",
              "not_measured": "Mass distribution, distal link COMs, encoder zero and loaded dynamics",
              "frame": "+X forward, +Y left, +Z up; native +Y joint rotations",
              "cad_origin_mm": origin, "battery_slot": args.battery,
              "body_mass_kg": args.body_mass, "body_com_m": body,
              "body_groups": groups, "components": parts,
              "poses": {label: whole_com(body, args.body_mass, *q) for label, q in poses.items()}}
    text = json.dumps(result, indent=2, allow_nan=False) + "\n"
    if args.output:
        args.output.write_text(text, encoding="utf-8")
        print(json.dumps({key: result[key] for key in
                          ["source_sha256", "body_com_m", "poses"]}, indent=2))
    else:
        print(text, end="")


if __name__ == "__main__":
    main()
