#!/usr/bin/env python3
"""Extract the seven moving visual links from the RB STEP assembly.

The supplied STEP is a complete assembly, in millimetres, saved in a folded
pose.  It contains many PCB/mechanical subcomponents rather than seven named
MuJoCo links.  This importer keeps the complete detailed assembly geometry, uses
the known CAD joint frames to undo the saved pose, and writes seven STL files
in link-local metres.

This is an authoring-time tool.  OCP is intentionally not a runtime
dependency; the MJCF model consumes only the generated STL assets.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import tempfile

import numpy as np
import trimesh

from OCP.BRep import BRep_Builder
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.StlAPI import StlAPI_Writer
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDocStd import TDocStd_Document
from OCP.TopoDS import TopoDS_Compound
from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ShapeTool
from OCP.collections import Sequence_TDF_Label


# CAD assembly axes -> native robot link axes.  CAD +X is the common joint
# axis, and positive CAD X is the left side.  The robot control contract keeps
# +Y on both sides after this mapping.
CAD_TO_LINK = np.array([
    [0.0, 0.0, -1.0],
    [1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
], dtype=float)

# The assembly was saved in this symmetric folded pose.  These values are
# obtained from the CAD wheel centre and the existing CAD-zero origins, not
# from simulator pose or controller telemetry.
ASSEMBLY_HIP_RAD = -0.39507567
ASSEMBLY_KNEE_RAD = -1.64961287

HIP_CAD = np.array([125.0, 0.0, 0.0])
KNEE_CAD_ZERO = np.array([127.0, 104.76742, 14.10935])
WHEEL_CAD_ZERO = np.array([154.5, 204.76742, -32.52141])

# In the supplied STEP the terminal wheel centre is [154.5, 15, -95] mm.
# The two-link FK below reproduces that location from the CAD-zero origins.

# The STEP's final mechanical subassembly is the pair of 28-component side
# groups.  All earlier components are the body/electronics assembly.  These
# indices are documented because the exporter intentionally refuses to guess
# from product names: the source uses generic names such as ``实体1`` and
# library names for mechanical parts.
RIGHT_COMPONENTS = range(1108, 1136)  # positive CAD X
LEFT_COMPONENTS = range(1159, 1187)   # negative CAD X
# The STEP stores the knee-assist barrel/rod as one compound component.  It
# must not be assigned wholly to the thigh: its lower end is carried by the
# shin-side lever as the knee moves.
ASSIST_COMPONENTS = {1133: "left", 1180: "right"}
ASSIST_SPLIT_Z_M = -0.055


def rot_x(angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[1.0, 0.0, 0.0],
                     [0.0, c, -s],
                     [0.0, s, c]], dtype=float)


def component_center(shape) -> np.ndarray:
    # Bounding-box centres are robust for zero-volume imported PCB faces and
    # are sufficient for assigning a complete component to one moving link.
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib

    box = Bnd_Box()
    BRepBndLib.Add_s(shape, box)
    return np.array([
        (box.GetXMin() + box.GetXMax()) * 0.5,
        (box.GetYMin() + box.GetYMax()) * 0.5,
        (box.GetZMin() + box.GetZMax()) * 0.5,
    ], dtype=float)


def segment_distance(point: np.ndarray, start: np.ndarray,
                     end: np.ndarray) -> float:
    direction = end - start
    fraction = np.clip(np.dot(point - start, direction) /
                       np.dot(direction, direction), 0.0, 1.0)
    return float(np.linalg.norm(point - (start + fraction * direction)))


def assembly_frames(side_sign: float):
    hip = HIP_CAD.copy()
    knee_zero = KNEE_CAD_ZERO.copy()
    wheel_zero = WHEEL_CAD_ZERO.copy()
    hip[0] *= side_sign
    knee_zero[0] *= side_sign
    wheel_zero[0] *= side_sign

    # Joint offsets are in the CAD sagittal Y/Z plane and rotate about CAD X.
    hip_to_knee = knee_zero - hip
    knee_to_wheel = wheel_zero - knee_zero
    knee_pose = hip + rot_x(ASSEMBLY_HIP_RAD) @ hip_to_knee
    wheel_pose = knee_pose + rot_x(ASSEMBLY_HIP_RAD + ASSEMBLY_KNEE_RAD) @ knee_to_wheel
    return hip, knee_pose, wheel_pose


def classify_component(index: int, center: np.ndarray) -> str:
    if index in ASSIST_COMPONENTS:
        return f"{ASSIST_COMPONENTS[index]}_assist"
    if index in RIGHT_COMPONENTS:
        side = "left"
        sign = 1.0
    elif index in LEFT_COMPONENTS:
        side = "right"
        sign = -1.0
    else:
        return "body"

    hip, knee, wheel = assembly_frames(sign)
    # The exact wheel parts are centred on the wheel axle.  The remaining
    # side components are separated by their nearest posed link corridor.
    if np.linalg.norm(center - wheel) <= 25.0:
        return f"{side}_wheel"
    thigh_distance = segment_distance(center, hip, knee)
    shin_distance = segment_distance(center, knee, wheel)
    return f"{side}_thigh" if thigh_distance <= shin_distance else f"{side}_shin"


def make_compound(shapes):
    builder = BRep_Builder()
    compound = TopoDS_Compound()
    builder.MakeCompound(compound)
    for shape in shapes:
        builder.Add(compound, shape)
    return compound


def mesh_group(compound, destination: Path, deflection_mm: float) -> None:
    BRepMesh_IncrementalMesh(compound, deflection_mm, False, 0.30, True)
    if not StlAPI_Writer().Write(compound, str(destination)):
        raise RuntimeError(f"failed to write {destination}")


def transform_mesh(source: Path, destination: Path, name: str) -> dict:
    mesh = trimesh.load_mesh(source, process=False)
    if not isinstance(mesh, trimesh.Trimesh):
        raise ValueError(f"{source} did not produce a triangle mesh")

    if name == "body":
        # Preserve the project contract that the body visual is mounted at
        # -pi/2 while making the final world orientation equal CAD_TO_LINK.
        visual_pitch = rot_x(0.0)  # kept explicit for the frame calculation
        del visual_pitch
        raw_rotation = np.array([
            [0.0, 0.0, 1.0],
            [0.0, 1.0, 0.0],
            [-1.0, 0.0, 0.0],
        ]) @ CAD_TO_LINK
        mesh.vertices = mesh.vertices @ raw_rotation.T * 0.001
    else:
        side = 1.0 if name.startswith("left_") else -1.0
        hip, knee, wheel = assembly_frames(side)
        if name.endswith("thigh"):
            mesh.vertices = ((mesh.vertices - hip) @ rot_x(-ASSEMBLY_HIP_RAD).T)
        elif name.endswith("shin"):
            mesh.vertices = ((mesh.vertices - knee) @
                             rot_x(-(ASSEMBLY_HIP_RAD + ASSEMBLY_KNEE_RAD)).T)
        elif name.endswith("wheel"):
            mesh.vertices = ((mesh.vertices - wheel) @
                             rot_x(-(ASSEMBLY_HIP_RAD + ASSEMBLY_KNEE_RAD)).T)
        else:
            raise ValueError(name)
        mesh.vertices = mesh.vertices @ CAD_TO_LINK.T * 0.001

    # ``remove_duplicate_faces`` was removed from recent trimesh releases;
    # ``unique_faces`` is the equivalent stable API and preserves the source
    # triangulation without welding away intentional CAD seams.
    mesh.update_faces(mesh.unique_faces())
    mesh.remove_unreferenced_vertices()
    destination.parent.mkdir(parents=True, exist_ok=True)
    mesh.export(destination, file_type="stl")
    return {
        "vertices": int(len(mesh.vertices)),
        "faces": int(len(mesh.faces)),
        "bounds_m": mesh.bounds.tolist(),
        "center_of_mass_m": mesh.center_mass.tolist(),
        "watertight": bool(mesh.is_watertight),
        "sha256": hashlib.sha256(destination.read_bytes()).hexdigest(),
    }


def transform_assist_mesh(source: Path, destination_root: Path, side: str) -> dict:
    """Split the CAD assist compound at its fixed/moving interface.

    The split is made in the unposed thigh frame, where the upper barrel is
    fixed to the thigh and the lower clevis/rod is fixed to the shin.  Both
    outputs remain direct triangles from the STEP; the MuJoCo tendon fills
    the moving interface between them.
    """
    mesh = trimesh.load_mesh(source, process=False)
    if not isinstance(mesh, trimesh.Trimesh):
        raise ValueError(f"{source} did not produce a triangle mesh")

    side_sign = 1.0 if side == "left" else -1.0
    hip, knee, _ = assembly_frames(side_sign)
    thigh_vertices = ((mesh.vertices - hip) @
                      rot_x(-ASSEMBLY_HIP_RAD).T)
    thigh_vertices = thigh_vertices @ CAD_TO_LINK.T * 0.001
    face_vertices_z = thigh_vertices[mesh.faces, 2]
    # Omit the few triangles that straddle the cut.  The physical tendon is
    # the intentional bridge, so keeping either side's triangles would make
    # one visual half leak across the moving interface.
    upper_faces = np.all(face_vertices_z >= ASSIST_SPLIT_Z_M, axis=1)
    lower_faces = np.all(face_vertices_z < ASSIST_SPLIT_Z_M, axis=1)

    def export_part(mask: np.ndarray, vertices: np.ndarray,
                    destination: Path) -> dict:
        selected_faces = mesh.faces[mask]
        selected_vertices, inverse = np.unique(selected_faces,
                                                return_inverse=True)
        part = trimesh.Trimesh(
            vertices=vertices[selected_vertices],
            faces=inverse.reshape((-1, 3)),
            process=False,
        )
        part.update_faces(part.unique_faces())
        part.remove_unreferenced_vertices()
        destination.parent.mkdir(parents=True, exist_ok=True)
        part.export(destination, file_type="stl")
        return {
            "vertices": int(len(part.vertices)),
            "faces": int(len(part.faces)),
            "bounds_m": part.bounds.tolist(),
            "sha256": hashlib.sha256(destination.read_bytes()).hexdigest(),
        }

    upper = export_part(upper_faces, thigh_vertices,
                        destination_root / f"{side}_assist_upper.stl")

    # Rebuild the lower piece in the CAD-zero shin frame.  This is the key
    # topology correction: it follows the shin rather than remaining rigidly
    # attached to the thigh.
    shin_vertices = ((mesh.vertices - knee) @
                     rot_x(-(ASSEMBLY_HIP_RAD + ASSEMBLY_KNEE_RAD)).T)
    shin_vertices = shin_vertices @ CAD_TO_LINK.T * 0.001
    lower = export_part(lower_faces, shin_vertices,
                        destination_root / f"{side}_assist_lower.stl")
    return {"upper": upper, "lower": lower}


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("step", type=Path)
    parser.add_argument("--output", type=Path,
                        default=root / "src/tap_sim/model/assets")
    parser.add_argument("--deflection-mm", type=float, default=0.20)
    args = parser.parse_args()
    if args.deflection_mm <= 0.0:
        raise SystemExit("--deflection-mm must be positive")

    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    status = reader.ReadFile(str(args.step))
    if str(status).split(".")[-1] != "IFSelect_RetDone":
        raise RuntimeError(f"STEP read failed: {status}")
    document = TDocStd_Document(TCollection_ExtendedString("rb"))
    if not reader.Transfer(document):
        raise RuntimeError("STEP transfer failed")
    shape_tool = XCAFDoc_DocumentTool.ShapeTool_s(document.Main())
    free = Sequence_TDF_Label()
    shape_tool.GetFreeShapes(free)
    if free.Size() != 1:
        raise RuntimeError(f"expected one STEP assembly, found {free.Size()}")
    components = Sequence_TDF_Label()
    XCAFDoc_ShapeTool.GetComponents_s(free.Value(1), components)
    if components.Size() < max(LEFT_COMPONENTS):
        raise RuntimeError(f"unexpected component count: {components.Size()}")

    grouped = {name: [] for name in
               ("body", "left_thigh", "left_shin", "left_wheel",
                "right_thigh", "right_shin", "right_wheel",
                "left_assist", "right_assist")}
    for index in range(1, components.Size() + 1):
        shape = shape_tool.GetShape_s(components.Value(index))
        grouped[classify_component(index, component_center(shape))].append(shape)

    args.output.mkdir(parents=True, exist_ok=True)
    metadata = {
        "source_step": str(args.step),
        "source_sha256": hashlib.sha256(args.step.read_bytes()).hexdigest(),
        "source_length_unit": "millimetre",
        "component_count": components.Size(),
        "mesh_deflection_mm": args.deflection_mm,
        "assembly_pose_rad": {
            "hip": ASSEMBLY_HIP_RAD,
            "knee": ASSEMBLY_KNEE_RAD,
        },
        "component_groups": {},
    }
    with tempfile.TemporaryDirectory(prefix="tap-rb-step-") as temporary:
        temporary = Path(temporary)
        for name, shapes in grouped.items():
            if not shapes:
                raise RuntimeError(f"empty link group: {name}")
            raw = temporary / f"{name}.stl"
            mesh_group(make_compound(shapes), raw, args.deflection_mm)
            if name.endswith("_assist"):
                side = name.removesuffix("_assist")
                metadata["component_groups"][name] = {
                    "component_count": len(shapes),
                    **transform_assist_mesh(raw, args.output, side),
                }
            else:
                metadata["component_groups"][name] = {
                    "component_count": len(shapes),
                    **transform_mesh(raw, args.output / f"{name}.stl", name),
                }

    # The native default is ``model/assets``; for an arbitrary authoring
    # output directory keep the manifest beside that directory instead of
    # accidentally resolving to the filesystem root.
    manifest_root = (args.output.parent if args.output.name == "assets"
                     else args.output)
    manifest = manifest_root / "rb_cad_manifest.json"
    manifest.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    for name, info in metadata["component_groups"].items():
        if name.endswith("_assist"):
            print(f"{name}: {info['component_count']} components, "
                  f"upper={info['upper']['faces']} faces, "
                  f"lower={info['lower']['faces']} faces")
        else:
            print(f"{name}: {info['component_count']} components, "
                  f"{info['vertices']} vertices, {info['faces']} faces, "
                  f"watertight={info['watertight']}, sha256={info['sha256']}")
    print(f"manifest: {manifest}")


if __name__ == "__main__":
    main()
