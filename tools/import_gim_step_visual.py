#!/usr/bin/env python3
"""Export a colored, static MuJoCo view of the complete GIM STEP assembly."""

from collections import defaultdict
import json
from pathlib import Path
import struct
import re

from OCP.BRep import BRep_Builder
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.Bnd import Bnd_Box
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.StlAPI import StlAPI_Writer
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDocStd import TDocStd_Document
from OCP.TDF import TDF_Label
from OCP.TopoDS import TopoDS_Compound
from OCP.TopLoc import TopLoc_Location
from OCP.XCAFApp import XCAFApp_Application
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.collections import Sequence_TDF_Label
from OCP.gp import gp_Trsf, gp_Vec


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "assets/cad/ROBOT (GIM6010-8).STEP"
ASSETS = ROOT / "simulation/model/gim_visual"
MODEL = ROOT / "simulation/model/gim_visual.xml"
COMPONENT_ASSETS = ROOT / "simulation/model/gim_components"
COMPONENT_MODEL = ROOT / "simulation/model/gim_components.xml"
COMPONENT_MANIFEST = ROOT / "simulation/model/gim_components.json"
MAX_TRIANGLES_PER_MESH = 180_000
MATERIALS = {
    "structure": ("structure", "0.68 0.70 0.72 1"),
    "motor": ("motor", "0.24 0.26 0.29 1"),
    "wheel": ("wheel", "0.08 0.09 0.10 1"),
    "electronics": ("electronics", "0.12 0.48 0.24 1"),
    "battery": ("battery", "0.72 0.20 0.14 1"),
    "spring": ("spring", "0.78 0.80 0.82 1"),
    "hardware": ("hardware", "0.48 0.50 0.53 1"),
}


def label_name(label: TDF_Label) -> str:
    name = TDataStd_Name()
    return name.Get().ToExtString() if label.FindAttribute(TDataStd_Name.GetID_s(), name) else ""


def material_for(name: str) -> str:
    value = name.casefold()
    if "wheel" in value:
        return "wheel"
    if "dong co" in value or "motor" in value and "bracket" not in value:
        return "motor"
    if "pcb" in value:
        return "electronics"
    if "baterry" in value or "battery" in value:
        return "battery"
    if "spring" in value:
        return "spring"
    if any(term in value for term in ("bearing", "m2", "m3", "4x10", "screw", "bolt")):
        return "hardware"
    return "structure"


def main() -> None:
    if not SOURCE.is_file():
        raise FileNotFoundError(SOURCE)

    app = XCAFApp_Application.GetApplication_s()
    doc = TDocStd_Document(TCollection_ExtendedString("gim_visual"))
    app.NewDocument(TCollection_ExtendedString("MDTV-XCAF"), doc)
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    reader.SetColorMode(True)
    if not reader.ReadFile(str(SOURCE)).name.endswith("RetDone"):
        raise RuntimeError(f"cannot read STEP: {SOURCE}")
    if not reader.Transfer(doc):
        raise RuntimeError(f"cannot transfer STEP assembly: {SOURCE}")

    shape_tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    colors = defaultdict(list)
    components = []

    def collect(label: TDF_Label, parent_location: TopLoc_Location) -> None:
        if shape_tool.IsReference_s(label):
            referred = TDF_Label()
            if not shape_tool.GetReferredShape_s(label, referred):
                raise RuntimeError("STEP assembly contains an unresolved component")
            location = parent_location.Multiplied(shape_tool.GetLocation_s(label))
            if shape_tool.IsAssembly_s(referred):
                collect(referred, location)
            else:
                shape = shape_tool.GetShape_s(referred)
                shape = BRepBuilderAPI_Transform(shape, location.Transformation(), True).Shape()
                category = material_for(label_name(referred))
                colors[category].append(shape)
                components.append((len(components) + 1, label_name(referred),
                                   label_name(label), category, shape))
        elif shape_tool.IsAssembly_s(label):
            children = Sequence_TDF_Label()
            shape_tool.GetComponents_s(label, children, False)
            for i in range(1, children.Length() + 1):
                collect(children.Value(i), parent_location)
        else:
            colors[material_for(label_name(label))].append(shape_tool.GetShape_s(label))

    roots = Sequence_TDF_Label()
    shape_tool.GetFreeShapes(roots)
    if roots.Length() == 0:
        raise RuntimeError("STEP assembly contains no shapes")
    for i in range(1, roots.Length() + 1):
        collect(roots.Value(i), TopLoc_Location())
    if not colors:
        raise RuntimeError("STEP assembly contains no component geometry")

    all_shapes = [shape for group in colors.values() for shape in group]
    bounds = Bnd_Box()
    for shape in all_shapes:
        BRepBndLib.Add_s(shape, bounds, True)
    xmin, ymin, zmin = bounds.GetXMin(), bounds.GetYMin(), bounds.GetZMin()
    xmax, ymax, zmax = bounds.GetXMax(), bounds.GetYMax(), bounds.GetZMax()
    if not all(v == v and abs(v) < 1.0e100 for v in (xmin, ymin, zmin, xmax, ymax, zmax)):
        raise RuntimeError("STEP assembly has invalid bounds")

    center = gp_Trsf()
    center.SetTranslation(gp_Vec(-0.5 * (xmin + xmax), -0.5 * (ymin + ymax),
                                 -0.5 * (zmin + zmax)))
    ASSETS.mkdir(parents=True, exist_ok=True)
    for old in ASSETS.glob("*.stl"):
        old.unlink()

    meshes = []
    writer = StlAPI_Writer()
    writer.ASCIIMode = False
    for category, shapes in colors.items():
        compound = TopoDS_Compound()
        builder = BRep_Builder()
        builder.MakeCompound(compound)
        for shape in shapes:
            centered = BRepBuilderAPI_Transform(shape, center, True).Shape()
            builder.Add(compound, centered)
        BRepMesh_IncrementalMesh(compound, 0.5, False, 0.35, True)
        raw_path = ASSETS / f"gim_{category}.stl"
        if not writer.Write(compound, str(raw_path)):
            raise RuntimeError(f"failed to export {category} mesh")
        raw = raw_path.read_bytes()
        if len(raw) < 84:
            raise RuntimeError(f"invalid {category} STL")
        triangle_count = struct.unpack_from("<I", raw, 80)[0]
        if len(raw) != 84 + triangle_count * 50 or triangle_count == 0:
            raise RuntimeError(f"invalid triangle data in {category} STL")
        raw_path.unlink()
        triangles = memoryview(raw)[84:]
        for index, start in enumerate(range(0, triangle_count, MAX_TRIANGLES_PER_MESH)):
            count = min(MAX_TRIANGLES_PER_MESH, triangle_count - start)
            filename = f"gim_{category}_{index}.stl"
            header = f"GIM6010 static CAD {category}".encode().ljust(80, b"\0")
            (ASSETS / filename).write_bytes(
                header + struct.pack("<I", count) + triangles[start * 50:(start + count) * 50]
            )
            meshes.append((category, filename))

    asset_xml = "\n".join(
        f'    <mesh name="{Path(filename).stem}" file="{filename}" scale="0.001 0.001 0.001"/>'
        for _, filename in meshes
    )
    material_xml = "\n".join(
        f'    <material name="gim_{key}" rgba="{rgba}"/>'
        for key, (_, rgba) in MATERIALS.items() if any(group == key for group, _ in meshes)
    )
    geom_xml = "\n".join(
        f'    <geom name="{Path(filename).stem}" type="mesh" mesh="{Path(filename).stem}" '
        f'material="gim_{category}" contype="0" conaffinity="0"/>'
        for category, filename in meshes
    )
    axes = {
        "hip_left": (0.0, 0.1025, 0.00828, "0.10 0.45 1 1"),
        "hip_right": (0.0, -0.1025, 0.00828, "0.10 0.45 1 1"),
        "knee_left": (-0.09816, 0.1130, -0.01080, "1 0.25 0.12 1"),
        "knee_right": (-0.09816, -0.1130, -0.01080, "1 0.25 0.12 1"),
        "wheel_left": (0.0, 0.1327, -0.02988, "0.20 0.95 0.35 1"),
        "wheel_right": (0.0, -0.1327, -0.02988, "0.20 0.95 0.35 1"),
    }
    axis_xml = []
    for name, (x, y, z, color) in axes.items():
        # A short +Y shaft with a four-spoke tip makes direction legible in 3D.
        axis_xml.extend((
            f'    <geom name="axis_{name}" type="capsule" '
            f'fromto="{x} {y - .03} {z} {x} {y + .03} {z}" size="0.0018" '
            f'rgba="{color}" contype="0" conaffinity="0"/>',
            f'    <site name="axis_label_{name}" type="sphere" '
            f'pos="{x} {y + .03} {z}" size="0.001" rgba="{color}"/>',
        ))
        for dx, dz in ((.006, 0), (-.006, 0), (0, .006), (0, -.006)):
            axis_xml.append(
                f'    <geom name="axis_tip_{name}_{dx}_{dz}" type="capsule" '
                f'fromto="{x + dx} {y + .022} {z + dz} {x} {y + .03} {z}" '
                f'size="0.0014" rgba="{color}" contype="0" conaffinity="0"/>'
            )
    MODEL.write_text(
        '<mujoco model="gim6010_cad_visual">\n'
        '  <compiler angle="radian"/>\n'
        '  <option gravity="0 0 0"/>\n'
        '  <visual>\n'
        '    <global azimuth="135" elevation="-20" fovy="45"/>\n'
        '    <headlight ambient="0.65 0.65 0.65" diffuse="0.8 0.8 0.8" '
        'specular="0.25 0.25 0.25"/>\n'
        '  </visual>\n  <asset>\n' + asset_xml + '\n' + material_xml +
        '\n  </asset>\n  <worldbody>\n' + geom_xml + '\n' +
        '\n'.join(axis_xml) + '\n  </worldbody>\n</mujoco>\n',
        encoding="utf-8",
    )

    columns = 8
    spacing_m = 0.42
    component_meshes = []
    component_manifest = []
    COMPONENT_ASSETS.mkdir(parents=True, exist_ok=True)
    for old in COMPONENT_ASSETS.glob("*.stl"):
        old.unlink()
    for number, part_name, occurrence, category, shape in components:
        part_bounds = Bnd_Box()
        BRepBndLib.Add_s(shape, part_bounds, True)
        px, py, pz = (0.5 * (part_bounds.GetXMin() + part_bounds.GetXMax()),
                      0.5 * (part_bounds.GetYMin() + part_bounds.GetYMax()),
                      0.5 * (part_bounds.GetZMin() + part_bounds.GetZMax()))
        cad_center_m = ((px - 0.5 * (xmin + xmax)) * 0.001,
                        (py - 0.5 * (ymin + ymax)) * 0.001,
                        (pz - 0.5 * (zmin + zmax)) * 0.001)
        local = gp_Trsf()
        local.SetTranslation(gp_Vec(-px, -py, -pz))
        local_shape = BRepBuilderAPI_Transform(shape, local, True).Shape()
        BRepMesh_IncrementalMesh(local_shape, 0.5, False, 0.35, True)
        safe_part = re.sub(r"[^A-Za-z0-9]+", "_", part_name).strip("_") or "part"
        safe_occurrence = re.sub(r"[^A-Za-z0-9]+", "_", occurrence).strip("_") or "item"
        stem = f"part_{number:02d}_{safe_part}_{safe_occurrence}"
        raw_path = COMPONENT_ASSETS / f"{stem}.stl"
        if not writer.Write(local_shape, str(raw_path)):
            raise RuntimeError(f"failed to export component {part_name}")
        raw = raw_path.read_bytes()
        if len(raw) < 84:
            raise RuntimeError(f"invalid STL for component {part_name}")
        triangle_count = struct.unpack_from("<I", raw, 80)[0]
        if len(raw) != 84 + triangle_count * 50 or triangle_count == 0:
            raise RuntimeError(f"invalid triangle data for component {part_name}")
        raw_path.unlink()
        faces = memoryview(raw)[84:]
        col, row = (number - 1) % columns, (number - 1) // columns
        x = (col - (columns - 1) / 2) * spacing_m
        rows = (len(components) + columns - 1) // columns
        y = (row - (rows - 1) / 2) * spacing_m
        for index, start in enumerate(range(0, triangle_count, MAX_TRIANGLES_PER_MESH)):
            count = min(MAX_TRIANGLES_PER_MESH, triangle_count - start)
            mesh_name = f"{stem}_{index}"
            filename = f"{mesh_name}.stl"
            header = f"GIM6010 component {part_name} {occurrence}".encode()[:80].ljust(80, b"\0")
            (COMPONENT_ASSETS / filename).write_bytes(
                header + struct.pack("<I", count) + faces[start * 50:(start + count) * 50]
            )
            component_meshes.append((mesh_name, filename, category, x, y,
                                     f"{number:02d}_{safe_part}_{safe_occurrence}"))
        component_manifest.append({
            "number": number, "part": part_name, "occurrence": occurrence,
            "category": category, "center_m": cad_center_m,
            "mesh_stem": stem,
        })

    COMPONENT_MANIFEST.write_text(json.dumps({
        "source": SOURCE.name, "frame": "centered CAD bbox, +X forward +Y left +Z up",
        "components": component_manifest,
    }, indent=2) + "\n", encoding="utf-8")

    component_assets_xml = "\n".join(
        f'    <mesh name="{mesh}" file="{filename}" scale="0.001 0.001 0.001"/>'
        for mesh, filename, _, _, _, _ in component_meshes
    )
    component_materials_xml = "\n".join(
        f'    <material name="gim_{key}" rgba="{rgba}"/>'
        for key, (_, rgba) in MATERIALS.items()
        if any(category == key for _, _, category, _, _, _ in component_meshes)
    )
    component_geoms_xml = "\n".join(
        f'    <geom name="{name}" type="mesh" mesh="{mesh}" material="gim_{category}" '
        f'pos="{x:.3f} {y:.3f} 0" contype="0" conaffinity="0"/>'
        for mesh, _, category, x, y, name in component_meshes
    )
    COMPONENT_MODEL.write_text(
        '<mujoco model="gim6010_component_gallery">\n'
        '  <compiler angle="radian"/>\n'
        '  <option gravity="0 0 0"/>\n'
        '  <visual><global azimuth="90" elevation="-85" fovy="45"/></visual>\n'
        '  <asset>\n' + component_assets_xml + '\n' + component_materials_xml +
        '\n  </asset>\n  <worldbody>\n' + component_geoms_xml +
        '\n  </worldbody>\n</mujoco>\n', encoding="utf-8")

    print(f"wrote {sum(len(v) for v in colors.values())} components as {len(meshes)} meshes")
    print(f"wrote component gallery with {len(components)} individually named parts")
    print("display colors are representative component colors; STEP has no color metadata")
    print("STEP bounds (mm):", tuple(round(v, 3) for v in
          (xmin, ymin, zmin, xmax, ymax, zmax)))


if __name__ == "__main__":
    main()
