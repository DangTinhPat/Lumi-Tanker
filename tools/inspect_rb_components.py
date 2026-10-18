#!/usr/bin/env python3
"""Export selected STEP component instances for CAD mechanism inspection.

This is an authoring-time diagnostic.  The generated STL files are written to
the requested scratch directory and are not runtime dependencies.
"""

from __future__ import annotations

import argparse
from pathlib import Path

from OCP.BRep import BRep_Builder
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.StlAPI import StlAPI_Writer
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDocStd import TDocStd_Document
from OCP.TopoDS import TopoDS_Compound
from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ShapeTool
from OCP.collections import Sequence_TDF_Label


def component_box(shape) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    box = Bnd_Box()
    BRepBndLib.Add_s(shape, box)
    lo = (box.GetXMin(), box.GetYMin(), box.GetZMin())
    hi = (box.GetXMax(), box.GetYMax(), box.GetZMax())
    return lo, hi


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("step", type=Path)
    parser.add_argument("--indices", nargs="+", type=int,
                        default=[1118, 1119, 1125, 1170, 1176, 1177])
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--deflection-mm", type=float, default=0.10)
    args = parser.parse_args()

    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    status = reader.ReadFile(str(args.step))
    if str(status).split(".")[-1] != "IFSelect_RetDone":
        raise RuntimeError(f"STEP read failed: {status}")
    document = TDocStd_Document(TCollection_ExtendedString("rb-inspect"))
    if not reader.Transfer(document):
        raise RuntimeError("STEP transfer failed")
    shape_tool = XCAFDoc_DocumentTool.ShapeTool_s(document.Main())
    free = Sequence_TDF_Label()
    shape_tool.GetFreeShapes(free)
    components = Sequence_TDF_Label()
    XCAFDoc_ShapeTool.GetComponents_s(free.Value(1), components)
    if any(index < 1 or index > components.Size() for index in args.indices):
        raise ValueError(f"component index outside 1..{components.Size()}")

    args.output.mkdir(parents=True, exist_ok=True)
    for index in args.indices:
        shape = shape_tool.GetShape_s(components.Value(index))
        lo, hi = component_box(shape)
        print(f"{index}: center="
              f"{tuple(round((a + b) / 2.0, 3) for a, b in zip(lo, hi))} mm "
              f"size={tuple(round(b - a, 3) for a, b in zip(lo, hi))} mm")
        builder = BRep_Builder()
        compound = TopoDS_Compound()
        builder.MakeCompound(compound)
        builder.Add(compound, shape)
        BRepMesh_IncrementalMesh(compound, args.deflection_mm, False, 0.30, True)
        destination = args.output / f"component_{index}.stl"
        if not StlAPI_Writer().Write(compound, str(destination)):
            raise RuntimeError(f"failed to write {destination}")


if __name__ == "__main__":
    main()
