#!/usr/bin/env python3
"""Open a MuJoCo gallery with each GIM STEP component separated."""

from pathlib import Path

import mujoco
import mujoco.viewer


ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / "src/tap_sim/model/gim_components.xml"
ASSETS = ROOT / "src/tap_sim/model/gim_components"


def main() -> None:
    xml = MODEL.read_text(encoding="utf-8")
    meshes = {path.name: path.read_bytes() for path in ASSETS.glob("*.stl")}
    if not meshes:
        raise FileNotFoundError(f"no GIM component meshes in {ASSETS}; run tools/import_gim_step_visual.py")
    model = mujoco.MjModel.from_xml_string(xml, assets=meshes)
    data = mujoco.MjData(model)
    print(f"Opening GIM component gallery: {model.ngeom} separate parts; static visual only.",
          flush=True)
    mujoco.viewer.launch(model, data)


if __name__ == "__main__":
    main()
