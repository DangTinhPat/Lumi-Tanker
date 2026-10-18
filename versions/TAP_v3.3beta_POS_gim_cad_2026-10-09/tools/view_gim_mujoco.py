#!/usr/bin/env python3
"""Open the current GIM STEP assembly as a static MuJoCo CAD view."""

from pathlib import Path

import mujoco
import mujoco.viewer


ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / "src/tap_sim/model/gim_visual.xml"
ASSETS = ROOT / "src/tap_sim/model/gim_visual"


def main() -> None:
    xml = MODEL.read_text(encoding="utf-8")
    meshes = {path.name: path.read_bytes() for path in ASSETS.glob("*.stl")}
    if not meshes:
        raise FileNotFoundError(f"no generated GIM mesh assets in {ASSETS}")
    model = mujoco.MjModel.from_xml_string(xml, assets=meshes)
    data = mujoco.MjData(model)
    print("Opening static GIM CAD view with six +Y joint-axis vectors; "
          "this scene has no robot dynamics.", flush=True)
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.opt.label = mujoco.mjtLabel.mjLABEL_SITE
        while viewer.is_running():
            viewer.sync()


if __name__ == "__main__":
    main()
