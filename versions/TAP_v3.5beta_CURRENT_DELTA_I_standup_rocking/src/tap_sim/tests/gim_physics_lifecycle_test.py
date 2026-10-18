#!/usr/bin/env python3
"""GIM controller stops must not freeze MuJoCo gravity/contact integration."""

import argparse
from pathlib import Path
import subprocess

from native_runner_contract_test import fields


def check_cad_contact(path, view, fallen=False):
    import mujoco

    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    data.qpos[:7] = [float(view[k]) for k in ("x", "y", "z", "qw", "qx", "qy", "qz")]
    for joint, key in (("hip_left", "lh"), ("knee_left", "lk"), ("wheel_left", "lw"),
                       ("hip_right", "rh"), ("knee_right", "rk"), ("wheel_right", "rw")):
        data.qpos[model.jnt_qposadr[model.joint(joint).id]] = float(view[key])
    mujoco.mj_forward(model, data)
    body = model.body("gim_body").id
    ground = model.geom("ground").id
    bottoms = {}
    for geom in range(model.ngeom):
        if model.geom_type[geom] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        mesh = model.geom_dataid[geom]
        vertices = model.mesh_vert[model.mesh_vertadr[mesh]:model.mesh_vertadr[mesh] + model.mesh_vertnum[mesh]]
        link = model.body(model.geom_bodyid[geom]).name
        bottom = float(data.geom_xpos[geom, 2]
                       + (vertices @ data.geom_xmat[geom].reshape(3, 3)[2]).min())
        bottoms[link] = min(bottoms.get(link, float("inf")), bottom)
    for link, bottom in bottoms.items():
        assert bottom >= -.0001, f"resting CAD {link} penetrates floor: z={bottom} m"
    if not fallen:
        return
    assert abs(bottoms["gim_body"]) < .0001, "fallen CAD body must reach the floor"
    assert any(ground in (c.geom1, c.geom2)
               and any(model.geom_bodyid[g] == body and model.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH
                       for g in (c.geom1, c.geom2)) for c in data.contact), \
        "fallen body must contact its CAD hull, not an invisible oversized box"


def run(runner, model, action):
    p = subprocess.Popen(
        [str(runner), "--model", str(model), "--run-for", "10"],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, bufsize=1)
    states, views = [], []
    sent = False
    try:
        if action != "SPAWN":
            p.stdin.write("ENABLE\n")
            p.stdin.flush()
        for line in p.stdout:
            if line.startswith("STATE|"):
                state = fields(line)
                states.append(state)
                if action in ("DISABLE", "ESTOP", "RESET") and not sent and float(state["t"]) > .4:
                    p.stdin.write(action + "\n")
                    p.stdin.flush()
                    sent = True
                if action == "Fault" and not sent and float(state["t"]) >= 2:
                    p.stdin.write("PUSH 10 .3\n")
                    p.stdin.flush()
                    sent = True
            elif line.startswith("VIEW|"):
                views.append(fields(line))
        assert p.wait(timeout=2) == 0
    finally:
        if p.poll() is None:
            p.kill()
            p.wait()
        p.stdin.close()
        p.stdout.close()

    assert states and views
    check_cad_contact(model, views[-1], fallen=action == "Fault")
    if action == "Fault":
        stopped = [s for s in states if s["mode"] == "Fault"]
        assert stopped, "fixture did not exercise the fall fault"
        assert all(float(s[k]) == 0 for s in stopped for k in ("wheel_cmd_l", "wheel_cmd_r"))
    elif action in ("DISABLE", "ESTOP"):
        key = "enabled" if action == "DISABLE" else "estop"
        value = "0" if action == "DISABLE" else "1"
        stopped = [s for s in states if float(s["t"]) > .4 and s[key] == value]
        assert sent and stopped
    else:
        assert action == "SPAWN" or sent
        assert states[-1]["mode"] == "Idle"
        assert float(views[-1]["physics_t"]) == 0
        assert abs(float(views[-1]["z"]) - .216490386) < 1e-6
        assert abs(float(views[-1]["lk"]) - 1.97222222222) < 1e-5
        return

    after_stop = [v for v in views if float(v["t"]) >= float(stopped[0]["t"])]
    assert len(after_stop) > 2
    assert float(after_stop[-1]["physics_t"]) - float(after_stop[0]["physics_t"]) > .5, \
        f"{action} froze physics; a resting pose must still integrate time"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--runner", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    args = parser.parse_args()
    for action in ("SPAWN", "Fault", "DISABLE", "ESTOP", "RESET"):
        run(args.runner, args.model, action)
        print(f"PASS GIM physics lifecycle: {action}")


if __name__ == "__main__":
    main()
