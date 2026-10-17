#!/usr/bin/env python3
"""Sensor-only recovery; ground-truth contact/CoM checks are offline scoring."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import math
import random
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import mujoco
from native_runner_contract_test import fields


def run(args, name, extra, duration=24.0, start=.2, model=None):
    command = [str(args.runner), "--model", str(model or args.model), "--rate", "3000",
               "--run-for", str(duration), "--start-key", "home", "--roll", "1",
               "--at", f"{start}:STANDUP", *extra]
    result = subprocess.run(command, input="", capture_output=True, text=True,
                            timeout=180, check=False)
    (args.evidence / f"{name}.log").write_text(result.stdout)
    assert result.returncode == 0, f"{name}: runner exited {result.returncode}: {result.stderr}"
    states = [fields(line) for line in result.stdout.splitlines() if line.startswith("STATE|")]
    views = [fields(line) for line in result.stdout.splitlines() if line.startswith("VIEW|")]
    assert states and views, f"{name}: missing telemetry"
    assert all(s["ground_truth_to_controller"] == "0" for s in states)
    if any(s["mode"] == "Balancing" for s in states):
        events = [fields(line) for line in result.stdout.splitlines()
                  if line.startswith("RECOVERY_HANDOFF|")]
        assert len(events) == 1, f"{name}: missing handoff tick"
        event = events[0]
        for key in ("plan_v", "lqr_vref"):
            assert abs(float(event[key]) - float(event["ground_velocity"])) < .001, \
                f"{name}: {key} lost rolling handoff velocity"
    return name, states, views, command


def score(model, result):
    name, states, views, command = result
    assert not any(s["mode"] == "Fault" for s in states), f"{name}: Fault"
    first = {}
    for s in states:
        first.setdefault(s["su_phase_name"], s)
    assert {"Folding", "Kneeling", "Pitching", "Catching", "Support"} <= first.keys(), name
    pause = float(first["Pitching"]["t"]) - float(first["Kneeling"]["t"])
    assert .99 <= pause <= 1.04, f"{name}: kneeling pause {pause}"
    balance = [s for s in states if s["mode"] == "Balancing"]
    assert balance and float(balance[0]["t"]) < 9, f"{name}: failed to capture promptly"
    handoff = float(balance[0]["t"])
    catch_start = float(first["Catching"]["t"])
    assert handoff - catch_start < 2.0, f"{name}: capture took too long"
    assert all(s["mode"] == "Balancing" for s in states if float(s["t"]) >= handoff)
    assert abs(float(first["Support"]["su_capture"])) <= .061, f"{name}: divergent handoff"
    tail = [s for s in states if float(s["t"]) >= 22]
    assert tail and max(abs(float(s["pitch"])) for s in tail) < .05, f"{name}: body not level"
    assert max(abs(float(s["ground_velocity"])) for s in tail) < .08, f"{name}: did not stop"
    assert max(abs(float(s[k])) for s in states for k in ("wheel_tq_l", "wheel_tq_r")) <= .20001
    # Telemetry is emitted before the next physics step, so allow one 20 ms
    # sample at cap transitions. The controller's exact expiry is host-tested.
    base_only = [s for s in states if float(s["t"]) < catch_start-.02 or
                 float(s["t"]) > min(catch_start+.6, handoff)+.04]
    assert max(abs(float(s[k])) for s in base_only for k in ("wheel_tq_l", "wheel_tq_r")) <= .13001
    peak = [s for s in states if s.get("su_peak") == "1"]
    assert peak and all(s["su_phase_name"] == "Catching" for s in peak)
    peak_window = float(peak[-1]["t"])-float(peak[0]["t"])+.02
    assert peak_window <= .621, f"{name}: peak window did not expire"
    catch = [s for s in states if s["su_phase_name"] == "Catching"]
    hip_rates = [float(s["su_hip_rate"]) for s in catch]
    assert max(hip_rates) > .1, f"{name}: no hip lift"
    # Successful fast capture need not reverse the hip; both signs remain
    # covered by the feedback unit contract, without demanding oscillation.
    assert max(abs(float(s["su_wheel"])) for s in catch) <= 16.001
    assert first["Catching"]["su_clearance_valid"] == "1"
    assert float(first["Catching"]["su_clearance"]) >= .003
    # Replay only after the runner exits. No exact pose/contact goes back to it.
    data = mujoco.MjData(model)
    ground = model.geom("ground").id
    wheels = {model.geom("left_wheel_collision").id, model.geom("right_wheel_collision").id}
    body = model.body("gim_body").id
    left = model.body("left_wheel").id
    right = model.body("right_wheel").id
    shell = {g for g in range(model.ngeom) if model.geom_bodyid[g] == body and model.geom_contype[g]}
    com_error = 0.0
    angles = []
    replay = [v for v in views if float(v["t"]) >= catch_start]
    for v in replay:
        data.qpos[:] = [float(v[k]) for k in ("x", "y", "z", "qw", "qx", "qy", "qz",
                                               "lh", "lk", "lw", "rh", "rk", "rw")]
        mujoco.mj_forward(model, data)
        supports = set()
        for contact in data.contact:
            if contact.geom1 == ground:
                supports.add(contact.geom2)
            elif contact.geom2 == ground:
                supports.add(contact.geom1)
        assert not supports & shell, f"{name}: shell returned to floor at t={v['t']}"
        if float(v["t"]) >= handoff:
            assert supports == wheels, f"{name}: non-wheel support at t={v['t']}: {supports}"
        w, x, y, z = (float(v[k]) for k in ("qw", "qx", "qy", "qz"))
        angle = math.asin(max(-1, min(1, 2*(w*y-z*x))))
        angles.append((float(v["t"]), angle))
        if float(v["t"]) >= 22:
            axle_x = .5 * (data.xpos[left, 0] + data.xpos[right, 0])
            com_error = max(com_error, abs(float(data.subtree_com[body, 0]) - axle_x))
            assert .12 < float(v["z"]) < .14, f"{name}: did not rise to requested 130 mm"
    assert com_error < .006, f"{name}: true CoM remains {com_error} m from axle"
    back = max(0, -min(a for _, a in angles))
    assert back <= .4, f"{name}: backward pitch {back:.4f} rad exceeds 0.4"
    # Any backward excursion past 0.1 rad must return to |pitch|<=0.1 for
    # a continuous 0.5 s, with return beginning within 1.5 s of that excursion.
    recovery = 0.0; excursion = None; level_since = None
    for t, angle in angles:
        if excursion is None and angle < -.1:
            excursion = t
        if excursion is not None:
            level_since = (t if level_since is None else level_since) if abs(angle) <= .1 else None
            if level_since is not None and t-level_since >= .5:
                recovery = max(recovery, level_since-excursion)
                excursion = level_since = None
    assert excursion is None, f"{name}: backward excursion did not recover"
    assert recovery <= 1.5, f"{name}: backward recovery {recovery:.3f} s"
    return dict(case=name, pause_s=round(pause, 3), handoff_s=handoff,
                catch_s=round(handoff-catch_start, 3),
                back_max_rad=back, recovery_s=recovery, peak_window_s=peak_window,
                shell_contacts_after_catch=0,
                tail_pitch_max_rad=max(abs(float(s["pitch"])) for s in tail),
                tail_speed_max_m_s=max(abs(float(s["ground_velocity"])) for s in tail),
                true_com_axle_error_max_m=com_error, command=command)


def model_variant(args, name, com_dx=0, mass_scale=1, friction=None, hip_delta=0):
    """Perturb the plant only. Controller geometry remains the nominal model."""
    tree = ET.parse(args.model)
    for mesh in tree.findall('./asset/mesh'):
        mesh.set('file', str((args.model.parent / mesh.get('file')).resolve()))
    body = tree.find('./worldbody/body[@name="gim_body"]/inertial')
    pos = list(map(float, body.get('pos').split())); pos[0] += com_dx
    body.set('pos', ' '.join(map(str, pos)))
    body.set('mass', str(float(body.get('mass')) * mass_scale))
    inertia = 'fullinertia' if body.get('fullinertia') else 'diaginertia'
    body.set(inertia, ' '.join(str(float(x)*mass_scale) for x in body.get(inertia).split()))
    if friction is not None:
        for pair in tree.findall('./contact/pair'):
            values = pair.get('friction').split(); values[:2] = [str(friction)]*2
            pair.set('friction', ' '.join(values))
    if hip_delta:
        key = tree.find('./keyframe/key[@name="home"]')
        q = list(map(float, key.get('qpos').split())); q[7] += hip_delta; q[10] += hip_delta
        # Lift the reset pose enough to avoid initial shell/wheel penetration;
        # it settles freely before STANDUP, without a fixture.
        q[2] += .003
        key.set('qpos', ' '.join(map(str, q)))
    folder = args.evidence / 'models'; folder.mkdir(exist_ok=True)
    path = folder / f'{name}.xml'; tree.write(path)
    return path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--runner", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--evidence", type=Path)
    parser.add_argument("--cases", default="all")
    parser.add_argument("--extended", action="store_true", help="include plant mismatch and sensor noise cases")
    parser.add_argument("--standup", default="", help="explicit tuning override, recorded in commands")
    parser.add_argument("--random-cases", type=int, default=0, help="fixed-seed combined perturbation cases")
    args = parser.parse_args()
    args.runner = args.runner.resolve(); args.model = args.model.resolve()
    args.evidence = args.evidence or Path(tempfile.mkdtemp(prefix="tap-standup-"))
    args.evidence.mkdir(parents=True, exist_ok=True)
    root = Path(__file__).resolve().parents[3]
    sources = [Path(__file__), args.runner, args.model,
               root / "src_fw/include/tap_fw/v3/StandUp.hpp",
               root / "src_fw/src/v3/V3Controller.cpp",
               root / "src/tap_sim/native/gim_v3_runner.cpp",
               root / "src/tap_control/core/include/robot_control/GeneratedGeometry.hpp",
               root / "src/tap_control/core/include/robot_control/GeneratedBodyEnvelope.hpp"]
    (args.evidence / "fingerprints.json").write_text(json.dumps(
        {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in sources}, indent=2))
    cases = [
        ("nominal", [], .2),
        ("start_later", [], .7),
        ("wheel_20_35", ["--wheel-delay", ".02", "--wheel-lag", ".035"], .2),
        ("wheel_35_50", ["--wheel-delay", ".035", "--wheel-lag", ".05"], .2),
        ("wheel_60_90", ["--wheel-delay", ".06", "--wheel-lag", ".09"], .2),
        ("push_forward", ["--at", "4.35:PUSH 1.0 0.08"], .2),
        ("push_back", ["--at", "4.35:PUSH -1.0 0.08"], .2),
    ]
    variants = {}
    if args.extended:
        cases += [("start_early", [], .1), ("start_035", [], .35), ("start_12", [], 1.2)]
        for seed in (1, 7, 23, 101):
            cases.append((f"noise_{seed}", ["--sensor-noise", "1", "--noise-seed", str(seed)], .2))
        changes = dict(com_front=dict(com_dx=.002), com_back=dict(com_dx=-.002),
                       mass_light=dict(mass_scale=.95), mass_heavy=dict(mass_scale=1.05),
                       friction_low=dict(friction=.6), friction_high=dict(friction=1),
                       hip_plus=dict(hip_delta=.02), hip_minus=dict(hip_delta=-.02))
        for name, change in changes.items():
            variants[name] = model_variant(args, name, **change)
            cases.append((name, ["--allow-model-mismatch"], .7 if name.startswith('hip_') else .2))
    assert 0 <= args.random_cases <= 100, "random case count must be 0..100"
    rng = random.Random(3504)
    mixed = []
    for index in range(args.random_cases):
        name = f"mixed_{index:02d}"
        change = dict(com_dx=rng.uniform(-.002, .002), mass_scale=rng.uniform(.95, 1.05),
                      friction=rng.uniform(.6, 1.0), hip_delta=rng.uniform(-.02, .02))
        delay = rng.uniform(.02, .06); lag = rng.uniform(.035, .09)
        start = rng.uniform(.3, 1.2); noise_seed = rng.randrange(1, 1000000)
        variants[name] = model_variant(args, name, **change)
        extra = ["--allow-model-mismatch", "--wheel-delay", str(delay), "--wheel-lag", str(lag),
                 "--sensor-noise", "1", "--noise-seed", str(noise_seed)]
        cases.append((name, extra, start))
        mixed.append(dict(case=name, **change, delay_s=delay, lag_s=lag, start_s=start, noise_seed=noise_seed))
    if mixed:
        (args.evidence / 'perturbations.json').write_text(json.dumps(mixed, indent=2))
    if args.cases != "all":
        selected = set(args.cases.split(","))
        cases = [c for c in cases if c[0] in selected]
        assert len(cases) == len(selected), "unknown case"
    model = mujoco.MjModel.from_xml_path(str(args.model))
    results = []; failures = []
    with ThreadPoolExecutor(max_workers=2) as pool:
        pending = [(name, pool.submit(run, args, name,
                    extra + (["--standup", args.standup] if args.standup else []),
                    24, start, variants.get(name)))
                   for name, extra, start in cases]
        for name, future in pending:
            try:
                actual_model = mujoco.MjModel.from_xml_path(str(variants[name])) if name in variants else model
                result = score(actual_model, future.result())
                results.append(result)
                print(json.dumps(result), flush=True)
            except Exception as error:
                failures.append(dict(case=name, error=str(error)))
                print(json.dumps(failures[-1]), flush=True)
    # Save recovery evidence even if a subsequent control fails. Run controls
    # with the same candidate tuning, plus the explicit diagnostic override.
    controls = []
    def save_summary():
        (args.evidence / "summary.json").write_text(json.dumps(
            dict(passed=results, failed=failures, controls=controls), indent=2))
    save_summary()
    for name in ("hold_control", "estop"):
        try:
            if name == "hold_control":
                spec = ",".join(filter(None, (args.standup, "hold_after_flip=1")))
                _, held, _, _ = run(args, name, ["--standup", spec], 7)
                assert any(s["su_phase_name"] == "Flipped" for s in held)
                assert not any(s["mode"] == "Balancing" for s in held)
            else:
                extra = ["--standup", args.standup] if args.standup else []
                _, stopped, views, _ = run(args, name, ["--at", "4.35:ESTOP", *extra], 6)
                assert any(s.get("su_peak") == "1" for s in stopped), "ESTOP control never exercised peak capture"
                after = [s for s in stopped if float(s["t"]) >= 4.37]
                assert after and all(s["mode"] == "Idle" and s["wheel_cmd_l"] == "0"
                                    and s["wheel_cmd_r"] == "0" and s.get("su_peak") == "0" for s in after)
                assert float(views[-1]["physics_t"]) > 5.5, "ESTOP suspended physics"
            controls.append(dict(case=name, passed=True))
        except Exception as error:
            failures.append(dict(case=name, error=str(error)))
        save_summary()
    assert not failures, failures
    print(f"{len(results)} recovery scenarios + hold/ESTOP controls passed; evidence: {args.evidence}")


if __name__ == "__main__":
    main()
