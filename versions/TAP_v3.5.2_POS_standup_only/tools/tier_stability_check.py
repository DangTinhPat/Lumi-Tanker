#!/usr/bin/env python3
"""Run the existing 2-D tier plant and save repeatable evidence + playback.

  python3 tools/tier_stability_check.py --binary build/bin/v3_tier_tests \
      --ctest-build build --baseline /path/to/before-tier

3 N presses are the acceptance suite; 6 N is a separately reported stress
suite (known wheel saturation at low stance). This tool never opens serial.
"""
from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[1]
PRESS = re.compile(
    r"PRESS ([\d.]+) N hold ([\d.]+) h\s+([\d.]+): falls (\d+) held (\d+)"
    r" \| body worst ([\d.]+) mean ([\d.]+) \| dhip ([\d.]+)"
    r" \| roll-away ([\d.]+) m \| rest ([\d.]+)")


def fingerprint(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--baseline", type=Path)
    parser.add_argument("--ctest-build", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    out = (args.output or ROOT / "log" / dt.datetime.now().strftime("tier_stability_%Y%m%d_%H%M%S")).resolve()
    out.mkdir(parents=True, exist_ok=True)
    env = {k: v for k, v in os.environ.items()
           if not k.startswith(("TIER_", "TR_", "ANIM_", "PRESS_", "WB_", "ROUGH_", "RW_", "RS_", "GV_"))}
    results: dict = {"sensor_contract": "MPU6500 and six encoders; no hardware I/O",
                     "acceptance": {"force_n": 3, "variants_per_height": 7,
                                    "heights_mm": [189, 136, 110, 84],
                                    "required_falls": 0, "body_peak_rad_max": 0.15},
                     "stress": {"force_n": 6, "acceptance_gate": False}, "runs": {}}

    def run(label: str, command: list[str], settings: dict[str, str]) -> str:
        print(label, flush=True)
        cp = subprocess.run(command, cwd=ROOT, env=env | settings,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        (out / f"{label}.log").write_text(cp.stdout)
        if cp.returncode:
            raise RuntimeError(f"{label}: exit {cp.returncode}; see {out / (label + '.log')}")
        return cp.stdout

    sources = [str(p.relative_to(ROOT)) for folder in
               ("src_fw", "src/tap_control/core", "src/tap_control/hal")
               for p in sorted((ROOT / folder).rglob("*"))
               if p.is_file() and p.suffix in (".cpp", ".hpp", ".h", ".txt")]
    sources += ["CMakeLists.txt", "tools/tier_stability_check.py", "tools/tier_sim_view.py"]
    results["source_sha256"] = {p: fingerprint(ROOT / p) for p in sources}
    # Include newly added/untracked test sources; git diff alone omits them.
    for p in sources:
        snapshot = out / "source" / p
        snapshot.parent.mkdir(parents=True, exist_ok=True)
        snapshot.write_bytes((ROOT / p).read_bytes())
    results["git_head"] = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    (out / "working-tree.patch").write_bytes(subprocess.check_output(["git", "diff", "--binary"], cwd=ROOT))
    if args.ctest_build:
        log = run("ctest", ["ctest", "--test-dir", str(args.ctest_build.resolve()), "--output-on-failure",
                            "-R", "^(v3_tests|v3_physics_tests|v3_tier_tests|v3_coordination_tests|leg_height_tests)$"], {})
        results["ctest_passed"] = "100% tests passed" in log
    # Standard sagittal regression keeps roll off; also check AppV3's boot
    # configuration (roll on, velocity FF off) on the lateral proxy.
    for label, path in [("before", args.baseline), ("after", args.binary),
                        ("after_boot", args.binary)]:
        if path is None:
            continue
        path = path.resolve()
        original = path
        snapshot = out / "binaries" / f"{label}-tier"
        snapshot.parent.mkdir(parents=True, exist_ok=True)
        snapshot.write_bytes(path.read_bytes())
        snapshot.chmod(0o755)
        path = snapshot
        entry = {"binary": str(path), "original_binary": str(original),
                 "sha256": fingerprint(path), "press": {},
                 "roll_enabled": label == "after_boot"}
        configuration = {"TR_ROLL": "1"} if label == "after_boot" else {}
        for force in (3, 6):
            log = run(f"{label}_{force}n", [str(path)],
                      configuration | {"TIER_PRESS": "1", "PRESS_F": str(force), "PRESS_HOLD": "0.5"})
            rows = [{"height_mm": float(m[2]), "falls": int(m[3]), "held": int(m[4]),
                     "body_peak_rad": float(m[5]), "body_mean_rad": float(m[6]),
                     "hip_peak_rad": float(m[7]), "travel_peak_m": float(m[8]),
                     "last4s_rate_rms_rad_s": float(m[9])} for m in PRESS.findall(log)]
            if len(rows) != 4:
                raise RuntimeError(f"{label}_{force}n: expected four height summaries, got {len(rows)}")
            entry["press"][str(force)] = rows
        run(f"{label}_animation", [str(path)],
            configuration | {"TIER_ANIM": "1", "ANIM_H": "0.084", "ANIM_PRESS": "3", "ANIM_HOLD": "0.5",
             "TR_ANIM": str(out / f"{label}.csv")})
        results["runs"][label] = entry
    accepted = all(r["falls"] == 0 and r["body_peak_rad"] <= 0.15
                   for label in ("after", "after_boot")
                   for r in results["runs"][label]["press"]["3"])
    results["accepted"] = accepted
    results["limitations"] = ["Planar LegPlant plus approximate lateral roll proxy, not full 3-D physics",
                              "Motor knee model and limits are provisional; no hardware balance claim",
                              "6 N stress failures remain recorded even when acceptance passes"]
    (out / "summary.json").write_text(json.dumps(results, indent=2) + "\n")
    print(f"Acceptance: {'PASS' if accepted else 'FAIL'}; evidence: {out}", flush=True)
    return 0 if accepted else 1


if __name__ == "__main__":
    raise SystemExit(main())
