#!/usr/bin/env python3
"""Native regression for the leg speed loop and the leg-command rate limit.

Two things are pinned here.

The leg speed loop: shifting the body horizontally over the contact is what
makes the robot accelerate, so the legs can own speed tracking while the wheel
controller owns balance.  It is driven by the speed ERROR, which is negative
feedback - body forward raises speed, which shrinks the error that moved it.
Driving the same geometry from measured acceleration inverts that into
positive feedback and the robot tips; that is not a tuning difference, so the
direction checks below are the guard against anyone rewiring it.

The leg-command rate limit: no path may step a leg position target.  A pending
height transaction that aborts used to drop straight back to the base pose - a
~0.6 rad knee jump in one sample - which tipped the robot when it happened at
speed.  The height-during-drive case reproduces exactly that.

Only the STATE stream is inspected.  VIEW/ground-truth pose is never used as
controller evidence.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import selectors
import subprocess
import time


def fields(line: str):
    return {part.split("=", 1)[0]: part.split("=", 1)[1]
            for part in line.split("|")[1:] if "=" in part}


def run_trial(runner: Path, model: Path, drive_m_s: float, seed: int,
              height_during_drive: bool, t_end: float = 11.0):
    """Drive at a constant command, re-sent at 50 Hz the way the operator GUI
    does, optionally requesting a height change mid-drive."""
    process = subprocess.Popen(
        [str(runner), "--model", str(model), "--rate", "3000",
         "--sensor-rate", "500", "--realtime", "--seed", str(seed)],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1)
    assert process.stdin is not None
    assert process.stdout is not None
    selector = selectors.DefaultSelector()
    selector.register(process.stdout, selectors.EVENT_READ)

    def send(command: str):
        try:
            process.stdin.write(command + "\n")
            process.stdin.flush()
        except OSError:
            pass

    states = []
    driving = 0.0
    last_sent = 0.0
    height_sent = False
    deadline = time.monotonic() + t_end + 25.0
    try:
        while time.monotonic() < deadline:
            if not selector.select(0.5):
                continue
            line = process.stdout.readline().rstrip("\n")
            if line.startswith("READY|"):
                send("ENABLE")
            elif line.startswith("STATE|"):
                state = fields(line)
                states.append(state)
                sim_time = float(state["t"])
                if state.get("mode") == "Balance" and sim_time > 3.0 \
                        and driving == 0.0:
                    driving = drive_m_s
                if height_during_drive and not height_sent and sim_time > 4.0:
                    send("HEIGHT 0.20")
                    height_sent = True
                if driving != 0.0 and sim_time - last_sent >= 0.02:
                    send(f"DRIVE {driving:.6g} 0")
                    last_sent = sim_time
                if sim_time >= t_end:
                    break
    finally:
        if process.poll() is None:
            try:
                process.stdin.close()
            except OSError:
                pass
            process.terminate()
            process.wait(timeout=5)
    assert driving != 0.0, "drive command was never issued"
    return states


def window(states, lo, hi):
    return [s for s in states if lo <= float(s["t"]) < hi]


def check(states, drive, label):
    assert states
    assert all(s["ground_truth_to_controller"] == "0" for s in states)
    peak = max(abs(float(s["pitch"])) for s in states)
    assert all(s["mode"] != "Fallen" for s in states), \
        f"{label}: fell (peak pitch {peak:.3f} rad)"
    # Generous, but far under the ~1.8 rad the stepped-leg-command bug reached.
    assert peak < 0.30, f"{label}: peak pitch {peak:.4f} rad"

    accel = window(states, 3.0, 4.5)
    assert accel
    # The leg shift must go the way the demand does. A sign flip here is the
    # signature of the loop having been rewired to positive feedback.
    shift = (max(float(s["wb_speed_hip"]) for s in accel) if drive > 0
             else min(float(s["wb_speed_hip"]) for s in accel))
    assert abs(shift) > 0.004 and (shift > 0) == (drive > 0), \
        f"{label}: leg speed loop not engaging with the demand: {shift}"

    # Live-pose inertial lead, not a frozen constant.
    lead = float(accel[-1]["wb_speed_lead"])
    assert 0.02 < lead < 0.50, f"{label}: implausible inertial lead {lead} s"

    # The robot must actually travel. A near-fall that recovers can leave peak
    # pitch inside bounds while the robot goes nowhere, so speed is checked
    # independently of attitude.
    # The leg-command rate limit is sized so it never binds in correct
    # operation, which makes it a detector: if it clipped, something upstream
    # emitted a leg-target discontinuity. That is how the stale pre-solved
    # height endpoint used to reach the actuators, and asserting on the
    # outcome alone would not catch it - the limit would simply absorb it.
    clipped = max(int(s["leg_cmd_rate_limited"]) for s in states)
    assert clipped == 0, \
        f"{label}: leg command rate limit clipped on {clipped} ticks - " \
        "an upstream path stepped the leg target"

    cruise = window(states, 8.0, 11.0)
    assert cruise
    mean_v = sum(float(s["wheel_velocity"]) for s in cruise) / len(cruise)
    assert (mean_v > 0) == (drive > 0) and abs(mean_v) > 0.5 * abs(drive), \
        f"{label}: commanded {drive} m/s but cruised at {mean_v:.4f} m/s"
    return peak, mean_v


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--runner", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    args = parser.parse_args()

    results = []
    for drive in (1.0, -1.0):
        for height in (False, True):
            label = ("forward" if drive > 0 else "reverse") + \
                    (" +height" if height else "")
            states = run_trial(args.runner, args.model, drive, 6814, height)
            results.append((label,) + check(states, drive, label))

    for label, peak, mean_v in results:
        print(f"  {label:18s} peak={peak:.4f} rad  cruise={mean_v:+.3f} m/s")
    print("PASS native leg speed loop + leg-command rate limit")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
