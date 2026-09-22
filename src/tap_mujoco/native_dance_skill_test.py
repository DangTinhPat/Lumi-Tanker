#!/usr/bin/env python3
"""Closed-loop regression for the native rotate-and-height dance skill."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import selectors
import subprocess


def fields(line: str):
    return {part.split("=", 1)[0]: part.split("=", 1)[1]
            for part in line.split("|")[1:] if "=" in part}


def yaw_from_view(view):
    qw, qx, qy, qz = (float(view[key]) for key in ("qw", "qx", "qy", "qz"))
    return math.atan2(2.0 * (qw * qz + qx * qy),
                     1.0 - 2.0 * (qy * qy + qz * qz))


def wrapped_delta(previous: float, current: float) -> float:
    return math.atan2(math.sin(current - previous),
                      math.cos(current - previous))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--runner", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--duration-s", type=float, default=28.0)
    args = parser.parse_args()

    process = subprocess.Popen(
        [str(args.runner), "--model", str(args.model), "--rate", "3000",
         "--sensor-rate", "500", "--seed", "6814"],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, bufsize=1)
    assert process.stdin is not None and process.stdout is not None
    selector = selectors.DefaultSelector()
    selector.register(process.stdout, selectors.EVENT_READ)

    def send(command: str) -> None:
        process.stdin.write(command + "\n")
        process.stdin.flush()

    states = []
    views = []
    status_events = []
    last_status = None
    dance_start = None
    dance_stop = None
    try:
        while True:
            events = selector.select(1.0)
            if not events:
                continue
            line = process.stdout.readline().rstrip("\n")
            if not line:
                continue
            if line.startswith("READY|"):
                send("ENABLE")
            elif line.startswith("STATE|"):
                state = fields(line)
                states.append(state)
                sim_time = float(state["t"])
                status = tuple(state.get(key) for key in (
                    "mode", "height_target", "height_motion", "station_hold",
                    "station_eligible", "support_contact"))
                if status != last_status:
                    status_events.append({"t": state["t"], **dict(zip(
                        ("mode", "height_target", "height_motion", "station_hold",
                         "station_eligible", "support_contact"), status))})
                    last_status = status
                if (dance_start is None and state.get("mode") == "Balance" and
                        sim_time > 2.0):
                    send("DANCE START 1")
                    dance_start = sim_time
                elif (dance_start is not None and dance_stop is None and
                      sim_time >= dance_start + args.duration_s):
                    send("DANCE STOP")
                    dance_stop = sim_time
                if dance_stop is not None and sim_time >= dance_stop + 1.0:
                    break
                if dance_start is not None and state.get("mode") == "Fallen":
                    recent = [
                        {key: item.get(key) for key in (
                            "t", "mode", "pitch", "pitch_rate", "lqr_pitch_ref",
                            "balance_pitch_ref", "lqr_accel", "wheel_position",
                            "wheel_velocity", "lqr_velocity", "lqr_anchor", "station_hold",
                            "station_capture_distance", "station_eligible",
                            "support_contact", "support_unloading", "support_loss_s",
                            "flight", "airborne", "settling", "free_fall_confidence",
                            "height_target", "height_motion", "dance_height_target")}
                        for item in states[-12:]]
                    raise AssertionError(
                        f"dance fell at t={sim_time:.3f}s: {state}; "
                        f"recent={recent}; events={status_events[-40:]}")
            elif line.startswith("VIEW|"):
                views.append(fields(line))
    finally:
        if process.poll() is None:
            try:
                send("QUIT")
            except BrokenPipeError:
                pass
            process.wait(timeout=5.0)
        selector.close()

    if dance_start is None or dance_stop is None:
        raise AssertionError("dance command was not exercised")
    # The STOP line is read between control ticks; exclude that handoff edge
    # (and the START line's pre-command telemetry) from the assertions.
    active_end = dance_stop - 0.05
    dance_states = [state for state in states
                    if dance_start + 0.05 <= float(state["t"]) <= active_end]
    if not dance_states:
        raise AssertionError("no dance telemetry")
    if not all(state.get("skill") == "dance" for state in dance_states):
        raise AssertionError("dance skill retired before DANCE STOP")
    if not all(state.get("station_hold") == "1" and
               state.get("station_eligible") == "1"
               for state in dance_states):
        raise AssertionError("dance released the translation station anchor")
    max_pitch_state = max(dance_states,
                          key=lambda state: abs(float(state["pitch"])))
    if abs(float(max_pitch_state["pitch"])) >= 0.20:
        raise AssertionError(
            f"dance exceeded the pitch stability envelope: {max_pitch_state}")
    targets = {round(float(state["dance_height_target"]), 3)
               for state in dance_states}
    if not {0.155, 0.200}.issubset(targets):
        raise AssertionError(f"dance did not visit both height targets: {targets}")
    if not any(abs(float(state["dance_yaw_rate"])) > 0.15
               for state in dance_states):
        raise AssertionError("dance never commanded a meaningful yaw rate")
    if not any(state.get("height_motion") == "0" and
               state.get("height_request") == "0"
               for state in dance_states):
        raise AssertionError("dance never retired a height transaction")
    if max(abs(float(state["lqr_accel"])) for state in dance_states) > 1.41:
        raise AssertionError("dance planned acceleration beyond wheel torque envelope")

    score_views = [view for view in views
                   if dance_start + 0.2 <= float(view["t"]) <= active_end]
    if len(score_views) < 10:
        raise AssertionError("insufficient VIEW samples for dance scoring")
    x0, y0 = float(score_views[0]["x"]), float(score_views[0]["y"])
    drift = max(math.hypot(float(view["x"]) - x0,
                           float(view["y"]) - y0)
                for view in score_views)
    accumulated_yaw = 0.0
    previous_yaw = yaw_from_view(score_views[0])
    for view in score_views[1:]:
        current_yaw = yaw_from_view(view)
        accumulated_yaw += wrapped_delta(previous_yaw, current_yaw)
        previous_yaw = current_yaw
    if drift >= 0.12:
        raise AssertionError(f"dance translated {drift:.3f} m")
    if abs(accumulated_yaw) <= 1.5:
        raise AssertionError(
            f"dance rotated only {accumulated_yaw:.3f} rad")

    print("PASS native dance skill: no fall, both heights, "
          f"yaw={accumulated_yaw:.3f} rad drift={drift:.3f} m")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
