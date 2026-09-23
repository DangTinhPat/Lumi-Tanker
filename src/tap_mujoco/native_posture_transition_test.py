#!/usr/bin/env python3
"""Closed-loop regression for chained native height transactions.

The test deliberately sends the next HEIGHT command on the first telemetry
sample after the previous transaction retires.  That is the operator race that
used to bypass the smooth path and make deep -> squat fall.  VIEW is collected
only for offline drift scoring; it is never sent to the controller.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import selectors
import subprocess


NATURAL = 0.1484350375
SQUAT_25 = 0.1463262781
DEEP = 0.140
MAX_TRANSITION_DRIFT_M = 0.020


def fields(line: str):
    return {part.split("=", 1)[0]: part.split("=", 1)[1]
            for part in line.split("|")[1:] if "=" in part}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--runner", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    args = parser.parse_args()

    process = subprocess.Popen(
        [str(args.runner), "--model", str(args.model), "--rate", "3000",
         "--sensor-rate", "500", "--seed", "6814", "--realtime"],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, bufsize=1)
    assert process.stdin is not None and process.stdout is not None
    selector = selectors.DefaultSelector()
    selector.register(process.stdout, selectors.EVENT_READ)

    def send(command: str) -> None:
        process.stdin.write(command + "\n")
        process.stdin.flush()

    targets = [SQUAT_25, DEEP, SQUAT_25, NATURAL]
    next_target = 0
    active_start = None
    motion_start = None
    active_target = None
    request_seen = False
    transitions = []
    states = []
    views = []
    try:
        while next_target < len(targets):
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
                if (active_target is None and next_target == 0 and
                        state.get("mode") == "Balance" and
                        state.get("station_hold") == "1"):
                    active_target = targets[next_target]
                    active_start = sim_time
                    request_seen = False
                    send(f"HEIGHT {active_target:.9f}")
                elif active_target is not None:
                    request_seen = request_seen or state.get("height_request") == "1"
                    if (state.get("height_motion") == "1" and
                            motion_start is None):
                        motion_start = sim_time
                    retired = (state.get("height_motion") == "0" and
                               state.get("height_request") == "0" and
                               request_seen and
                               sim_time > active_start)
                    if retired:
                        transitions.append((motion_start or active_start, sim_time,
                                            active_target, motion_start is not None))
                        next_target += 1
                        active_target = None
                        active_start = None
                        motion_start = None
                        if next_target < len(targets):
                            # The command is intentionally issued immediately
                            # on this retirement edge, without a manual pause.
                            active_target = targets[next_target]
                            active_start = sim_time
                            request_seen = False
                            send(f"HEIGHT {active_target:.9f}")
                if state.get("mode") == "Fallen":
                    raise AssertionError(
                        f"Fallen during posture chain at t={sim_time:.3f}s")
            elif line.startswith("VIEW|"):
                views.append(fields(line))
    finally:
        if process.poll() is None:
            try:
                send("QUIT")
            except BrokenPipeError:
                pass
            process.wait(timeout=3.0)
        selector.close()

    if len(transitions) != len(targets):
        raise AssertionError(f"only retired {len(transitions)}/{len(targets)} HEIGHT transactions")
    if not states:
        raise AssertionError("native runner produced no STATE telemetry")

    # Score each transition using the simulator pose only after the run.  The
    # controller saw only delayed/noisy IMU and encoder packets.
    failures = []
    reports = []
    for start, end, target, had_motion in transitions:
        if not had_motion:
            # A request already satisfied at the measured task boundary is a
            # valid no-op transaction.  It owns no leg trajectory, so do not
            # charge pre-acceptance station settling drift to the no-op.
            endpoint_states = [state for state in states
                               if end - 0.05 <= float(state["t"]) <= end + 0.25]
            if not endpoint_states:
                raise AssertionError("missing no-op HEIGHT telemetry")
            endpoint_height = float(endpoint_states[-1]["body_height"])
            if abs(endpoint_height - target) > 0.0025:
                failures.append(
                    f"HEIGHT {target:.6f} no-op ended at {endpoint_height:.6f} m")
            continue
        window = [view for view in views if start <= float(view["t"]) <= end + 1.0]
        if len(window) < 2:
            raise AssertionError("insufficient VIEW samples for posture scoring")
        x0 = float(window[0]["x"])
        max_view = max(window, key=lambda view: abs(float(view["x"]) - x0))
        drift = abs(float(max_view["x"]) - x0)
        max_state = min(states, key=lambda state: abs(
            float(state["t"]) - float(max_view["t"])))
        reports.append((target, drift, max_view["t"], x0, max_view["x"],
                        max_state))
        if drift >= MAX_TRANSITION_DRIFT_M:
            failures.append(
                f"HEIGHT {target:.6f} drifted {drift:.6f} m")
        endpoint_states = [state for state in states
                           if end - 0.05 <= float(state["t"]) <= end + 0.25]
        if not endpoint_states:
            raise AssertionError("missing HEIGHT endpoint telemetry")
        if any(state.get("height_rejected") == "1" for state in endpoint_states):
            raise AssertionError(f"HEIGHT {target:.6f} was rejected/aborted")
        endpoint_height = float(endpoint_states[-1]["body_height"])
        if abs(endpoint_height - target) > 0.0025:
            failures.append(
                f"HEIGHT {target:.6f} ended at {endpoint_height:.6f} m")

    if failures:
        summary = ", ".join(
            f"target={target:.6f}: drift={drift:.6f}m at t={time} "
            f"({x0}->{x}), wheel={state.get('wheel_position')} "
            f"wheel_v={state.get('wheel_velocity')} "
            f"anchor={state.get('lqr_anchor')} station={state.get('station_velocity_target')} "
                        f"lqr={state.get('lqr_velocity')} ref={state.get('lqr_velocity_reference')} "
                        f"accel={state.get('lqr_accel')} blend={state.get('lqr_station_gain_blend')} "
                        f"hold={state.get('station_hold')} pitch={state.get('pitch')} "
                        f"pitch_ref={state.get('lqr_pitch_ref')} "
                        f"pitch_state={state.get('lqr_pitch_state')} "
                        f"cmd_lw={state.get('cmd_lw')} cmd_rw={state.get('cmd_rw')} "
                        f"speed_hip={state.get('wb_speed_hip')} speed_com={state.get('wb_speed_com')} "
                        f"vmc={state.get('wb_vmc_pitch_torque')} "
                        f"trim={state.get('wb_trim')} com={state.get('wb_com_x')} "
                        f"comline={state.get('wb_com_line_angle')} "
                        f"joints=({state.get('joint_lh')},{state.get('joint_lk')},"
                        f"{state.get('joint_rh')},{state.get('joint_rk')}) "
                        f"joint_v=({state.get('joint_v_lh')},{state.get('joint_v_lk')},"
                        f"{state.get('joint_v_rh')},{state.get('joint_v_rk')})"
            for target, drift, time, x0, x, state in reports)
        raise AssertionError(
            "; ".join(failures) + f"; all transitions: {summary}")

    scored_states = [state for state in states if float(state["t"]) >= transitions[0][1]]
    if max(abs(float(state["pitch"])) for state in scored_states) >= 0.25:
        raise AssertionError("posture chain exceeded the pitch stability envelope")

    print("PASS native posture chain: natural/25%/deep/25%/natural "
          f"({len(transitions)} immediate HEIGHT handoffs)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
