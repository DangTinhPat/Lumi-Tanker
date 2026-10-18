#!/usr/bin/env python3
"""Assert GIM hip response during speed changes, not only final station hold."""
import argparse
import json
from pathlib import Path

import numpy as np

from gim_crouch_test import run, stable


def responsive(states, start, sign):
    baseline = min(states, key=lambda s: abs(float(s["t"])-(start-.02)))
    event = [s for s in states if start <= float(s["t"]) <= start+.08]
    request = next((s for s in event if sign*(float(s["hip_trunk_request"])-
                                             float(s["hip_level"])) > .01), None)
    assert request is not None, "hip preview took more than 80 ms"
    for side in ("joint_lh", "joint_rh"):
        moved = next((s for s in event if sign*(float(s[side])-float(baseline[side])) > .008), None)
        assert moved is not None, "physical hip response took more than 80 ms"
        visible = [s for s in event if float(s["t"]) <= start+.061 and
                   sign*(float(s[side])-float(baseline[side])) > np.pi/180]
        assert visible, "physical hip did not move one degree within 60 ms (50 Hz telemetry)"
    return round(float(request["t"])-start, 4)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--runner", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--evidence", type=Path, required=True)
    args = parser.parse_args()
    args.evidence.mkdir(parents=True, exist_ok=True)
    counter, _, _ = run(args, "virtual_hip_lead", 45,
                        [(6, "HEIGHT .08"), (35, "DRIVE .3 0")],
                        ("--virtual-hip-lead",))
    assert all(s["hip_lead_gyro"] == "0" for s in counter)
    # Its fall depends on actuator/contact phase. The deterministic unit
    # countercase separately proves feedback from encoder rate into hip lead.
    schedule = [(35, "DRIVE .3 0"), (42, "DRIVE .5 0"), (49, "DRIVE .1 0"), (56, "STOP"),
                (64, "DRIVE -.3 0"), (71, "DRIVE -.5 0"), (78, "DRIVE -.1 0"), (85, "STOP"),
                (93, "PUSH 2 .2"), (99, "PUSH -2 .2")]
    for height in (.08, .13, .20):
        states, views, metrics = run(args, f"maneuver_{height:.2f}", 135,
                                    [(6, f"HEIGHT {height}"), *schedule])
        stable(states, views, metrics, height=height)
        for start, sign in ((35, 1), (42, 1), (49, -1), (56, -1),
                            (64, -1), (71, -1), (78, 1), (85, 1)):
            responsive(states, start, sign)
            event = [s for s in states if start <= float(s["t"]) < start+3]
            assert max(sign*float(s["plan_acceleration"]) for s in event) > .2
            dynamic = [sign*(float(s["hip_trunk_request"])-float(s["hip_level"])) for s in event]
            assert max(dynamic) > .01, "hip preview did not answer signed acceleration"
            for side in ("joint_lh", "joint_rh"):
                q = [float(s[side]) for s in event]
                assert max(q)-min(q) > .008, "encoder showed no physical hip response"
            assert max(float(s["reflex_gate"]) for s in event) > .1, "inertial gate did not open"
        for when, speed in ((39, .3), (46, .5), (53, .1), (68, -.3), (75, -.5), (82, -.1)):
            cruise = [float(s["ground_velocity"]) for s in states if when <= float(s["t"]) < when+1]
            assert abs(sum(cruise)/len(cruise)-speed) < .05, "speed change did not reach the robot"
        hf = {}
        for key, bound in (("pitch_rate", .08), ("hip_target", .005)):
            signal = np.array([float(s[key]) for s in states if 35 <= float(s["t"]) < 85])
            spectrum = np.fft.rfft(signal-signal.mean())
            spectrum[np.fft.rfftfreq(len(signal), .02) <= 5] = 0
            hf[key] = float(np.sqrt(np.mean(np.fft.irfft(spectrum, n=len(signal))**2)))
            assert hf[key] < bound, "fast hip profile increased vibration above the bound"
        print(json.dumps(dict(case=f"maneuver_{height:.2f}", signed_hip_events=8,
                              cruise_windows=6, settled_gate=metrics["quiet_gate"],
                              maneuver_hf_rms=hf)), flush=True)

    presses = ((35, .2, .3), (42, .2, -.3), (49, .8, .3), (57, .8, -.3))
    button_schedule = [(6, "HEIGHT .08")]
    for start, duration, speed in presses:
        button_schedule.extend(((start, f"DRIVE {speed} 0"), (start+duration, "STOP")))
    states, views, metrics = run(args, "button_taps", 100, button_schedule)
    stable(states, views, metrics)
    for start, _, speed in presses:
        event = [s for s in states if start <= float(s["t"]) < start+3]
        sign = 1 if speed > 0 else -1
        responsive(states, start, sign)
        assert max(sign*float(s["plan_acceleration"]) for s in event) > .15
        assert max(sign*(float(s["hip_trunk_request"])-float(s["hip_level"]))
                   for s in event) > .005, "short button press had no hip preview"
        for side in ("joint_lh", "joint_rh"):
            q = [float(s[side]) for s in event]
            assert max(q)-min(q) > .01, "short button press had no physical hip response"
        assert max(float(s["reflex_gate"]) for s in event) > .1
    print(json.dumps(dict(case="button_taps", signed_hip_events=4,
                          settled_gate=metrics["quiet_gate"])), flush=True)


if __name__ == "__main__":
    main()
