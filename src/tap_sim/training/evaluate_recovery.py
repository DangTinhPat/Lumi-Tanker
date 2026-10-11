#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

from stable_baselines3 import PPO

from tap_recovery import GimRecoveryEnv
def main():
    parser = argparse.ArgumentParser(description="Evaluate a saved GIM recovery policy.")
    parser.add_argument("policy", type=Path)
    parser.add_argument("--episodes", type=int, default=20)
    parser.add_argument("--seed", type=int, default=100_000)
    parser.add_argument("--randomize-physics", action="store_true")
    args = parser.parse_args()
    env = GimRecoveryEnv(randomize_physics=args.randomize_physics)
    model = PPO.load(args.policy)
    results = []
    if args.episodes <= 0:
        parser.error("--episodes must be positive")
    for episode in range(args.episodes):
        obs, _ = env.reset(seed=args.seed + episode)
        while True:
            action, _ = model.predict(obs, deterministic=True)
            obs, _, terminated, truncated, info = env.step(action)
            if terminated or truncated:
                results.append(info)
                break
    stood_up = sum(row["stood_up"] for row in results)
    body_lifted = sum(row["body_lifted"] for row in results)
    balance_on = sum(row["balance_activated"] for row in results)
    success = sum(row["is_success"] for row in results)
    stand_times = [row["recovery_time_s"] for row in results
                   if row["recovery_time_s"] >= 0.0]
    balance_times = [row["balance_activation_time_s"] for row in results
                     if row["balance_activation_time_s"] >= 0.0]
    mean_stand = sum(stand_times) / len(stand_times) if stand_times else float("nan")
    mean_balance = (sum(balance_times) / len(balance_times)
                    if balance_times else float("nan"))
    mean_current = sum(row["mean_abs_leg_current_a"] for row in results) / len(results)
    mean_clearance = sum(row["body_clearance_m"] for row in results) / len(results)
    mean_body_load = sum(row["mean_body_ground_load_fraction"] for row in results) / len(results)
    mean_com_offset = sum(row["mean_abs_com_axle_offset_m"] for row in results) / len(results)
    mean_wheel_only = sum(row["wheel_only_contact_fraction"] for row in results) / len(results)
    mean_nonwheel = sum(row["nonwheel_contact_fraction"] for row in results) / len(results)
    requests = [row for row in results if row["balance_requested"]]
    wheel_only_requests = sum(row["balance_request_wheels_only"] for row in requests)
    request_wheel_only = (wheel_only_requests /
                          len(requests) if requests else float("nan"))
    print(f"GIM home recovery | {len(results)} episodes")
    print(f"  Body fully lifted: {body_lifted}/{len(results)} "
          "(all body shells >2 cm clear, upright, both wheels supported)")
    print(f"  Stood up:          {stood_up}/{len(results)}")
    print(f"  V3 balance active: {balance_on}/{len(results)}")
    print(f"  Success (held):    {success}/{len(results)} "
          f"(V3 balance stable for {env.balance_hold_steps / env.control_hz:.1f}s)")
    print(f"  Mean time to stand: {mean_stand:.2f}s")
    print(f"  Mean time to V3:    {mean_balance:.2f}s")
    print(f"  Mean leg current:   {mean_current:.2f}A")
    print(f"  Mean shell clearance: {mean_clearance:.3f}m")
    print(f"  Mean body floor load: {mean_body_load:.2f}x robot weight")
    print(f"  Mean |COM-axle offset|: {mean_com_offset:.3f}m")
    print(f"  Wheel-only contact: {mean_wheel_only:.1%} of steps")
    print(f"  Other-part contact: {mean_nonwheel:.1%} of steps")
    print(f"  V3 requests at wheel-only state: {request_wheel_only:.1%}"
          f" ({wheel_only_requests}/{len(requests)} requested episodes)")
    env.close()


if __name__ == "__main__":
    main()
