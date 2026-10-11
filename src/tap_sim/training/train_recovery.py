#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import math
import subprocess
from datetime import datetime, timezone
from pathlib import Path

import gymnasium
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv
from stable_baselines3.common.utils import get_schedule_fn

from tap_recovery import GimRecoveryEnv
from tap_recovery.env import MODEL_PATH


ROOT = Path(__file__).resolve().parents[3]


def make_env(seed: int, randomize_physics: bool, reward_stage: str,
             log_file: str | None = None):
    def build():
        env = GimRecoveryEnv(randomize_physics=randomize_physics,
                             reward_stage=reward_stage)
        env = Monitor(env, filename=log_file,
                      info_keywords=("body_lifted", "stood_up", "balance_activated", "is_success",
                                     "recovery_time_s", "balance_activation_time_s",
                                     "balance_stable_s", "sim_time_s",
                                     "body_clearance_m", "mean_body_ground_load_fraction",
                                     "mean_abs_com_axle_offset_m", "mean_abs_leg_current_a",
                                     "wheel_only_contact_fraction", "nonwheel_contact_fraction",
                                     "balance_requested", "balance_request_wheels_only"))
        env.reset(seed=seed)
        return env
    return build


def main():
    parser = argparse.ArgumentParser(description="Train the GIM MuJoCo recovery policy with PPO.")
    parser.add_argument("--timesteps", type=int, default=2_000_000)
    parser.add_argument("--envs", type=int, default=2)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--stage", choices=("discovery", "deployable"),
                        default="discovery",
                        help="Discovery finds a get-up motion; deployable fine-tunes it smoothly")
    parser.add_argument("--ent-coef", type=float,
                        help="Defaults to 0.01 for discovery and 0.002 for deployable")
    parser.add_argument("--learning-rate", type=float,
                        help="Defaults to 3e-4 for discovery and 5e-5 for deployable")
    parser.add_argument("--resume", type=Path,
                        help="Fine-tune an existing PPO policy with the current reward")
    parser.add_argument("--randomize-physics", action="store_true")
    args = parser.parse_args()
    if args.ent_coef is None:
        args.ent_coef = 0.002 if args.stage == "deployable" else 0.01
    if args.learning_rate is None:
        args.learning_rate = 5e-5 if args.stage == "deployable" else 3e-4
    if (args.timesteps <= 0 or args.envs <= 0 or not math.isfinite(args.ent_coef)
            or args.ent_coef < 0 or not math.isfinite(args.learning_rate)
            or args.learning_rate <= 0):
        parser.error("timesteps/envs must be positive; entropy coefficient must be nonnegative; "
                     "learning rate must be positive")
    if args.stage == "deployable" and not args.resume:
        parser.error("--stage deployable requires --resume with a discovery checkpoint")
    torch.set_num_threads(1)

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    output = (args.output or ROOT / "training_runs" / "recovery" / stamp).resolve()
    output.mkdir(parents=True, exist_ok=False)
    n_envs = args.envs
    env_fns = [make_env(args.seed + i, args.randomize_physics, args.stage,
                        str(output / f"monitor_{i}.csv")) for i in range(n_envs)]
    train_env = DummyVecEnv(env_fns) if n_envs == 1 else SubprocVecEnv(
        env_fns, start_method="spawn")
    eval_env = DummyVecEnv([make_env(args.seed + 100_000,
                                     args.randomize_physics, args.stage)])
    eval_callback = EvalCallback(
        eval_env, best_model_save_path=str(output / "best"),
        log_path=str(output / "evaluation"), eval_freq=max(10_000 // n_envs, 1),
        n_eval_episodes=10, deterministic=True, render=False,
    )
    checkpoints = CheckpointCallback(
        save_freq=max(50_000 // n_envs, 1), save_path=str(output / "checkpoints"),
        name_prefix="recovery",
    )
    device = "cuda" if torch.cuda.is_available() else "cpu"
    if args.resume:
        model = PPO.load(args.resume, env=train_env, device=device,
                         tensorboard_log=str(output / "tensorboard"))
        model.ent_coef = args.ent_coef
        model.learning_rate = args.learning_rate
        model.lr_schedule = get_schedule_fn(args.learning_rate)
    else:
        model = PPO(
            "MlpPolicy", train_env, learning_rate=args.learning_rate, n_steps=1024,
            batch_size=256, n_epochs=10, gamma=0.99, gae_lambda=0.95,
            clip_range=0.2, ent_coef=args.ent_coef,
            policy_kwargs={"net_arch": {"pi": [128, 128], "vf": [128, 128]}},
            tensorboard_log=str(output / "tensorboard"), seed=args.seed,
            device=device, verbose=1,
        )
    model.learn(total_timesteps=args.timesteps,
                callback=[eval_callback, checkpoints], progress_bar=True,
                reset_num_timesteps=not bool(args.resume))
    model.save(output / "recovery_policy")

    model_path = MODEL_PATH
    git_status = subprocess.run(["git", "-C", str(ROOT), "status", "--porcelain"],
                                capture_output=True, text=True).stdout
    training_sources = (Path(__file__), Path(__file__).with_name("evaluate_recovery.py"),
                        Path(__file__).parent / "tap_recovery" / "env.py",
                        Path(__file__).parent / "tap_recovery" / "v3_balance.py",
                        Path(__file__).parent / "v3_balance_bridge.cpp",
                        ROOT / "src" / "tap_sim" / "CMakeLists.txt")
    metadata = {
        "branch": subprocess.run(["git", "-C", str(ROOT), "branch", "--show-current"],
                                  capture_output=True, text=True).stdout.strip(),
        "commit": subprocess.run(["git", "-C", str(ROOT), "rev-parse", "HEAD"],
                                  capture_output=True, text=True).stdout.strip(),
        "git_dirty": bool(git_status),
        "model": str(model_path.relative_to(ROOT)),
        "model_sha256": hashlib.sha256(model_path.read_bytes()).hexdigest(),
        "training_sources_sha256": {
            str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in training_sources
        },
        "seed": args.seed,
        "envs": args.envs,
        "timesteps": args.timesteps,
        "stage": args.stage,
        "ent_coef": args.ent_coef,
        "learning_rate": args.learning_rate,
        "randomize_physics": args.randomize_physics,
        "resumed_from": str(args.resume.resolve()) if args.resume else None,
        "python": __import__("sys").version,
        "torch": torch.__version__,
        "gymnasium": gymnasium.__version__,
        "mujoco": __import__("mujoco").__version__,
        "cuda_available": torch.cuda.is_available(),
        "gpu": torch.cuda.get_device_name(0) if torch.cuda.is_available() else None,
    }
    (output / "run.json").write_text(json.dumps(metadata, indent=2) + "\n")
    train_env.close()
    eval_env.close()
    print(f"Saved policy and run metadata under {output}")


if __name__ == "__main__":
    main()
