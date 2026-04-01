# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

"""
csv_play.py – Load a trained RSL-RL model and drive the simulation by
              injecting pre-recorded observations (and/or actions) from
              CSV files into the model.

Observation CSV format (--obs_csv)
-----------------------------------
One row per simulation step, 78 columns in this exact order:

  cols  0- 2  base_ang_vel        (3)  angular velocity in body frame
                                        stored AFTER scale × 0.25
  cols  3- 5  projected_gravity   (3)  gravity vector projected in body frame
                                        stored AFTER scale × 1.0
  cols  6- 8  velocity_commands   (3)  [vx_cmd, vy_cmd, wz_cmd]
                                        stored AFTER scale × 1.0
  cols  9-31  joint_pos          (23)  relative joint positions
                                        stored AFTER scale × 1.0
  cols 32-54  joint_vel          (23)  relative joint velocities
                                        stored AFTER scale × 0.05
  cols 55-77  actions            (23)  last actions sent to the env
                                        stored AFTER scale × 1.0

  Total: 78 columns

  If --obs_raw is set, the CSV contains unscaled sensor values (rad,
  rad/s, m/s …) and the script applies the standard policy scales
  automatically before feeding them to the model.

Action CSV format (--csv)  [optional]
--------------------------------------
One row per step, one column per action DOF (23 columns).
If omitted the policy's own output drives env.step().

Usage examples
--------------
# Feed obs from CSV → policy → actions drive the sim
python csv_play.py \\
    --task  Isaac-Velocity-Flat-Hanu-A4-Play-v0 \\
    --checkpoint /path/to/model.pt \\
    --obs_csv /path/to/observations.csv \\
    --num_envs 1

# Feed both obs and actions from CSV (policy output is ignored)
python csv_play.py \\
    --task  Isaac-Velocity-Flat-Hanu-A4-Play-v0 \\
    --checkpoint /path/to/model.pt \\
    --obs_csv /path/to/observations.csv \\
    --csv    /path/to/actions.csv \\
    --num_envs 1

# Feed actions only from CSV (obs come from live sim)
python csv_play.py \\
    --task  Isaac-Velocity-Flat-Hanu-A4-Play-v0 \\
    --checkpoint /path/to/model.pt \\
    --csv    /path/to/actions.csv \\
    --num_envs 1
"""

# ──────────────────────────────────────────────────────────────────────────────
# 1.  Argument parsing & AppLauncher  (must happen BEFORE any Isaac imports)
# ──────────────────────────────────────────────────────────────────────────────
import argparse
import sys

from isaaclab.app import AppLauncher

import cli_args  # isort: skip  (local helper in the same scripts directory)

parser = argparse.ArgumentParser(
    description=(
        "Play an RSL-RL environment by injecting pre-recorded observations "
        "and/or actions from CSV files into the trained model."
    )
)
parser.add_argument("--video",        action="store_true", default=False,
                    help="Record a video during play.")
parser.add_argument("--video_length", type=int, default=200,
                    help="Length of the recorded video (in steps).")
parser.add_argument("--disable_fabric", action="store_true", default=False,
                    help="Disable fabric and use USD I/O operations.")
parser.add_argument("--num_envs",     type=int, default=None,
                    help="Number of environments to simulate.")
parser.add_argument("--task",         type=str, default=None,
                    help="Name of the task.")
parser.add_argument("--agent",        type=str, default="rsl_rl_cfg_entry_point",
                    help="Name of the RL agent configuration entry point.")
parser.add_argument("--seed",         type=int, default=None,
                    help="Seed used for the environment.")
parser.add_argument("--use_pretrained_checkpoint", action="store_true",
                    help="Use the pre-trained checkpoint from Nucleus.")
parser.add_argument("--real-time",    action="store_true", default=False,
                    help="Run in real-time, if possible.")
parser.add_argument("--loop",         action="store_true", default=False,
                    help="Loop CSVs when exhausted instead of stopping.")

# ── Observation CSV ────────────────────────────────────────────────────────────
parser.add_argument(
    "--obs_csv", type=str, default="",
    help=(
        "Path to the CSV file containing pre-recorded policy observations. "
        "Each row = one simulation step; columns must match the 78-dim "
        "policy observation vector (see docstring for layout). "
        "If omitted, live environment observations are used."
    ),
)
parser.add_argument(
    "--obs_raw", action="store_true", default=False,
    help=(
        "Treat --obs_csv values as raw sensor readings (rad, rad/s, …) and "
        "apply the standard policy scales automatically. "
        "Default: CSV already contains post-scale values."
    ),
)

# ── Action CSV ────────────────────────────────────────────────────────────────
parser.add_argument(
    "--csv", type=str, default="",
    help=(
        "Path to the CSV file containing pre-recorded actions. "
        "Each row = one step; one column per action DOF. "
        "If omitted, the policy network output drives env.step()."
    ),
)
parser.add_argument(
    "--action_scale", type=float, default=0.25,
    help="Scalar multiplier applied to every value in --csv (default: 0.25).",
)

cli_args.add_rsl_rl_args(parser)
AppLauncher.add_app_launcher_args(parser)

args_cli, hydra_args = parser.parse_known_args()\
    if "--help" not in sys.argv else (parser.parse_args(), [])
if args_cli.video:
    args_cli.enable_cameras = True

sys.argv = [sys.argv[0]] + hydra_args

app_launcher   = AppLauncher(args_cli)
simulation_app = app_launcher.app

# ──────────────────────────────────────────────────────────────────────────────
# 2.  Python / Isaac imports  (after AppLauncher)
# ──────────────────────────────────────────────────────────────────────────────
import csv as _csv
import os
import time

import gymnasium as gym
import numpy as np
import torch
from rsl_rl.runners import DistillationRunner, OnPolicyRunner

from isaaclab.envs import (
    DirectMARLEnv,
    DirectMARLEnvCfg,
    DirectRLEnvCfg,
    ManagerBasedRLEnvCfg,
    multi_agent_to_single_agent,
)
from isaaclab.utils.assets import retrieve_file_path
from isaaclab.utils.dict import print_dict
from isaaclab.utils.pretrained_checkpoint import get_published_pretrained_checkpoint

from isaaclab_rl.rsl_rl import (
    RslRlBaseRunnerCfg,
    RslRlVecEnvWrapper,
)

import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils import get_checkpoint_path
from isaaclab_tasks.utils.hydra import hydra_task_config

import hanu_lab.tasks  # noqa: F401


# ──────────────────────────────────────────────────────────────────────────────
# 3.  Observation term layout
# ──────────────────────────────────────────────────────────────────────────────

# Each entry: (name, start_col, end_col_exclusive, policy_scale)
# Columns are the positions inside the 78-dim policy observation vector.
# The *policy_scale* is the factor applied by IsaacLab's ObservationManager
# BEFORE the value is fed into the neural network.
OBS_TERMS: list[tuple[str, int, int, float]] = [
    # name                  start  stop   policy_scale
    ("base_ang_vel",            0,    3,   0.25),
    ("projected_gravity",       3,    6,   1.00),
    ("velocity_commands",       6,    9,   1.00),
    ("joint_pos",               9,   32,   1.00),
    ("joint_vel",              32,   55,   0.05),
    ("actions",                55,   78,   1.00),
]
OBS_DIM = 78   # total policy observation vector length


def print_obs_layout() -> None:
    """Print the observation term / CSV column mapping to stdout."""
    print("\n========== POLICY OBS CSV COLUMN LAYOUT ==========")
    print(f"  {'Term':<25} {'cols':>12}  {'dims':>5}  {'policy_scale':>12}")
    print(f"  {'-'*25} {'-'*12}  {'-'*5}  {'-'*12}")
    for name, start, stop, scale in OBS_TERMS:
        dim = stop - start
        cols = f"{start}–{stop-1}"
        print(f"  {name:<25} {cols:>12}  {dim:>5}  {scale:>12.4f}")
    print(f"  {'TOTAL':<25} {'0–77':>12}  {OBS_DIM:>5}")
    print("===================================================\n", flush=True)


# ──────────────────────────────────────────────────────────────────────────────
# 4.  Generic CSV loader
# ──────────────────────────────────────────────────────────────────────────────

def _load_csv(csv_path: str, label: str) -> np.ndarray:
    """
    Load a 2-D float array from *csv_path*.

    Skips any row whose first field is non-numeric (header detection).
    Returns shape ``(T, num_cols)`` as float32.
    """
    if not os.path.isfile(csv_path):
        raise FileNotFoundError(f"[CSV-{label}] File not found: {csv_path}")

    rows: list[list[float]] = []
    with open(csv_path, newline="") as f:
        reader = _csv.reader(f)
        for lineno, raw_row in enumerate(reader, start=1):
            if not raw_row:
                continue
            try:
                rows.append([float(x) for x in raw_row])
            except ValueError:
                print(f"[CSV-{label}] Skipping non-numeric row {lineno}: {raw_row[:5]}…")

    if not rows:
        raise ValueError(f"[CSV-{label}] No numeric rows found in {csv_path}")

    arr = np.array(rows, dtype=np.float32)
    print(f"[CSV-{label}] Loaded {arr.shape[0]} steps × {arr.shape[1]} cols from {csv_path}")
    return arr


# ──────────────────────────────────────────────────────────────────────────────
# 5.  Observation builder from CSV row
# ──────────────────────────────────────────────────────────────────────────────

def build_obs_from_row(csv_row: np.ndarray, apply_raw_scale: bool, device, num_envs: int) -> dict:
    """
    Convert one CSV row (shape ``(78,)`` or ``(OBS_DIM,)``) into the
    ``{"policy": tensor}`` dict consumed by the RSL-RL policy.

    Parameters
    ----------
    csv_row :         1-D float32 array of length OBS_DIM.
    apply_raw_scale : If True the CSV contains *raw* sensor values and we
                      multiply each term by its policy_scale here.
    device :          torch device string / object.
    num_envs :        Number of parallel environments (the tensor is tiled).

    Returns
    -------
    dict with key ``"policy"``  →  tensor shape ``(num_envs, OBS_DIM)``
    """
    obs_vec = csv_row.copy()   # (OBS_DIM,)

    if apply_raw_scale:
        for _, start, stop, scale in OBS_TERMS:
            obs_vec[start:stop] *= scale

    obs_t = torch.tensor(obs_vec, dtype=torch.float32, device=device)   # (OBS_DIM,)
    obs_t = obs_t.unsqueeze(0).expand(num_envs, -1)                     # (num_envs, OBS_DIM)
    return {"policy": obs_t}


# ──────────────────────────────────────────────────────────────────────────────
# 6.  Main
# ──────────────────────────────────────────────────────────────────────────────

@hydra_task_config(args_cli.task, args_cli.agent)
def main(
    env_cfg:   ManagerBasedRLEnvCfg | DirectRLEnvCfg | DirectMARLEnvCfg,
    agent_cfg: RslRlBaseRunnerCfg,
):
    """
    Load the trained model, optionally inject observations and/or actions
    from CSV files, and simulate.

    Modes
    -----
    - obs_csv only     : CSV obs → policy → actions → env.step()
    - csv (action) only: live obs → env.step(csv_actions)
    - both             : CSV obs → policy (output ignored) → env.step(csv_actions)
    - neither          : normal live play (obs from env, actions from policy)
    """

    # ── resolve checkpoint path ───────────────────────────────────────────────
    task_name       = args_cli.task.split(":")[-1]
    train_task_name = task_name.replace("-Play", "")

    agent_cfg: RslRlBaseRunnerCfg = cli_args.update_rsl_rl_cfg(agent_cfg, args_cli)
    env_cfg.scene.num_envs = (
        args_cli.num_envs if args_cli.num_envs is not None else env_cfg.scene.num_envs
    )
    env_cfg.seed       = agent_cfg.seed
    env_cfg.sim.device = (
        args_cli.device if args_cli.device is not None else env_cfg.sim.device
    )

    log_root_path = os.path.abspath(os.path.join("logs", "rsl_rl", agent_cfg.experiment_name))
    print(f"[INFO] Loading experiment directory: {log_root_path}")

    if args_cli.use_pretrained_checkpoint:
        resume_path = get_published_pretrained_checkpoint("rsl_rl", train_task_name)
        if not resume_path:
            print("[INFO] No pre-trained checkpoint available for this task.")
            return
    elif args_cli.checkpoint:
        resume_path = retrieve_file_path(args_cli.checkpoint)
    else:
        resume_path = get_checkpoint_path(
            log_root_path, agent_cfg.load_run, agent_cfg.load_checkpoint
        )

    log_dir = os.path.dirname(resume_path)
    env_cfg.log_dir = log_dir

    # ── print obs layout ──────────────────────────────────────────────────────
    print_obs_layout()

    # ── load observation CSV (optional) ───────────────────────────────────────
    obs_csv_data: np.ndarray | None = None
    if args_cli.obs_csv:
        obs_csv_data = _load_csv(args_cli.obs_csv, "OBS")
        num_obs_rows, num_obs_cols = obs_csv_data.shape
        if num_obs_cols != OBS_DIM:
            print(
                f"[WARNING] --obs_csv has {num_obs_cols} columns but policy expects "
                f"{OBS_DIM}. Rows will be ZERO-PADDED or TRUNCATED to {OBS_DIM}."
            )
            padded = np.zeros((num_obs_rows, OBS_DIM), dtype=np.float32)
            copy_cols = min(num_obs_cols, OBS_DIM)
            padded[:, :copy_cols] = obs_csv_data[:, :copy_cols]
            obs_csv_data = padded
        print(
            f"[CSV-OBS] Will inject CSV observations into policy "
            f"({'raw sensor values → auto-scaled' if args_cli.obs_raw else 'already-scaled values'})."
        )
    else:
        print("[CSV-OBS] No --obs_csv provided → using live environment observations.")

    # ── load action CSV (optional) ────────────────────────────────────────────
    act_csv_data: np.ndarray | None = None
    if args_cli.csv:
        act_csv_data = _load_csv(args_cli.csv, "ACT")
        num_act_rows, num_act_dofs = act_csv_data.shape
        print(
            f"[CSV-ACT] Will override env.step() with CSV actions "
            f"(scale × {args_cli.action_scale})."
        )
    else:
        print("[CSV-ACT] No --csv provided → actions come from the policy network.")

    # ── build environment ─────────────────────────────────────────────────────
    env = gym.make(
        args_cli.task,
        cfg=env_cfg,
        render_mode="rgb_array" if args_cli.video else None,
    )

    try:
        print("[DEBUG] obs group dims:", env.unwrapped.observation_manager.group_obs_dim)
    except Exception as exc:
        print("[DEBUG] obs dim check error:", exc)

    if isinstance(env.unwrapped, DirectMARLEnv):
        env = multi_agent_to_single_agent(env)

    if args_cli.video:
        video_kwargs = {
            "video_folder":   os.path.join(log_dir, "videos", "csv_play"),
            "step_trigger":   lambda step: step == 0,
            "video_length":   args_cli.video_length,
            "disable_logger": True,
        }
        print("[INFO] Recording video.")
        print_dict(video_kwargs, nesting=4)
        env = gym.wrappers.RecordVideo(env, **video_kwargs)

    env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)

    # ── warm-start reset ──────────────────────────────────────────────────────
    obs = env.reset()
    if isinstance(obs, tuple):
        obs = obs[0]
    print("\n========== OBS DEBUG (live env after reset) ==========")
    if hasattr(obs, "keys"):
        print("keys:", list(obs.keys()))
        if "policy" in obs:
            print("obs['policy'].shape:", obs["policy"].shape)
    else:
        print("obs.shape:", obs.shape)
    print("======================================================\n", flush=True)

    # ── load checkpoint ───────────────────────────────────────────────────────
    print(f"[INFO] Loading model checkpoint from: {resume_path}")

    if agent_cfg.class_name == "OnPolicyRunner":
        runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    elif agent_cfg.class_name == "DistillationRunner":
        runner = DistillationRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    else:
        raise ValueError(f"Unsupported runner class: {agent_cfg.class_name}")

    # Partial load – tolerates obs-dim mismatches.
    ckpt    = torch.load(resume_path, map_location="cpu")
    state   = ckpt["model_state_dict"]
    current = runner.alg.policy.state_dict()
    filtered, skipped = {}, []
    for k, v in state.items():
        if k in current and current[k].shape == v.shape:
            filtered[k] = v
        else:
            skipped.append(
                (k, tuple(v.shape), tuple(current[k].shape) if k in current else None)
            )
    runner.alg.policy.load_state_dict(filtered, strict=False)
    print(f"[DEBUG] Loaded params: {len(filtered)}  |  Skipped: {len(skipped)}")
    for s in skipped[:10]:
        print("[DEBUG] skipped:", s)

    policy    = runner.get_inference_policy(device=env.unwrapped.device)
    try:
        policy_nn = runner.alg.policy
    except AttributeError:
        policy_nn = runner.alg.actor_critic

    # ── simulation loop ───────────────────────────────────────────────────────
    dt         = env.unwrapped.step_dt
    obs        = env.get_observations()
    device     = env.unwrapped.device
    num_envs   = env.unwrapped.num_envs
    timestep   = 0
    csv_row_idx = 0    # shared pointer for both CSVs (they stay in sync)

    total_csv_rows = (
        min(
            obs_csv_data.shape[0] if obs_csv_data is not None else int(1e9),
            act_csv_data.shape[0] if act_csv_data is not None else int(1e9),
        )
        if (obs_csv_data is not None or act_csv_data is not None)
        else int(1e9)
    )
    print(f"[CSV] Simulation will run for up to {total_csv_rows} CSV steps. "
          f"{'Looping enabled.' if args_cli.loop else 'Will stop when CSV is exhausted.'}")

    while simulation_app.is_running():
        start_time = time.time()

        # ── check CSV exhaustion ──────────────────────────────────────────────
        if csv_row_idx >= total_csv_rows:
            if args_cli.loop:
                csv_row_idx = 0
                print("[CSV] Rewound to start of CSV.")
            else:
                print("[CSV] All CSV rows consumed – exiting simulation.")
                break

        # ── build observations ────────────────────────────────────────────────
        if obs_csv_data is not None:
            # Override: construct obs tensor directly from CSV row
            obs = build_obs_from_row(
                obs_csv_data[csv_row_idx],
                apply_raw_scale=args_cli.obs_raw,
                device=device,
                num_envs=num_envs,
            )
        # else: obs comes from the previous env.step() / env.get_observations()

        # ── run policy to get actions ─────────────────────────────────────────
        with torch.inference_mode():
            actions = policy(obs)

        # ── override actions from CSV if provided ─────────────────────────────
        if act_csv_data is not None:
            csv_row = act_csv_data[csv_row_idx] * args_cli.action_scale   # (num_dofs,)

            # Validate DOF count on the first step
            if timestep == 0:
                try:
                    env_act_dim = env.action_manager.total_action_dim
                except AttributeError:
                    env_act_dim = csv_row.shape[0]
                if csv_row.shape[0] != env_act_dim:
                    print(
                        f"[WARNING] --csv has {csv_row.shape[0]} DOFs but env expects "
                        f"{env_act_dim} DOFs. Will ZERO-PAD or TRUNCATE."
                    )

            act_np  = np.tile(csv_row, (num_envs, 1))
            actions = torch.tensor(act_np, dtype=torch.float32, device=device)

        # ── step the environment ──────────────────────────────────────────────
        with torch.inference_mode():
            obs, _, dones, _ = env.step(actions)
            policy_nn.reset(dones)

        csv_row_idx += 1

        # ── per-step debug log (every 200 steps) ─────────────────────────────
        if timestep % 200 == 0:
            base_env = env.unwrapped

            print(f"\n[CSV] ── step {timestep:5d}  (csv_row {csv_row_idx}/{total_csv_rows}) ──")

            # --- observation injection summary ---
            if obs_csv_data is not None:
                row = obs_csv_data[csv_row_idx - 1]
                print("[OBS-CSV] Injected observation terms:")
                for name, start, stop, scale in OBS_TERMS:
                    vals = row[start:stop]
                    scale_tag = f"(raw×{scale})" if args_cli.obs_raw else "(pre-scaled)"
                    vals_str  = ", ".join(f"{v:.4f}" for v in vals[:6])
                    if stop - start > 6:
                        vals_str += f", … ({stop-start} total)"
                    print(f"  {name:<25} {scale_tag}  [{vals_str}]")

            # --- action summary ---
            act_np_log = actions[0].cpu().numpy()   # env 0
            print(f"[ACT]  actions[0] ({len(act_np_log)} DOFs): "
                  f"{[f'{v:.4f}' for v in act_np_log[:8]]}{'...' if len(act_np_log)>8 else ''}")

            # --- live robot state ---
            try:
                cmd = base_env.command_manager.get_command("base_velocity")
                print(f"[DBG]  cmd base_velocity env0: {cmd[0].tolist()}")
            except Exception as exc:
                print(f"[DBG]  cmd read error: {exc}")
            try:
                v_b = base_env.scene["robot"].data.root_lin_vel_b
                print(f"[DBG]  root_lin_vel_b env0: {v_b[0].tolist()}")
            except Exception as exc:
                print(f"[DBG]  vel_b read error: {exc}")

        timestep += 1

        # ── early exit for video recording ────────────────────────────────────
        if args_cli.video and timestep >= args_cli.video_length:
            break

        # ── real-time pacing ──────────────────────────────────────────────────
        sleep_time = dt - (time.time() - start_time)
        if args_cli.real_time and sleep_time > 0:
            time.sleep(sleep_time)

    # ── tidy up ───────────────────────────────────────────────────────────────
    print(f"[CSV] Simulation finished at step {timestep}.")
    env.close()


# ──────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    main()
    simulation_app.close()
