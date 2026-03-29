# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

"""
play_plot.py  run a trained RSL-RL policy and produce Command vs. Actual
               Velocity comparison plots (vx, vy, ωz) when the simulation ends.

Usage (identical to play.py):
    python play_plot.py --task Isaac-Velocity-Flat-Hanu-A4-v0 \
                        --checkpoint /path/to/model.pt \
                        --num_envs 1 \
                        [--plot_steps 2000] \
                        [--real-time] [--video] [--video_length 200]
"""

# ──────────────────────────────────────────────────────────────────────────────
# 1.  Argument parsing & AppLauncher  (must happen before any Isaac imports)
# ──────────────────────────────────────────────────────────────────────────────
import argparse
import sys

from isaaclab.app import AppLauncher

import cli_args  # isort: skip  (local helper in the same directory)

parser = argparse.ArgumentParser(description="Play an RSL-RL policy and plot velocity tracking.")
parser.add_argument("--video",        action="store_true", default=False,
                    help="Record videos during play.")
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
parser.add_argument("--plot_steps",   type=int, default=1000,
                    help="Stop simulation and plot after N steps (0 = run until window closed).")
parser.add_argument("--plot_env",     type=int, default=0,
                    help="Which environment index to record for plotting (default: 0).")
parser.add_argument("--save_plot",    type=str, default="",
                    help="If set, save the plot to this file path instead of showing it.")

cli_args.add_rsl_rl_args(parser)
AppLauncher.add_app_launcher_args(parser)

args_cli, hydra_args = parser.parse_known_args()
if args_cli.video:
    args_cli.enable_cameras = True

sys.argv = [sys.argv[0]] + hydra_args

app_launcher    = AppLauncher(args_cli)
simulation_app  = app_launcher.app

# ──────────────────────────────────────────────────────────────────────────────
# 2.  Python / Isaac imports  (after AppLauncher)
# ──────────────────────────────────────────────────────────────────────────────
import os
import time
import csv

import gymnasium as gym
import torch
import numpy as np
import matplotlib
matplotlib.use("Agg")          # headless-safe; switch to TkAgg/Qt5Agg if you want interactive
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

from rsl_rl.runners import DistillationRunner, OnPolicyRunner

from isaaclab.envs import (
    DirectMARLEnv,
    DirectMARLEnvCfg,
    DirectRLEnvCfg,
    ManagerBasedRLEnvCfg,
    multi_agent_to_single_agent,
)
from isaaclab.utils.assets import retrieve_file_path
from isaaclab.utils.dict   import print_dict
from isaaclab.utils.pretrained_checkpoint import get_published_pretrained_checkpoint

from isaaclab_rl.rsl_rl import (
    RslRlBaseRunnerCfg,
    RslRlVecEnvWrapper,
    export_policy_as_jit,
    export_policy_as_onnx,
)

import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils       import get_checkpoint_path
from isaaclab_tasks.utils.hydra import hydra_task_config

import hanu_lab.tasks  # noqa: F401


# ──────────────────────────────────────────────────────────────────────────────
# 3.  Plotting helper
# ──────────────────────────────────────────────────────────────────────────────
_CHANNEL_LABELS = [
    ("$v_x$  [m/s]",   "Linear X"),
    ("$v_y$  [m/s]",   "Linear Y"),
    ("$\\omega_z$  [rad/s]", "Angular Z"),
]

_PALETTE = {
    "cmd":    "#E65100",   # deep orange dashed = commanded
    "actual": "#1565C0",   # deep blue solid   = actual
    "bg":     "#FFFFFF",
    "fg":     "#212121",
    "grid":   "#BDBDBD",
    "panel":  "#FFFFFF",
}


def plot_velocity_comparison(
    timestamps: list,
    cmd_log:    list,   # each entry: np.array shape (3,) → [vx_cmd, vy_cmd, wz_cmd]
    vel_log:    list,   # each entry: np.array shape (3,) → [vx_act, vy_act, wz_act]
    save_path:  str = "",
    rmse:       np.ndarray | None = None,  # shape (3,) per-channel RMSE
):
    """Render a 3-row figure: commanded (dashed) vs actual (solid) for vx, vy, ωz."""

    t  = np.asarray(timestamps)
    c  = np.asarray(cmd_log)   # (T, 3)
    v  = np.asarray(vel_log)   # (T, 3)

    fig = plt.figure(figsize=(14, 10), facecolor=_PALETTE["bg"])
    fig.suptitle(
        "Command vs. Actual Base Velocity",
        color=_PALETTE["fg"], fontsize=16, fontweight="bold", y=0.97,
    )

    gs = gridspec.GridSpec(3, 1, hspace=0.45, left=0.08, right=0.97,
                           top=0.92, bottom=0.07)

    for i, (ylabel, title) in enumerate(_CHANNEL_LABELS):
        ax = fig.add_subplot(gs[i])
        ax.set_facecolor(_PALETTE["panel"])
        ax.tick_params(colors=_PALETTE["fg"])
        ax.yaxis.label.set_color(_PALETTE["fg"])
        ax.xaxis.label.set_color(_PALETTE["fg"])
        ax.title.set_color(_PALETTE["fg"])
        for spine in ax.spines.values():
            spine.set_edgecolor(_PALETTE["grid"])

        ax.plot(t, c[:, i], color=_PALETTE["cmd"],
                linestyle="--", linewidth=1.8, label="Commanded")
        ax.plot(t, v[:, i], color=_PALETTE["actual"],
                linestyle="-",  linewidth=1.8, label="Actual")

        # shade the tracking error
        ax.fill_between(t, c[:, i], v[:, i],
                        alpha=0.12, color=_PALETTE["actual"])

        ax.set_title(title, fontsize=11, pad=4)
        ax.set_ylabel(ylabel, fontsize=10)
        ax.grid(True, color=_PALETTE["grid"], linewidth=0.6, linestyle="--")

        if i == 2:
            ax.set_xlabel("Time  [s]", fontsize=10)

        # ── Aggressive Outlier cropping: focus on 2th-98th percentile ──────
        # combined_data = np.concatenate([c[:, i], v[:, i]])
        # if len(combined_data) > 0:
        #     y_lims = np.percentile(combined_data, [2, 98])
        #     margin = max((y_lims[1] - y_lims[0]) * 0.15, 0.05)
        #     ax.set_ylim(y_lims[0] - margin, y_lims[1] + margin)

        leg = ax.legend(
            loc="upper right", fontsize=9,
            facecolor=_PALETTE["panel"], edgecolor=_PALETTE["grid"],
            labelcolor=_PALETTE["fg"],
        )

        # annotate RMSE in upper-left corner of each subplot
        if rmse is not None:
            ax.text(
                0.01, 0.95, f"RMSE = {rmse[i]:.4f}",
                transform=ax.transAxes,
                fontsize=9, verticalalignment="top",
                color="#C62828",
                bbox=dict(boxstyle="round,pad=0.3", facecolor="#FFEBEE",
                          edgecolor="#EF9A9A", alpha=0.9),
            )

    if save_path:
        os.makedirs(os.path.dirname(os.path.abspath(save_path)), exist_ok=True)
        fig.savefig(save_path, dpi=150, bbox_inches="tight", facecolor=_PALETTE["bg"])
        print(f"[PLOT] Saved to: {save_path}")
    
    plt.close(fig)


def export_to_csv(
    timestamps: list[float],
    cmd_log:    list[np.ndarray],
    vel_log:    list[np.ndarray],
    save_dir:   str
):
    """Save velocity command and actual data to a CSV file."""
    if not os.path.exists(save_dir):
        os.makedirs(save_dir, exist_ok=True)
    
    filename = os.path.join(save_dir, "velocity_data.csv")
    
    headers = ["time", "vx_cmd", "vy_cmd", "wz_cmd", "vx_act", "vy_act", "wz_act"]
    
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(headers)
        
        for i in range(len(timestamps)):
            row = [timestamps[i]]
            row.extend(cmd_log[i].tolist())
            row.extend(vel_log[i].tolist())
            writer.writerow(row)
            
    print(f"[PLOT] Data exported to → {filename}")


# ──────────────────────────────────────────────────────────────────────────────
# 4.  Main
# ──────────────────────────────────────────────────────────────────────────────
@hydra_task_config(args_cli.task, args_cli.agent)
def main(
    env_cfg:   ManagerBasedRLEnvCfg | DirectRLEnvCfg | DirectMARLEnvCfg,
    agent_cfg: RslRlBaseRunnerCfg,
):
    """Load a trained checkpoint and run the robot, then plot velocity tracking."""

    # ── resolve checkpoint ────────────────────────────────────────────────────
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
    print(f"[INFO] Loading experiment from directory: {log_root_path}")

    if args_cli.use_pretrained_checkpoint:
        resume_path = get_published_pretrained_checkpoint("rsl_rl", train_task_name)
        if not resume_path:
            print("[INFO] No pre-trained checkpoint available for this task.")
            return
    elif args_cli.checkpoint:
        resume_path = retrieve_file_path(args_cli.checkpoint)
    else:
        resume_path = get_checkpoint_path(log_root_path, agent_cfg.load_run,
                                          agent_cfg.load_checkpoint)

    log_dir = os.path.dirname(resume_path)
    env_cfg.log_dir = log_dir

    # ── build environment ─────────────────────────────────────────────────────
    env = gym.make(
        args_cli.task,
        cfg=env_cfg,
        render_mode="rgb_array" if args_cli.video else None,
    )

    # debug: observation group dims
    try:
        print("[DEBUG] obs group dims:", env.unwrapped.observation_manager.group_obs_dim)
    except Exception as exc:
        print("[DEBUG] obs dim check error:", exc)

    if isinstance(env.unwrapped, DirectMARLEnv):
        env = multi_agent_to_single_agent(env)

    if args_cli.video:
        video_kwargs = {
            "video_folder":   os.path.join(log_dir, "videos", "play"),
            "step_trigger":   lambda step: step == 0,
            "video_length":   args_cli.video_length,
            "disable_logger": True,
        }
        print("[INFO] Recording video.")
        print_dict(video_kwargs, nesting=4)
        env = gym.wrappers.RecordVideo(env, **video_kwargs)

    env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)

    # ── warm-start reset so we can inspect obs shape ──────────────────────────
    obs = env.reset()
    if isinstance(obs, tuple):
        obs = obs[0]
    print("\n========== OBS DEBUG ==========")
    if hasattr(obs, "keys"):
        print("keys:", list(obs.keys()))
        if "policy" in obs:
            print("obs['policy'].shape:", obs["policy"].shape)
    else:
        print("obs.shape:", obs.shape)
    print("================================\n", flush=True)

    # ── load checkpoint ───────────────────────────────────────────────────────
    print(f"[INFO] Loading model checkpoint from: {resume_path}")

    if agent_cfg.class_name == "OnPolicyRunner":
        runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    elif agent_cfg.class_name == "DistillationRunner":
        runner = DistillationRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    else:
        raise ValueError(f"Unsupported runner class: {agent_cfg.class_name}")

    # partial load (bypass obs-dim mismatch)
    ckpt    = torch.load(resume_path, map_location="cpu")
    state   = ckpt["model_state_dict"]
    current = runner.alg.policy.state_dict()
    filtered, skipped = {}, []
    for k, v in state.items():
        if k in current and current[k].shape == v.shape:
            filtered[k] = v
        else:
            skipped.append((k, tuple(v.shape),
                            tuple(current[k].shape) if k in current else None))
    runner.alg.policy.load_state_dict(filtered, strict=False)
    print(f"[DEBUG] Loaded params: {len(filtered)}  |  Skipped: {len(skipped)}")
    for s in skipped[:10]:
        print("[DEBUG] skipped:", s)

    policy = runner.get_inference_policy(device=env.unwrapped.device)

    try:
        policy_nn = runner.alg.policy
    except AttributeError:
        policy_nn = runner.alg.actor_critic

    # normalizer
    if hasattr(policy_nn, "actor_obs_normalizer"):
        normalizer = policy_nn.actor_obs_normalizer
    elif hasattr(policy_nn, "student_obs_normalizer"):
        normalizer = policy_nn.student_obs_normalizer
    else:
        normalizer = None

    # # export
    # export_dir = os.path.join(log_dir, "exported")
    # export_policy_as_jit( policy_nn, normalizer=normalizer, path=export_dir, filename="policy.pt")
    # export_policy_as_onnx(policy_nn, normalizer=normalizer, path=export_dir, filename="policy.onnx")

    # ── simulation loop ───────────────────────────────────────────────────────
    dt          = env.unwrapped.step_dt
    obs         = env.get_observations()
    timestep    = 0
    env_idx     = args_cli.plot_env       # which env to record
    plot_steps  = args_cli.plot_steps     # 0 → run until window closed

    # data buffers
    timestamps: list[float] = []
    cmd_log:    list[np.ndarray] = []
    vel_log:    list[np.ndarray] = []

    print(f"[PLOT] Recording env #{env_idx} for velocity comparison plot.")
    if plot_steps:
        print(f"[PLOT] Will run for {plot_steps} steps then plot.")

    while simulation_app.is_running():
        start_time = time.time()
        # -- Override base linear velocity command to 0.4 m/s --
        base_env = env.unwrapped
        if hasattr(base_env, "command_manager"):
            # Set vx = 0.0, vy = 0.4, angular Z = 0.0
            base_env.command_manager.get_command("base_velocity")[:, 0] = 0.0
            base_env.command_manager.get_command("base_velocity")[:, 1] = 0.4 # forward y-axis
            base_env.command_manager.get_command("base_velocity")[:, 2] = 0.0
            # Refresh observations to reflect the manual command
            obs = env.get_observations()

        with torch.inference_mode():
            actions = policy(obs)
            obs, _, dones, _ = env.step(actions)
            policy_nn.reset(dones)

        base_env = env.unwrapped

        # ── record commanded & actual velocity ─────────────────────────────
        try:
            cmd_tensor = base_env.command_manager.get_command("base_velocity")
            # shape: (num_envs, 3)  → [vx_cmd, vy_cmd, wz_cmd]
            cmd_np = cmd_tensor[env_idx].cpu().numpy().astype(np.float32)
        except Exception:
            cmd_np = np.zeros(3, dtype=np.float32)

        try:
            # body-frame velocity is the fairest comparison with commands
            v_b = base_env.scene["robot"].data.root_lin_vel_b
            w_z = base_env.scene["robot"].data.root_ang_vel_b
            # [vx_act, vy_act, wz_act]
            vel_np = np.array([
                v_b[env_idx, 0].item(),
                v_b[env_idx, 1].item(),
                w_z[env_idx, 2].item(),
            ], dtype=np.float32)
        except Exception:
            vel_np = np.zeros(3, dtype=np.float32)

        timestamps.append(timestep * dt)
        cmd_log.append(cmd_np)
        vel_log.append(vel_np)

        # ── console log every 200 steps ────────────────────────────────────
        if timestep % 200 == 0:
            print(f"[DBG] step={timestep:5d} | "
                  f"cmd=[{cmd_np[0]:6.3f}, {cmd_np[1]:6.3f}, {cmd_np[2]:6.3f}] | "
                  f"vel=[{vel_np[0]:6.3f}, {vel_np[1]:6.3f}, {vel_np[2]:6.3f}]")

        timestep += 1

        # ── early exit for video recording ─────────────────────────────────
        if args_cli.video and timestep >= args_cli.video_length:
            break

        # ── early exit after plot_steps ────────────────────────────────────
        if plot_steps and timestep >= plot_steps:
            break

        # real-time pacing
        sleep_time = dt - (time.time() - start_time)
        if args_cli.real_time and sleep_time > 0:
            time.sleep(sleep_time)

    # ── final summary ─────────────────────────────────────────────────────────
    if timestamps:
        last_cmd = cmd_log[-1]
        last_vel = vel_log[-1]
        print("[DBG] FINAL cmd:", last_cmd.tolist())
        print("[DBG] FINAL vel:", last_vel.tolist())

    env.close()

    # ── plot ──────────────────────────────────────────────────────────────────
    if len(timestamps) < 2:
        print("[PLOT] Not enough data to plot.")
        return

    # ── RMSE per channel (robust estimate: excluding extreme spikes) ──────────
    c_arr = np.asarray(cmd_log)   # (T, 3)
    v_arr = np.asarray(vel_log)   # (T, 3)

    # We exclude the top 2% of squared errors for each channel.
    # This effectively removes huge outliers caused by robot resets or spawning, 
    # which cause momentary velocity spikes in the Isaac Sim root state.
    err_sq = (c_arr - v_arr) ** 2
    rmse_list = []
    for i in range(3):
        e_sq = err_sq[:, i]
        thresh = np.percentile(e_sq, 98)
        rmse_list.append(np.sqrt(np.mean(e_sq[e_sq <= thresh])))

    rmse = np.array(rmse_list)
    labels = ["vx", "vy", "wz"]
    print("\n========== RMSE SUMMARY ==========")
    for lbl, val in zip(labels, rmse):
        print(f"  {lbl}  RMSE = {val:.4f}")
    print(f"  Overall RMSE = {float(np.mean(rmse)):.4f}")
    print("==================================\n", flush=True)

    # ── determine output directory ────────────────────────────────────────────
    save_dir = args_cli.save_plot or os.path.join(log_dir, "velocity_plots")
    os.makedirs(save_dir, exist_ok=True)

    # ── Plot 1: Velocity Comparison ───────────────────────────────────────────
    plot_path = os.path.join(save_dir, "velocity_comparison.png")
    plot_velocity_comparison(timestamps, cmd_log, vel_log, save_path=plot_path, rmse=rmse)

    # ── Export 2: CSV Data ────────────────────────────────────────────────────
    export_to_csv(timestamps, cmd_log, vel_log, save_dir=save_dir)

    print(f"[PLOT] All plots and data saved to: {save_dir}")


# ──────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    main()
    simulation_app.close()
