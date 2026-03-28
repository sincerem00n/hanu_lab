# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

"""
toq_play.py  Run a trained RSL-RL policy and produce torque and action
             analysis plots when the simulation ends.

Usage:
    python toq_play.py --task Isaac-Velocity-Flat-Hanu-A4-v0 \\
                       --checkpoint /path/to/model.pt \\
                       --num_envs 1 \\
                       [--plot_steps 1000] \\
                       [--save_plot /path/to/save]
"""

# ──────────────────────────────────────────────────────────────────────────────
# 1.  Argument parsing & AppLauncher
# ──────────────────────────────────────────────────────────────────────────────
import argparse
import sys
import os

from isaaclab.app import AppLauncher

import cli_args  # isort: skip

parser = argparse.ArgumentParser(
    description="Play an RSL-RL policy and plot joint torques and policy actions."
)
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
                    help="Stop simulation and plot after N steps.")
parser.add_argument("--plot_env",     type=int, default=0,
                    help="Which environment index to record for plotting.")
parser.add_argument("--save_plot",    type=str, default="",
                    help="If set, save plots to this directory instead of showing them.")

cli_args.add_rsl_rl_args(parser)
AppLauncher.add_app_launcher_args(parser)

args_cli, hydra_args = parser.parse_known_args()
if args_cli.video:
    args_cli.enable_cameras = True

sys.argv = [sys.argv[0]] + hydra_args

app_launcher   = AppLauncher(args_cli)
simulation_app = app_launcher.app

# ──────────────────────────────────────────────────────────────────────────────
# 2.  Python / Isaac imports
# ──────────────────────────────────────────────────────────────────────────────
import time
import csv
from collections import defaultdict

import gymnasium as gym
import numpy as np
import torch
import matplotlib
matplotlib.use("Agg")
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
from isaaclab_rl.rsl_rl import RslRlBaseRunnerCfg, RslRlVecEnvWrapper
import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils       import get_checkpoint_path
from isaaclab_tasks.utils.hydra import hydra_task_config

import hanu_lab.tasks  # noqa: F401

# ──────────────────────────────────────────────────────────────────────────────
# 3.  Style and Config
# ──────────────────────────────────────────────────────────────────────────────
# Joints to monitor.  Edit these names to match your robot's joint names.
# The script will search for them in the articulation DOF name list.
_KEY_JOINTS = [
    "L_hip_pitch",
    "R_hip_pitch",
    "L_knee_pitch",
    "R_knee_pitch",
    # "L_shoulder_pitch",
    # "R_shoulder_pitch",
]

# Color map: one colour per joint
_JOINT_COLORS = {
    "L_hip_pitch": "#1f77b4",
    "R_hip_pitch": "#ff7f0e",
    "L_knee_pitch": "#2ca02c",
    "R_knee_pitch": "#d62728",
    # "L_shoulder_pitch": "#1f77b4",
    # "R_shoulder_pitch": "#ff7f0e",
}

_PALETTE = {
    "bg":    "#FFFFFF",
    "fg":    "#212121",
    "grid":  "#BDBDBD",
    "panel": "#FFFFFF",
}

def _styled_ax(ax):
    ax.set_facecolor(_PALETTE["panel"])
    ax.tick_params(colors=_PALETTE["fg"], labelsize=10)
    ax.yaxis.label.set_color(_PALETTE["fg"])
    ax.xaxis.label.set_color(_PALETTE["fg"])
    ax.title.set_color(_PALETTE["fg"])
    for spine in ax.spines.values():
        spine.set_edgecolor(_PALETTE["grid"])
        spine.set_alpha(0.5)
    ax.grid(True, color=_PALETTE["grid"], linewidth=0.8, linestyle=":", alpha=0.7)

# ──────────────────────────────────────────────────────────────────────────────
# 4.  Plotting functions
# ──────────────────────────────────────────────────────────────────────────────

def plot_trajectories(
    timestamps: np.ndarray,
    data_logs: dict,
    title: str,
    ylabel: str,
    filename: str,
    save_path: str = ""
):
    """General plotting function for multiple signals."""
    joints = sorted(data_logs.keys())
    n = len(joints)
    if n == 0:
        print(f"[TORQUE] No data to plot for {title}.")
        return

    # Single column layout for vertical extension
    n = len(joints)
    cols = 1
    rows = n

    fig = plt.figure(figsize=(14, 4.0 * n), facecolor=_PALETTE["bg"])
    fig.suptitle(title, color=_PALETTE["fg"], fontsize=18, fontweight="bold", y=0.99)

    gs = gridspec.GridSpec(rows, cols, hspace=0.55, left=0.08, right=0.97, top=0.93, bottom=0.06)

    for i, jname in enumerate(joints):
        val = data_logs[jname]
        col = _JOINT_COLORS.get(jname, None)
        ax = fig.add_subplot(gs[i])
        _styled_ax(ax)

        ax.plot(timestamps, val, color=col, linewidth=1.5)
        ax.set_title(f"{jname.replace('_', ' ')}", fontsize=12, fontweight='bold')
        ax.set_ylabel(ylabel, fontsize=10)
        if (i // cols) == rows - 1:
            ax.set_xlabel("Time [s]", fontsize=10)

        # Range annotation
        ax.text(
            0.05, 0.95,
            f"range: [{np.min(val):.2f}, {np.max(val):.2f}]",
            transform=ax.transAxes, fontsize=8, verticalalignment="top",
            bbox=dict(boxstyle="round", facecolor="#FFFFFF", alpha=0.8)
        )

    _save_or_show(fig, save_path, filename)

def _save_or_show(fig, save_dir: str, filename: str):
    if save_dir:
        os.makedirs(save_dir, exist_ok=True)
        out = os.path.join(save_dir, filename)
    else:
        out = os.path.join(os.path.dirname(__file__), filename)

    fig.savefig(out, dpi=150, bbox_inches="tight", facecolor=fig.get_facecolor())
    print(f"[TORQUE] Saved → {out}")
    plt.close(fig)

def export_to_csv(
    timestamps: np.ndarray,
    torque_logs: dict,
    action_logs: dict,
    save_dir: str
):
    if not os.path.exists(save_dir):
        os.makedirs(save_dir, exist_ok=True)
    
    filename = os.path.join(save_dir, "torque_action_data.csv")
    
    headers = ["time"]
    joint_names = sorted(torque_logs.keys())
    for j in joint_names:
        headers.append(f"{j}_torque_nm")
        headers.append(f"{j}_action")
    
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(headers)
        
        for i in range(len(timestamps)):
            row = [timestamps[i]]
            for j in joint_names:
                row.append(torque_logs[j][i])
                row.append(action_logs[j][i])
            writer.writerow(row)
            
    print(f"[TORQUE] Data exported to → {filename}")

# ──────────────────────────────────────────────────────────────────────────────
# 5.  Main
# ──────────────────────────────────────────────────────────────────────────────

@hydra_task_config(args_cli.task, args_cli.agent)
def main(
    env_cfg:   ManagerBasedRLEnvCfg | DirectRLEnvCfg | DirectMARLEnvCfg,
    agent_cfg: RslRlBaseRunnerCfg,
):
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

    log_root_path = os.path.abspath(
        os.path.join("logs", "rsl_rl", agent_cfg.experiment_name)
    )
    
    if args_cli.use_pretrained_checkpoint:
        resume_path = get_published_pretrained_checkpoint("rsl_rl", train_task_name)
    elif args_cli.checkpoint:
        resume_path = retrieve_file_path(args_cli.checkpoint)
    else:
        resume_path = get_checkpoint_path(
            log_root_path, agent_cfg.load_run, agent_cfg.load_checkpoint
        )

    log_dir = os.path.dirname(resume_path)
    env_cfg.log_dir = log_dir

    # ── build environment ─────────────────────────────────────────────────────
    env = gym.make(
        args_cli.task,
        cfg=env_cfg,
        render_mode="rgb_array" if args_cli.video else None,
    )

    if isinstance(env.unwrapped, DirectMARLEnv):
        env = multi_agent_to_single_agent(env)

    env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)

    # ── discover joint indices ────────────────────────────────────────────────
    base_env   = env.unwrapped
    robot      = base_env.scene["robot"]
    dof_names  = robot.data.joint_names

    joint_indices: dict[str, int] = {}
    for jname in _KEY_JOINTS:
        # exact match first
        if jname in dof_names:
            joint_indices[jname] = dof_names.index(jname)
        else:
            # case-insensitive partial match
            matches = [i for i, n in enumerate(dof_names)
                       if jname.lower() in n.lower()]
            if matches:
                joint_indices[jname] = matches[0]
                print(f"[TORQUE] '{jname}' → matched DOF '{dof_names[matches[0]]}' (idx {matches[0]})")
            else:
                print(f"[TORQUE] WARNING – joint '{jname}' not found. Skipping.")

    print(f"[TORQUE] Monitoring joints: {list(joint_indices.keys())}")

    # ── load checkpoint ───────────────────────────────────────────────────────
    print(f"[INFO] Loading model checkpoint from: {resume_path}")

    if agent_cfg.class_name == "OnPolicyRunner":
        runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None,
                                device=agent_cfg.device)
    elif agent_cfg.class_name == "DistillationRunner":
        runner = DistillationRunner(env, agent_cfg.to_dict(), log_dir=None,
                                    device=agent_cfg.device)
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

    # ── data buffers ──────────────────────────────────────────────────────────
    dt         = env.unwrapped.step_dt
    obs        = env.get_observations()
    timestep   = 0
    env_idx    = args_cli.plot_env
    plot_steps = args_cli.plot_steps

    timestamps: list[float] = []
    torque_logs: defaultdict[str, list] = defaultdict(list)
    action_logs: defaultdict[str, list] = defaultdict(list)

    print(f"[TORQUE] Recording env #{env_idx} for torque/action analysis.")

    # ── simulation loop ───────────────────────────────────────────────────────
    while timestep < plot_steps and simulation_app.is_running():
        start_time = time.time()

        with torch.inference_mode():
            actions = policy(obs)
            obs, _, dones, _ = env.step(actions)
            policy_nn.reset(dones)

        base_env = env.unwrapped
        robot    = base_env.scene["robot"]
        
        # applied_torque: shape (num_envs, n_dof)
        torques = robot.data.applied_torque[env_idx].cpu().numpy()
        current_actions = actions[env_idx].cpu().numpy()

        timestamps.append(timestep * dt)
        
        for jname, idx in joint_indices.items():
            torque_logs[jname].append(torques[idx])
            # Assuming actions order matches DOF order (common in Isaac Lab Locomotion)
            if idx < len(current_actions):
                action_logs[jname].append(current_actions[idx])

        if timestep % 100 == 0 and joint_indices:
            jname0 = next(iter(joint_indices))
            idx0   = joint_indices[jname0]
            print(f"[TORQUE] step={timestep:5d}/{plot_steps} | {jname0}: torque={torques[idx0]:.2f} Nm")

        timestep += 1

        sleep_time = dt - (time.time() - start_time)
        if args_cli.real_time and sleep_time > 0:
            time.sleep(sleep_time)

    env.close()

    if len(timestamps) < 2:
        print("[TORQUE] Not enough data to plot.")
        return

    t_arr = np.asarray(timestamps)
    save_dir = args_cli.save_plot or os.path.join(log_dir, "torque_plots")

    # ── Plot 1: Torques ───────────────────────────────────────────────────────
    plot_trajectories(t_arr, torque_logs, "Joint Applied Torques", "Torque [Nm]", "joint_torques.png", save_path=save_dir)

    # ── Plot 2: Actions ───────────────────────────────────────────────────────
    plot_trajectories(t_arr, action_logs, "Policy Actions", "Action value", "policy_actions.png", save_path=save_dir)

    # ── Export: CSV ───────────────────────────────────────────────────────────
    export_to_csv(t_arr, torque_logs, action_logs, save_dir=save_dir)

    print(f"[TORQUE] All plots and data saved to: {save_dir}")

if __name__ == "__main__":
    main()
    simulation_app.close()
