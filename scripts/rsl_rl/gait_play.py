# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

"""
gait_play.py  Run a trained RSL-RL policy and produce joint-trajectory
               analysis plots when the simulation ends:

  1. Joint Angle vs. Time    position of key joints over several step cycles.
  2. Phase Portrait (q̇ vs q)  joint velocity vs. joint angle.
               A stable walking gait forms a closed repeating loop
               (a limit cycle) on this graph.
  3. Foot Clearance           world-Z height of each swing foot over time.
               Proves the foot is not dragging on the ground; stance
               phases are shaded and toe-drag samples are flagged in red.

Usage (identical to loc_play.py):
    python gait_play.py --task Isaac-Velocity-Flat-Hanu-A4-v0 \\
                        --checkpoint /path/to/model.pt \\
                        --num_envs 1 \\
                        [--plot_steps 2000] \\
                        [--real-time] [--video] [--video_length 200]
"""

# ──────────────────────────────────────────────────────────────────────────────
# 1.  Argument parsing & AppLauncher  (must happen before any Isaac imports)
# ──────────────────────────────────────────────────────────────────────────────
import argparse
import sys

from isaaclab.app import AppLauncher

import cli_args  # isort: skip  (local helper in the same directory)

parser = argparse.ArgumentParser(
    description="Play an RSL-RL policy and plot joint-trajectory / phase-portrait."
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
                    help="Stop simulation and plot after N steps (0 = run until window closed).")
parser.add_argument("--plot_env",     type=int, default=0,
                    help="Which environment index to record for plotting (default: 0).")
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
# 2.  Python / Isaac imports  (after AppLauncher)
# ──────────────────────────────────────────────────────────────────────────────
import os
import time
import csv
from collections import defaultdict

import gymnasium as gym
import numpy as np
import torch
import matplotlib
matplotlib.use("Agg")   # headless-safe; switch to TkAgg/Qt5Agg for interactive
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.collections import LineCollection
from matplotlib.colors import Normalize

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
# 3.  Joint & foot definitions
# ──────────────────────────────────────────────────────────────────────────────
# Joints to monitor.  Edit these names to match your robot's joint names.
# The script will search for them in the articulation DOF name list
# and handle gracefully if any are absent.
_KEY_JOINTS = [
    # "L_hip_pitch",
    # "R_hip_pitch",
    # "L_knee_pitch",
    # "R_knee_pitch",
    "L_shoulder_pitch",
    "R_shoulder_pitch",
]

# Color map: one colour per joint
_JOINT_COLORS = {
    # "L_hip_pitch":      "#1f77b4",
    # "R_hip_pitch":      "#ff7f0e",
    # "L_knee_pitch":     "#2ca02c",
    # "R_knee_pitch":     "#d62728",
    "L_shoulder_pitch": "#1f77b4",
    "R_shoulder_pitch": "#ff7f0e",
}

# ── Foot body names ────────────────────────────────────────────────────────────
# The script tries exact match first, then case-insensitive partial match.
_FOOT_BODIES = ["L_foot", "R_foot"]

_FOOT_COLORS = {
    "L_foot": "#1f77b4",   # blue  – left
    "R_foot": "#d62728",   # red – right
}

# A foot is considered "in stance" when its world-Z height is at or below this
# threshold (metres).  Anything above is classified as swing.
_STANCE_THRESHOLD_M = 0.02   # 2 cm

# A sample is flagged as toe-drag when the foot is classified as SWING but its
# height is at or below this limit (can equal _STANCE_THRESHOLD_M or be lower).
_TOE_DRAG_THRESHOLD_M = 0.005  # 5 mm

_PALETTE = {
    "bg":    "#FFFFFF",
    "fg":    "#212121",
    "grid":  "#BDBDBD",
    "panel": "#FFFFFF",  # white panel for cleaner look
}


# ──────────────────────────────────────────────────────────────────────────────
# 4.  Plotting helpers
# ──────────────────────────────────────────────────────────────────────────────

def _styled_ax(ax):
    """Apply common dark / light style to an axes."""
    ax.set_facecolor(_PALETTE["panel"])
    ax.tick_params(colors=_PALETTE["fg"], labelsize=10)
    ax.yaxis.label.set_color(_PALETTE["fg"])
    ax.xaxis.label.set_color(_PALETTE["fg"])
    ax.title.set_color(_PALETTE["fg"])
    for spine in ax.spines.values():
        spine.set_edgecolor(_PALETTE["grid"])
        spine.set_alpha(0.5)
    ax.grid(True, color=_PALETTE["grid"], linewidth=0.8, linestyle=":", alpha=0.7)


def plot_joint_trajectories(
    timestamps:  np.ndarray,           # (T,)
    pos_logs:    dict,                  # {joint_name: np.ndarray (T,)}  [rad]
    save_path:   str = "",
):
    """
    Plot 1 – Joint Angle vs. Time.

    Shows the angular position of each key joint over the full recording.
    Each joint is drawn in a separate row so the waveforms are easy to compare.
    """
    joints   = [j for j in _KEY_JOINTS if j in pos_logs]
    n        = len(joints)
    if n == 0:
        print("[GAIT] No joint position data to plot (trajectory).")
        return

    fig = plt.figure(figsize=(14, 4.0 * n), facecolor=_PALETTE["bg"])
    fig.suptitle(
        "Joint Angle Trajectories  (position vs. time)",
        color=_PALETTE["fg"], fontsize=16, fontweight="bold", y=0.99,
    )

    gs = gridspec.GridSpec(n, 1, hspace=0.55, left=0.08, right=0.97,
                           top=0.93, bottom=0.06)

    for row, jname in enumerate(joints):
        q   = pos_logs[jname]          # (T,)
        col = _JOINT_COLORS.get(jname, "#333333")

        ax = fig.add_subplot(gs[row])
        _styled_ax(ax)

        ax.plot(timestamps, np.degrees(q), color=col, linewidth=2.0, label=jname)

        # Highlight every step-cycle: shade alternating half-periods
        # (Simple zero-crossing of a reference joint for visual guide)
        ax.axhline(0, color=_PALETTE["grid"], linewidth=1.0, linestyle=":")

        ax.set_title(f"{jname.replace('_', ' ')}", fontsize=14, fontweight='bold', pad=8, color=_PALETTE["fg"])
        ax.set_ylabel("Joint angle [°]", fontsize=12)
        if row == n - 1:
            ax.set_xlabel("Time  [s]", fontsize=12)

        # Annotate range
        qd = np.degrees(q)
        ax.text(
            0.01, 0.95,
            f"range: [{qd.min():.1f}°, {qd.max():.1f}°]  |  peak-to-peak: {qd.ptp():.1f}°",
            transform=ax.transAxes, fontsize=10, verticalalignment="top",
            color="#455A64",
            bbox=dict(boxstyle="round,pad=0.25", facecolor="#FFFFFF",
                      edgecolor=_PALETTE["grid"], alpha=0.9),
        )

        ax.legend(loc="upper right", fontsize=11,
                  facecolor="#FFFFFF", edgecolor=_PALETTE["grid"],
                  labelcolor=_PALETTE["fg"])

        # ── Aggressive Outlier cropping: focus on 5th-95th percentile ──────
        # y_lims = np.percentile(qd, [2, 98])
        # margin = max((y_lims[1] - y_lims[0]) * 0.10, 1.0)
        # ax.set_ylim(y_lims[0] - margin, y_lims[1] + margin)

    _save_or_show(fig, save_path, "joint_trajectories.png")


def plot_phase_portraits(
    pos_logs: dict,   # {joint_name: np.ndarray (T,)}  [rad]
    vel_logs: dict,   # {joint_name: np.ndarray (T,)}  [rad/s]
    save_path: str = "",
):
    """
    Plot 2 – Phase Portrait  (q̇  vs  q).

    A stable walking gait traces a closed, repeating loop (limit cycle) on this
    graph.  The trajectory is colour-mapped by time so you can see how it evolves
    over successive step cycles.
    """
    joints = [j for j in _KEY_JOINTS if j in pos_logs and j in vel_logs]
    n      = len(joints)
    if n == 0:
        print("[GAIT] No joint data to plot (phase portrait).")
        return

    # Two columns layout: left column = hip joints, right = knee joints
    cols = 2
    rows = (n + 1) // cols

    fig, axes = plt.subplots(rows, cols,
                             figsize=(7 * cols, 6 * rows),
                             facecolor=_PALETTE["bg"])
    fig.suptitle(
        "Phase Portraits  (q̇  vs  q)  –  Limit-Cycle Analysis",
        color=_PALETTE["fg"], fontsize=16, fontweight="bold", y=1.01,
    )
    axes_flat = np.array(axes).flatten()

    for idx, jname in enumerate(joints):
        q    = np.degrees(pos_logs[jname])     # position [°]
        qdot = np.degrees(vel_logs[jname])     # velocity [°/s]
        col  = _JOINT_COLORS.get(jname, "#333333")

        ax = axes_flat[idx]
        _styled_ax(ax)

        T = len(q)
        # ── colour-mapped line: colour = normalised time ────────────────────
        points  = np.array([q, qdot]).T.reshape(-1, 1, 2)
        segs    = np.concatenate([points[:-1], points[1:]], axis=1)
        norm    = Normalize(vmin=0, vmax=T - 2)
        lc      = LineCollection(segs, cmap="plasma", norm=norm,
                                 linewidth=1.4, alpha=0.85)
        lc.set_array(np.arange(T - 1))
        ax.add_collection(lc)

        # start / end markers
        ax.plot(q[0],  qdot[0],  "o", color="#00E676", markersize=7,
                zorder=5, label="Start")
        ax.plot(q[-1], qdot[-1], "s", color="#FF1744", markersize=7,
                zorder=5, label="End")

        ax.autoscale()
        ax.set_aspect("auto")

        cbar = fig.colorbar(lc, ax=ax, pad=0.02)
        cbar.set_label("Time step", fontsize=8, color=_PALETTE["fg"])
        cbar.ax.tick_params(colors=_PALETTE["fg"])

        ax.set_title(f"{jname.replace('_', ' ')}  –  phase portrait",
                     fontsize=14, fontweight='bold', pad=8, color=_PALETTE["fg"])
        ax.set_xlabel("Joint angle  [°]", fontsize=12)
        ax.set_ylabel("Joint velocity  [°/s]", fontsize=12)

        # Annotate with enclosure area (crude measure of gait energy)
        try:
            from scipy.spatial import ConvexHull
            pts  = np.column_stack([q, qdot])
            hull = ConvexHull(pts)
            area = hull.volume   # 2-D hull: "volume" = area
            ax.text(
                0.02, 0.97,
                f"Convex-hull area: {area:.2f} °²/s",
                transform=ax.transAxes, fontsize=10, verticalalignment="top",
                color="#37474F",
                bbox=dict(boxstyle="round,pad=0.25", facecolor="#FFFFFF",
                          edgecolor=_PALETTE["grid"], alpha=0.9),
            )
        except Exception:
            pass   # scipy not available or hull failed

        # ── Aggressive Outlier cropping: 2th-98th percentile ───────────────
        # q_lims = np.percentile(q, [2, 98])
        # qd_lims = np.percentile(qdot, [2, 98])
        # q_margin = max((q_lims[1] - q_lims[0]) * 0.1, 1.0)
        # qd_margin = max((qd_lims[1] - qd_lims[0]) * 0.1, 5.0)
        # ax.set_xlim(q_lims[0] - q_margin, q_lims[1] + q_margin)
        # ax.set_ylim(qd_lims[0] - qd_margin, qd_lims[1] + qd_margin)

        ax.legend(loc="lower right", fontsize=11,
                  facecolor="#FFFFFF", edgecolor=_PALETTE["grid"],
                  labelcolor=_PALETTE["fg"])

    # Hide unused subplots
    for idx in range(len(joints), len(axes_flat)):
        axes_flat[idx].set_visible(False)

    plt.tight_layout()
    _save_or_show(fig, save_path, "phase_portraits.png")


def plot_foot_clearance(
    timestamps:  np.ndarray,              # (T,)  [s]
    foot_logs:   dict,                    # {foot_name: np.ndarray (T,)}  [m]
    save_path:   str = "",
):
    """
    Plot 3  Foot Clearance (world-Z height vs. time).

    For each foot the plot shows:
      • The raw height signal (solid coloured line).
      • Grey shading during stance phases (height ≤ _STANCE_THRESHOLD_M).
      • Red scatter markers on any swing sample where height falls below
        _TOE_DRAG_THRESHOLD_M  (toe-drag warning).
      • A dashed zero line (ground level).
      • Annotation with minimum swing clearance and toe-drag count.

    A clean gait should show smooth arcs above zero during swing and should
    have NO red markers.
    """
    feet = [f for f in _FOOT_BODIES if f in foot_logs]
    if not feet:
        # Try any available foot
        feet = list(foot_logs.keys())
    if not feet:
        print("[GAIT] No foot clearance data to plot.")
        return

    n   = len(feet)
    fig = plt.figure(figsize=(14, 4.0 * n), facecolor=_PALETTE["bg"])
    fig.suptitle(
        "Foot Clearance  (swing foot height vs. time)",
        color=_PALETTE["fg"], fontsize=16, fontweight="bold", y=0.99,
    )

    gs = gridspec.GridSpec(n, 1, hspace=0.55, left=0.08, right=0.97,
                           top=0.93, bottom=0.06)

    for row, fname in enumerate(feet):
        z   = foot_logs[fname]                          # (T,) world-Z [m]
        col = _FOOT_COLORS.get(fname, "#333333")

        ax = fig.add_subplot(gs[row])
        _styled_ax(ax)

        # ── ground / stance reference ─────────────────────────────────────
        ax.axhline(0.0, color="#424242", linewidth=1.0, linestyle="-",
                   label="Ground (z = 0)", zorder=1)
        # ax.axhline(_STANCE_THRESHOLD_M, color="#78909C", linewidth=0.8,
        #            linestyle=":", label=f"Stance threshold ({_STANCE_THRESHOLD_M*100:.0f} cm)",
        #            zorder=1)

        # ── shade stance phases ───────────────────────────────────────────
        in_stance = z <= _STANCE_THRESHOLD_M
        # find contiguous stance spans
        stance_starts = np.where(np.diff(in_stance.astype(int)) == 1)[0] + 1
        stance_ends   = np.where(np.diff(in_stance.astype(int)) == -1)[0] + 1
        # handle edges
        if in_stance[0]:
            stance_starts = np.concatenate([[0], stance_starts])
        if in_stance[-1]:
            stance_ends = np.concatenate([stance_ends, [len(z)]])
        for s, e in zip(stance_starts, stance_ends):
            ax.axvspan(timestamps[s], timestamps[min(e, len(timestamps)-1)],
                       alpha=0.18, color="#FFFFFF", zorder=0)

        # ── main height trace ─────────────────────────────────────────────
        ax.plot(timestamps, z * 100, color=col, linewidth=2.0,
                label=fname, zorder=3)   # convert m → cm for readability

        # ── toe-drag detection: swing phase but height ≤ drag threshold ──
        is_swing    = ~in_stance
        is_toe_drag = is_swing & (z <= _TOE_DRAG_THRESHOLD_M)
        drag_count  = int(is_toe_drag.sum())
        if drag_count:
            ax.scatter(
                timestamps[is_toe_drag], z[is_toe_drag] * 100,
                color="#D32F2F", s=25, zorder=5,
                label=f"Toe drag ✗ ({drag_count} samples)",
            )

        # ── swing minimum clearance annotation ───────────────────────────
        swing_z = z[is_swing] if is_swing.any() else np.array([0.0])
        min_swing_cm = float(swing_z.min()) * 100
        max_swing_cm = float(swing_z.max()) * 100

        drag_label = "⚠ TOE DRAG" if drag_count else "✓ No toe drag"
        drag_color = "#B71C1C" if drag_count else "#1B5E20"
        face_color = "#FFFFFF" if drag_count else "#FFFFFF" # Keep white background
        edge_color = "#EF9A9A" if drag_count else "#A5D6A7"

        ax.text(
            0.01, 0.96,
            (f"Min swing clearance: {min_swing_cm:.1f} cm  |  "
             f"Max: {max_swing_cm:.1f} cm  |  {drag_label}"),
            transform=ax.transAxes, fontsize=10, verticalalignment="top",
            color=drag_color,
            bbox=dict(boxstyle="round,pad=0.3", facecolor=face_color,
                      edgecolor=edge_color, alpha=0.9),
        )

        ax.set_title(f"{fname.replace('_', ' ')}  –  foot clearance",
                     fontsize=14, fontweight='bold', pad=8, color=_PALETTE["fg"])
        ax.set_ylabel("Height  [cm]", fontsize=12)
        if row == n - 1:
            ax.set_xlabel("Time  [s]", fontsize=12)

        ax.legend(loc="upper right", fontsize=11,
                  facecolor="#FFFFFF", edgecolor=_PALETTE["grid"],
                  labelcolor=_PALETTE["fg"])

        # ── Aggressive Outlier cropping: 5th-95th percentile ───────────────
        z_cm = z * 100
        low  = np.percentile(z_cm, 2)
        high = np.percentile(z_cm, 98)
        margin = 0.2
        ax.set_ylim(low - margin, high + margin)

    _save_or_show(fig, save_path, "foot_clearance.png")


def _save_or_show(fig, save_dir: str, filename: str):
    """Save figure to *save_dir/filename*, or fall back to script directory."""
    if save_dir:
        os.makedirs(save_dir, exist_ok=True)
        out = os.path.join(save_dir, filename)
    else:
        out = os.path.join(os.path.dirname(__file__), filename)

    fig.savefig(out, dpi=150, bbox_inches="tight", facecolor=fig.get_facecolor())
    print(f"[GAIT] Saved → {out}")
    plt.close(fig)


def export_to_csv(
    timestamps: np.ndarray,
    pos_np:     dict[str, np.ndarray],
    vel_np:     dict[str, np.ndarray],
    foot_np:    dict[str, np.ndarray],
    save_dir:   str
):
    """Save all tracked data to a single consolidated CSV file."""
    if not os.path.exists(save_dir):
        os.makedirs(save_dir, exist_ok=True)
    
    filename = os.path.join(save_dir, "gait_data.csv")
    
    # Collect all headers
    headers = ["time"]
    joint_names = sorted(pos_np.keys())
    for j in joint_names:
        headers.append(f"{j}_pos_rad")
        headers.append(f"{j}_vel_radps")
    
    foot_names = sorted(foot_np.keys())
    for f in foot_names:
        headers.append(f"{f}_z_m")
    
    # Write rows
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(headers)
        
        for i in range(len(timestamps)):
            row = [timestamps[i]]
            for j in joint_names:
                row.append(pos_np[j][i])
                row.append(vel_np[j][i])
            for fn in foot_names:
                row.append(foot_np[fn][i])
            writer.writerow(row)
            
    print(f"[GAIT] Data exported to → {filename}")


# ──────────────────────────────────────────────────────────────────────────────
# 5.  Velocity estimation helper
# ──────────────────────────────────────────────────────────────────────────────

def _finite_diff(q_arr: np.ndarray, dt: float) -> np.ndarray:
    """
    Central-difference velocity estimate from position array.
    Edges use forward/backward difference.
    """
    qdot = np.empty_like(q_arr)
    qdot[1:-1] = (q_arr[2:] - q_arr[:-2]) / (2.0 * dt)
    qdot[0]    = (q_arr[1]  - q_arr[0])  / dt
    qdot[-1]   = (q_arr[-1] - q_arr[-2]) / dt
    return qdot


# ──────────────────────────────────────────────────────────────────────────────
# 6.  Main
# ──────────────────────────────────────────────────────────────────────────────
@hydra_task_config(args_cli.task, args_cli.agent)
def main(
    env_cfg:   ManagerBasedRLEnvCfg | DirectRLEnvCfg | DirectMARLEnvCfg,
    agent_cfg: RslRlBaseRunnerCfg,
):
    """Load a trained checkpoint, run the robot, then plot joint trajectories."""

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
    print(f"[INFO] Loading experiment from directory: {log_root_path}")

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
            "video_folder":   os.path.join(log_dir, "videos", "play"),
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
    print("\n========== OBS DEBUG ==========")
    if hasattr(obs, "keys"):
        print("keys:", list(obs.keys()))
        if "policy" in obs:
            print("obs['policy'].shape:", obs["policy"].shape)
    else:
        print("obs.shape:", obs.shape)
    print("================================\n", flush=True)

    # ── discover joint indices ─────────────────────────────────────────────────
    base_env   = env.unwrapped
    robot      = base_env.scene["robot"]
    dof_names  = robot.data.joint_names          # list[str]

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
                print(f"[GAIT] '{jname}' → matched DOF '{dof_names[matches[0]]}' (idx {matches[0]})")
            else:
                print(f"[GAIT] WARNING – joint '{jname}' not found in DOF list. Skipping.")

    print(f"[GAIT] Monitoring joints: {list(joint_indices.keys())}")
    print(f"[GAIT] All DOF names: {dof_names}\n", flush=True)

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

    normalizer = (
        getattr(policy_nn, "actor_obs_normalizer", None)
        or getattr(policy_nn, "student_obs_normalizer", None)
    )

    # export_dir = os.path.join(log_dir, "exported")
    # export_policy_as_jit( policy_nn, normalizer=normalizer,
    #                       path=export_dir, filename="policy.pt")
    # export_policy_as_onnx(policy_nn, normalizer=normalizer,
    #                       path=export_dir, filename="policy.onnx")

    # ── discover foot body indices ─────────────────────────────────────────────
    body_names = list(robot.data.body_names)   # list[str]
    foot_indices: dict[str, int] = {}
    for fname in _FOOT_BODIES:
        if fname in body_names:
            foot_indices[fname] = body_names.index(fname)
        else:
            matches = [i for i, n in enumerate(body_names)
                       if fname.lower() in n.lower()]
            if matches:
                foot_indices[fname] = matches[0]
                print(f"[GAIT] '{fname}' → matched body '{body_names[matches[0]]}' "
                      f"(idx {matches[0]})")
            else:
                print(f"[GAIT] WARNING – foot body '{fname}' not found. Skipping.")
    print(f"[GAIT] Monitoring feet: {list(foot_indices.keys())}")
    print(f"[GAIT] All body names: {body_names}\n", flush=True)

    # ── data buffers ──────────────────────────────────────────────────────────
    dt         = env.unwrapped.step_dt
    obs        = env.get_observations()
    timestep   = 0
    env_idx    = args_cli.plot_env
    plot_steps = args_cli.plot_steps

    timestamps: list[float] = []
    pos_logs:   defaultdict[str, list] = defaultdict(list)  # joint → [q_rad, ...]
    vel_logs:   defaultdict[str, list] = defaultdict(list)  # joint → [qdot_rad/s, ...]
    foot_logs:  defaultdict[str, list] = defaultdict(list)  # foot  → [z_m, ...]

    print(f"[GAIT] Recording env #{env_idx} for joint gait analysis.")
    if plot_steps:
        print(f"[GAIT] Will run for {plot_steps} steps then plot.\n")

    # ── simulation loop ───────────────────────────────────────────────────────
    while simulation_app.is_running():
        start_time = time.time()

        with torch.inference_mode():
            actions = policy(obs)
            obs, _, dones, _ = env.step(actions)
            policy_nn.reset(dones)

        base_env = env.unwrapped
        robot    = base_env.scene["robot"]

        # joint positions [rad] and velocities [rad/s] – shape (num_envs, n_dof)
        q_all    = robot.data.joint_pos
        qdot_all = robot.data.joint_vel

        # foot world-Z positions – body_pos_w shape: (num_envs, n_bodies, 3)
        body_pos_w = robot.data.body_pos_w   # (num_envs, n_bodies, 3)

        timestamps.append(timestep * dt)

        for jname, idx in joint_indices.items():
            pos_logs[jname].append(q_all[env_idx, idx].item())
            vel_logs[jname].append(qdot_all[env_idx, idx].item())

        for fname, bidx in foot_indices.items():
            # world-Z of the foot body centre
            z_world = body_pos_w[env_idx, bidx, 2].item()
            foot_logs[fname].append(z_world)

        # console log every 200 steps
        if timestep % 200 == 0 and joint_indices:
            jname0 = next(iter(joint_indices))
            idx0   = joint_indices[jname0]
            foot_str = "  ".join(
                f"{fn}: z={foot_logs[fn][-1]*100:5.1f} cm"
                for fn in foot_indices
            )
            print(f"[GAIT] step={timestep:5d} | "
                  f"{jname0}: q={np.degrees(q_all[env_idx, idx0].item()):7.2f}°  "
                  f"qdot={np.degrees(qdot_all[env_idx, idx0].item()):8.2f}°/s  "
                  f"| {foot_str}")

        timestep += 1

        if args_cli.video and timestep >= args_cli.video_length:
            break
        if plot_steps and timestep >= plot_steps:
            break

        sleep_time = dt - (time.time() - start_time)
        if args_cli.real_time and sleep_time > 0:
            time.sleep(sleep_time)

    env.close()

    # ── convert buffers to numpy arrays ──────────────────────────────────────
    if len(timestamps) < 2:
        print("[GAIT] Not enough data to plot.")
        return

    t_arr = np.asarray(timestamps, dtype=np.float32)

    pos_np:  dict[str, np.ndarray] = {
        j: np.asarray(v, dtype=np.float32) for j, v in pos_logs.items()
    }
    # Prefer physics velocities; fall back to finite-difference
    vel_np: dict[str, np.ndarray] = {}
    for jname in pos_np:
        raw_vel = np.asarray(vel_logs[jname], dtype=np.float32)
        if np.allclose(raw_vel, 0.0):
            print(f"[GAIT] '{jname}': physics velocity is all-zero, using finite-diff.")
            vel_np[jname] = _finite_diff(pos_np[jname], float(dt))
        else:
            vel_np[jname] = raw_vel

    # ── convert foot logs ─────────────────────────────────────────────────────
    foot_np: dict[str, np.ndarray] = {
        f: np.asarray(v, dtype=np.float32) for f, v in foot_logs.items()
    }

    # Normalise foot heights so that the lowest recorded stance contact = 0.
    # This removes any constant offset from terrain height or robot spawn height.
    # for fname, z in foot_np.items():
    #     stance_mask = z <= (z.min() + _STANCE_THRESHOLD_M)
    #     if stance_mask.any():
    #         ground_ref = float(z[stance_mask].mean())
    #     else:
    #         ground_ref = float(z.min())
    #     foot_np[fname] = z - ground_ref
    #     print(f"[GAIT] '{fname}' ground ref = {ground_ref:.4f} m  "
    #           f"(normalised so stance contact ≈ 0)")

    # ── print summary ─────────────────────────────────────────────────────────
    print("\n========== JOINT SUMMARY ==========")
    for jname, q in pos_np.items():
        qd = np.degrees(q)
        print(f"  {jname:20s}  range=[{qd.min():6.1f}°, {qd.max():6.1f}°]"
              f"  peak-to-peak={qd.ptp():.1f}°")
    print("====================================\n")

    print("========== FOOT CLEARANCE SUMMARY ==========")
    for fname, z in foot_np.items():
        swing_mask = z > _STANCE_THRESHOLD_M
        swing_z    = z[swing_mask] if swing_mask.any() else np.array([0.0])
        drag_count = int((swing_mask & (z <= _TOE_DRAG_THRESHOLD_M)).sum())
        status     = "✓ OK" if drag_count == 0 else f"✗ TOE DRAG ({drag_count} samples)"
        print(f"  {fname:10s}  min_swing={swing_z.min()*100:5.1f} cm  "
              f"max_swing={swing_z.max()*100:5.1f} cm  {status}")
    print("============================================\n", flush=True)

    # ── determine output directory ────────────────────────────────────────────
    save_dir = args_cli.save_plot or os.path.join(log_dir, "gait_plots")

    # ── Plot 1: Joint Trajectory (angle vs. time) ─────────────────────────────
    plot_joint_trajectories(t_arr, pos_np, save_path=save_dir)

    # ── Plot 2: Phase Portrait (q̇ vs q)  ─────────────────────────────────────
    plot_phase_portraits(pos_np, vel_np, save_path=save_dir)

    # ── Plot 3: Foot Clearance ────────────────────────────────────────────────
    if foot_np:
        plot_foot_clearance(t_arr, foot_np, save_path=save_dir)
    else:
        print("[GAIT] Skipping foot clearance plot (no foot bodies found).")

    # ── Export 4: CSV Data ────────────────────────────────────────────────────
    export_to_csv(t_arr, pos_np, vel_np, foot_np, save_dir=save_dir)

    print(f"[GAIT] All plots and data saved to: {save_dir}")


# ──────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    main()
    simulation_app.close()
