# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

"""
bal_play.py  Run a trained RSL-RL policy with command velocity FORCED to 0 m/s
              and produce balance evaluation plots when the simulation ends:

  1. Projected Gravity (x, y) vs. Time
       Body-frame gravity projection reveals how much the robot tilts
       forward/backward (gx) and sideways (gy) during stationary balance.
       A perfect balance → both signals ≈ 0.

  2. Joint Angle vs. Time (KEY_JOINTS)
       Tracks selected joint positions to see the postural adjustments
       the policy makes while standing still.

  3. Gravity 2-D Scatter  (gx vs. gy)
       Plots the gravity projection cloud in the XY plane, colour-mapped
       by time.  A tight cluster near origin = good balance.
       RMS radius is annotated as a single-number balance score.

Usage (identical to gait_play.py):
    python bal_play.py --task Isaac-Velocity-Flat-Hanu-A4-v0 \\
                       --checkpoint /path/to/model.pt \\
                       --num_envs 1 \\
                       [--plot_steps 1000] \\
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
    description="Play an RSL-RL policy at 0 m/s and evaluate balance via projected gravity."
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
)

import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils       import get_checkpoint_path
from isaaclab_tasks.utils.hydra import hydra_task_config

import hanu_lab.tasks  # noqa: F401


# ──────────────────────────────────────────────────────────────────────────────
# 3.  Joint definitions  (balance-relevant joints: hip & shoulder for sway)
# ──────────────────────────────────────────────────────────────────────────────
# Edit this list to focus on whichever joints dominate balance corrections.
_KEY_JOINTS = [
    # "L_hip_pitch",
    # "R_hip_pitch",
    # "L_hip_roll",
    # "R_hip_roll",
    "L_shoulder_pitch",
    "R_shoulder_pitch",
]

# Colour per joint
_JOINT_COLORS = {
    # "L_hip_pitch": "#1f77b4",
    # "R_hip_pitch": "#ff7f0e",
    # "L_hip_roll":  "#2ca02c",
    # "R_hip_roll":  "#d62728",
    "L_shoulder_pitch": "#9467bd",
    "R_shoulder_pitch": "#8c564b",
}

# Colour for the two gravity components
_GRAV_COLORS = {
    "gx": "#1E88E5",  # lateral tilt  (blue)  – robot faces Y-axis → body-X = sideways
    "gy": "#E53935",  # fwd/back tilt (red)   – robot faces Y-axis → body-Y = forward
}

_PALETTE = {
    "bg":    "#FFFFFF",
    "fg":    "#212121",
    "grid":  "#BDBDBD",
    "panel": "#FFFFFF",
}


# ──────────────────────────────────────────────────────────────────────────────
# 4.  Plotting helpers
# ──────────────────────────────────────────────────────────────────────────────

def _styled_ax(ax):
    """Apply common style to an axes."""
    ax.set_facecolor(_PALETTE["panel"])
    ax.tick_params(colors=_PALETTE["fg"], labelsize=10)
    ax.yaxis.label.set_color(_PALETTE["fg"])
    ax.xaxis.label.set_color(_PALETTE["fg"])
    ax.title.set_color(_PALETTE["fg"])
    for spine in ax.spines.values():
        spine.set_edgecolor(_PALETTE["grid"])
        spine.set_alpha(0.5)
    ax.grid(True, color=_PALETTE["grid"], linewidth=0.8, linestyle=":", alpha=0.7)


def _save_or_show(fig, save_dir: str, filename: str):
    """Save figure to *save_dir/filename*, or fall back to script directory."""
    if save_dir:
        os.makedirs(save_dir, exist_ok=True)
        out = os.path.join(save_dir, filename)
    else:
        out = os.path.join(os.path.dirname(__file__), filename)

    fig.savefig(out, dpi=150, bbox_inches="tight", facecolor=fig.get_facecolor())
    print(f"[BAL] Saved → {out}")
    plt.close(fig)


# ──────────────────────────────────────────────────────────────────────────────
# 4a.  Plot 1 – Projected Gravity (x, y) vs. Time
# ──────────────────────────────────────────────────────────────────────────────

def plot_projected_gravity(
    timestamps: np.ndarray,   # (T,)
    gx: np.ndarray,           # (T,)  lateral tilt  (body-X, robot faces Y)
    gy: np.ndarray,           # (T,)  fwd/back tilt (body-Y, robot faces Y)
    save_path: str = "",
):
    """
    Plot 1 – Projected Gravity components vs. Time.

    Robot faces world Y-axis → body-frame interpretation:
      • gx (body-X) ≈ 0  → robot leans neither left nor right  (lateral)
      • gy (body-Y) ≈ 0  → robot leans neither forward nor backward
    Large deviations indicate poor balance.  The ideal balance shows both
    signals tightly clustered around zero.
    """
    fig = plt.figure(figsize=(14, 7), facecolor=_PALETTE["bg"])
    fig.suptitle(
        "Projected Gravity vs. Time  (Balance Evaluation @ 0 m/s)",
        color=_PALETTE["fg"], fontsize=16, fontweight="bold", y=0.99,
    )

    gs = gridspec.GridSpec(2, 1, hspace=0.50, left=0.08, right=0.97,
                           top=0.93, bottom=0.07)

    # ── gx (forward/backward tilt) ──────────────────────────────────────────
    ax_gx = fig.add_subplot(gs[0])
    _styled_ax(ax_gx)
    ax_gx.plot(timestamps, gx, color=_GRAV_COLORS["gx"], linewidth=1.8,
               label="gx  (lateral tilt  left / right)")
    ax_gx.axhline(0, color=_PALETTE["grid"], linewidth=1.0, linestyle="--")

    gx_rms = float(np.sqrt(np.mean(gx ** 2)))
    ax_gx.text(
        0.01, 0.95,
        f"RMS = {gx_rms:.4f}  |  range: [{gx.min():.4f}, {gx.max():.4f}]",
        transform=ax_gx.transAxes, fontsize=10, verticalalignment="top",
        color="#455A64",
        bbox=dict(boxstyle="round,pad=0.25", facecolor="#FFFFFF",
                  edgecolor=_PALETTE["grid"], alpha=0.9),
    )
    ax_gx.set_title("Gravity X  (lateral lean  left / right)", fontsize=14,
                    fontweight="bold", pad=8, color=_PALETTE["fg"])
    ax_gx.set_ylabel("Projected gravity gx  [unit]", fontsize=12)
    ax_gx.legend(loc="upper right", fontsize=11,
                 facecolor="#FFFFFF", edgecolor=_PALETTE["grid"],
                 labelcolor=_PALETTE["fg"])

    # Crop outliers for cleaner view
    gx_lims = np.percentile(gx, [2, 98])
    margin_x = max((gx_lims[1] - gx_lims[0]) * 0.15, 0.01)
    ax_gx.set_ylim(gx_lims[0] - margin_x, gx_lims[1] + margin_x)

    # ── gy (lateral tilt) ───────────────────────────────────────────────────
    ax_gy = fig.add_subplot(gs[1])
    _styled_ax(ax_gy)
    ax_gy.plot(timestamps, gy, color=_GRAV_COLORS["gy"], linewidth=1.8,
               label="gy  (forward / backward tilt)")
    ax_gy.axhline(0, color=_PALETTE["grid"], linewidth=1.0, linestyle="--")

    gy_rms = float(np.sqrt(np.mean(gy ** 2)))
    ax_gy.text(
        0.01, 0.95,
        f"RMS = {gy_rms:.4f}  |  range: [{gy.min():.4f}, {gy.max():.4f}]",
        transform=ax_gy.transAxes, fontsize=10, verticalalignment="top",
        color="#455A64",
        bbox=dict(boxstyle="round,pad=0.25", facecolor="#FFFFFF",
                  edgecolor=_PALETTE["grid"], alpha=0.9),
    )
    ax_gy.set_title("Gravity Y  (forward / backward lean)", fontsize=14,
                    fontweight="bold", pad=8, color=_PALETTE["fg"])
    ax_gy.set_ylabel("Projected gravity gy  [unit]", fontsize=12)
    ax_gy.set_xlabel("Time  [s]", fontsize=12)
    ax_gy.legend(loc="upper right", fontsize=11,
                 facecolor="#FFFFFF", edgecolor=_PALETTE["grid"],
                 labelcolor=_PALETTE["fg"])

    # Crop outliers
    gy_lims = np.percentile(gy, [2, 98])
    margin_y = max((gy_lims[1] - gy_lims[0]) * 0.15, 0.01)
    ax_gy.set_ylim(gy_lims[0] - margin_y, gy_lims[1] + margin_y)

    _save_or_show(fig, save_path, "balance_projected_gravity.png")


# ──────────────────────────────────────────────────────────────────────────────
# 4b.  Plot 2 – Joint Angle vs. Time
# ──────────────────────────────────────────────────────────────────────────────

def plot_joint_trajectories(
    timestamps: np.ndarray,        # (T,)
    pos_logs:   dict,              # {joint_name: np.ndarray (T,)}  [rad]
    save_path:  str = "",
):
    """
    Plot 2 – Joint Angle vs. Time.

    Displays the postural adjustments each key joint makes while the robot
    balances at 0 m/s.  Ideally the traces stay near their default pose
    with minimal oscillation.
    """
    joints = [j for j in _KEY_JOINTS if j in pos_logs]
    n      = len(joints)
    if n == 0:
        print("[BAL] No joint position data to plot (trajectory).")
        return

    fig = plt.figure(figsize=(14, 4.0 * n), facecolor=_PALETTE["bg"])
    fig.suptitle(
        "Joint Angle Trajectories  (@ 0 m/s – Balance Mode)",
        color=_PALETTE["fg"], fontsize=16, fontweight="bold", y=0.99,
    )

    gs = gridspec.GridSpec(n, 1, hspace=0.55, left=0.08, right=0.97,
                           top=0.93, bottom=0.06)

    for row, jname in enumerate(joints):
        q   = pos_logs[jname]            # (T,)  [rad]
        qd  = np.degrees(q)
        col = _JOINT_COLORS.get(jname, "#333333")

        ax = fig.add_subplot(gs[row])
        _styled_ax(ax)

        ax.plot(timestamps, qd, color=col, linewidth=1.8, label=jname)
        ax.axhline(0, color=_PALETTE["grid"], linewidth=1.0, linestyle=":")

        ax.set_title(f"{jname.replace('_', ' ')}", fontsize=14,
                     fontweight="bold", pad=8, color=_PALETTE["fg"])
        ax.set_ylabel("Joint angle  [°]", fontsize=12)
        if row == n - 1:
            ax.set_xlabel("Time  [s]", fontsize=12)

        # Annotate range & peak-to-peak
        ax.text(
            0.01, 0.95,
            f"range: [{qd.min():.1f}°, {qd.max():.1f}°]  |  "
            f"peak-to-peak: {qd.ptp():.2f}°  |  "
            f"RMS: {float(np.sqrt(np.mean(qd**2))):.2f}°",
            transform=ax.transAxes, fontsize=10, verticalalignment="top",
            color="#455A64",
            bbox=dict(boxstyle="round,pad=0.25", facecolor="#FFFFFF",
                      edgecolor=_PALETTE["grid"], alpha=0.9),
        )

        ax.legend(loc="upper right", fontsize=11,
                  facecolor="#FFFFFF", edgecolor=_PALETTE["grid"],
                  labelcolor=_PALETTE["fg"])

        # Outlier cropping
        q_lims = np.percentile(qd, [2, 98])
        margin = max((q_lims[1] - q_lims[0]) * 0.15, 1.0)
        ax.set_ylim(q_lims[0] - margin, q_lims[1] + margin)

    _save_or_show(fig, save_path, "balance_joint_trajectories.png")


# ──────────────────────────────────────────────────────────────────────────────
# 4c.  Plot 3 – Gravity 2-D Scatter  (gx vs. gy)
# ──────────────────────────────────────────────────────────────────────────────

def plot_gravity_scatter(
    gx: np.ndarray,   # (T,)  lateral tilt   (robot faces Y → body-X = sideways)
    gy: np.ndarray,   # (T,)  fwd/back tilt  (robot faces Y → body-Y = forward)
    save_path: str = "",
):
    """
    Plot 3 – Projected Gravity 2-D cloud  (gx vs. gy).

    Robot faces world Y-axis: gx = lateral lean, gy = forward/backward lean.
    Colour-mapped by time (early = violet, late = yellow).
    A tight cluster near (0, 0) means the robot holds its balance well.
    The RMS radius is annotated as a single, easy-to-compare balance score.
    """
    T = len(gx)

    fig, ax = plt.subplots(figsize=(8, 8), facecolor=_PALETTE["bg"])
    fig.suptitle(
        "Gravity Projection Cloud  (gx vs. gy)  –  Balance Score",
        color=_PALETTE["fg"], fontsize=16, fontweight="bold", y=1.00,
    )
    _styled_ax(ax)

    # Colour-mapped scatter: time → plasma colourmap
    sc = ax.scatter(gx, gy, c=np.arange(T), cmap="plasma",
                    s=6, alpha=0.6, linewidths=0)
    cbar = fig.colorbar(sc, ax=ax, pad=0.02)
    cbar.set_label("Time step", fontsize=10, color=_PALETTE["fg"])
    cbar.ax.tick_params(colors=_PALETTE["fg"])

    # Colour-mapped line: time progression
    points = np.array([gx, gy]).T.reshape(-1, 1, 2)
    segs   = np.concatenate([points[:-1], points[1:]], axis=1)
    norm   = Normalize(vmin=0, vmax=T - 2)
    lc     = LineCollection(segs, cmap="plasma", norm=norm,
                             linewidth=0.8, alpha=0.35)
    lc.set_array(np.arange(T - 1))
    ax.add_collection(lc)

    # Origin crosshairs
    ax.axhline(0, color="#424242", linewidth=1.0, linestyle="--", alpha=0.7)
    ax.axvline(0, color="#424242", linewidth=1.0, linestyle="--", alpha=0.7)

    # Start / end markers
    ax.plot(gx[0],  gy[0],  "o", color="#00E676", markersize=9,
            zorder=6, label="Start")
    ax.plot(gx[-1], gy[-1], "s", color="#FF1744", markersize=9,
            zorder=6, label="End")

    # RMS radius  (balance score)
    rms_radius = float(np.sqrt(np.mean(gx ** 2 + gy ** 2)))
    max_radius = float(np.sqrt(np.max(gx ** 2 + gy ** 2)))

    # Draw RMS circle
    theta = np.linspace(0, 2 * np.pi, 200)
    ax.plot(rms_radius * np.cos(theta), rms_radius * np.sin(theta),
            color="#FF9800", linewidth=2.0, linestyle="--",
            label=f"RMS radius = {rms_radius:.4f}")

    ax.text(
        0.02, 0.98,
        f"RMS radius: {rms_radius:.4f}  (↓ better)\n"
        f"Max radius: {max_radius:.4f}\n"
        f"N samples:  {T}",
        transform=ax.transAxes, fontsize=11, verticalalignment="top",
        color="#37474F",
        bbox=dict(boxstyle="round,pad=0.35", facecolor="#FFFFFF",
                  edgecolor=_PALETTE["grid"], alpha=0.92),
    )

    ax.autoscale()
    ax.set_aspect("equal")
    ax.set_xlabel("Gravity X  (lateral lean  left/right)  [unit]", fontsize=12)
    ax.set_ylabel("Gravity Y  (forward / backward lean)  [unit]", fontsize=12)
    ax.set_title("Balance Footprint  (0 m/s command  –  robot faces Y)", fontsize=14,
                 fontweight="bold", pad=10, color=_PALETTE["fg"])
    ax.legend(loc="lower right", fontsize=11,
              facecolor="#FFFFFF", edgecolor=_PALETTE["grid"],
              labelcolor=_PALETTE["fg"])

    plt.tight_layout()
    _save_or_show(fig, save_path, "balance_gravity_scatter.png")


# ──────────────────────────────────────────────────────────────────────────────
# 4d.  Combined Plot – KEY_JOINTS vs. Projected Gravity on the same time axis
# ──────────────────────────────────────────────────────────────────────────────

def plot_joints_and_gravity(
    timestamps: np.ndarray,        # (T,)
    gx: np.ndarray,                # (T,)  forward tilt
    gy: np.ndarray,                # (T,)  lateral tilt
    pos_logs: dict,                # {joint_name: np.ndarray (T,)} [rad]
    save_path: str = "",
):
    """
    Plot 4 – KEY_JOINTS vs. Projected Gravity (overlaid, dual Y-axis).

    Each panel = one KEY_JOINT.
      Left  Y-axis (coloured):  joint angle [°]
      Right Y-axis (grey):       projected gravity gx  (red dashed)
                                               and gy  (blue dashed)

    This lets you see directly whether joint motion leads, lags, or correlates
    with the body tilt captured by the gravity projection.
    """
    joints = [j for j in _KEY_JOINTS if j in pos_logs]
    n      = len(joints)
    if n == 0:
        print("[BAL] No joint data for combined gravity/joint plot.")
        return

    fig = plt.figure(figsize=(14, 5.0 * n), facecolor=_PALETTE["bg"])
    fig.suptitle(
        "JOINTS vs. Projected Gravity  (dual-axis, @ 0 m/s)",
        color=_PALETTE["fg"], fontsize=16, fontweight="bold", y=0.99,
    )

    gs = gridspec.GridSpec(n, 1, hspace=0.60, left=0.08, right=0.88,
                           top=0.93, bottom=0.06)

    for row, jname in enumerate(joints):
        qd  = np.degrees(pos_logs[jname])          # (T,) [°]
        col = _JOINT_COLORS.get(jname, "#333333")

        ax_j = fig.add_subplot(gs[row])
        _styled_ax(ax_j)

        # ── left axis: joint angle ────────────────────────────────────────────
        lj, = ax_j.plot(timestamps, qd, color=col, linewidth=2.0,
                        label=f"{jname}  [°]")
        ax_j.set_ylabel("Joint angle  [°]", fontsize=11,
                        color=col)
        ax_j.tick_params(axis="y", colors=col)
        # ax_j.yaxis.label.set_color(col)

        # Outlier crop for joint axis
        q_lims  = np.percentile(qd, [2, 98])
        q_marg  = max((q_lims[1] - q_lims[0]) * 0.20, 1.0)
        ax_j.set_ylim(q_lims[0] - q_marg, q_lims[1] + q_marg)

        # ── right axis: projected gravity ─────────────────────────────────────
        ax_g = ax_j.twinx()
        # Style the twin manually (twinx skips _styled_ax)
        ax_g.tick_params(axis="y", colors=_PALETTE["fg"], labelsize=9)
        ax_g.yaxis.label.set_color(_PALETTE["fg"])
        ax_g.set_facecolor("none")   # transparent – let left panel show through

        lgx, = ax_g.plot(timestamps, gx, color=_GRAV_COLORS["gx"],
                         linewidth=1.4, linestyle="-", alpha=0.85,
                         label="gx  (lateral)")
        lgy, = ax_g.plot(timestamps, gy, color=_GRAV_COLORS["gy"],
                         linewidth=1.4, linestyle="-", alpha=0.85,
                         label="gy  (fwd/back)")
        ax_g.axhline(0, color=_PALETTE["grid"], linewidth=0.8,
                     linestyle=":", alpha=0.6)
        ax_g.set_ylabel("Proj. gravity  [unit]", fontsize=10,
                        color=_PALETTE["fg"])

        # Crop gravity axis symmetrically around zero for visual clarity
        g_max = max(
            float(np.percentile(np.abs(gx), 98)),
            float(np.percentile(np.abs(gy), 98)),
            0.01,
        ) * 1.3
        ax_g.set_ylim(-g_max, g_max)

        # ── average-cycle annotation: 5–95th percentile band + mean line ──────
        p5,  p95  = np.percentile(qd, [5, 95])
        q_mean    = float(np.mean(qd))
        q_std     = float(np.std(qd))

        # Shaded band – drawn behind the signal trace (low zorder)
        band = ax_j.axhspan(p5, p95, alpha=0.12, color=col, zorder=0,
                            label=f"5-95th pct  [{p5:.2f}°, {p95:.2f}°]")

        # Mean reference line
        mean_line = ax_j.axhline(q_mean, color=col, linewidth=1.4,
                                 linestyle="--", alpha=0.75, zorder=2,
                                 label=f"mean = {q_mean:.2f}°")

        # Stats annotation box (bottom-left, away from corr box at top-left)
        ax_j.text(
            0.01, 0.06,
            f"mean: {q_mean:+.2f}°   std: {q_std:.2f}°\n"
            f"p5-p95: [{p5:.2f}°, {p95:.2f}°]   range: {p95 - p5:.2f}°",
            transform=ax_j.transAxes, fontsize=9, verticalalignment="bottom",
            color=col,
            bbox=dict(boxstyle="round,pad=0.28", facecolor="#FFFFFF",
                      edgecolor=col, alpha=0.88, linewidth=1.2),
        )

        # ── combined legend (both axes + band + mean) ─────────────────────────
        lines  = [lj, band, mean_line, lgx, lgy]
        labels = [l.get_label() for l in lines]
        ax_j.legend(lines, labels, loc="lower right", fontsize=9,
                    facecolor="#FFFFFF", edgecolor=_PALETTE["grid"],
                    labelcolor=_PALETTE["fg"])

        # ── correlation annotation (top-left) ─────────────────────────────────
        corr_x = float(np.corrcoef(qd, gx)[0, 1])
        corr_y = float(np.corrcoef(qd, gy)[0, 1])
        ax_j.text(
            0.01, 0.95,
            f"corr(joint, gx)={corr_x:+.3f}   corr(joint, gy)={corr_y:+.3f}",
            transform=ax_j.transAxes, fontsize=9, verticalalignment="top",
            color="#455A64",
            bbox=dict(boxstyle="round,pad=0.25", facecolor="#FFFFFF",
                      edgecolor=_PALETTE["grid"], alpha=0.9),
        )

        ax_j.set_title(f"{jname.replace('_', ' ')}  vs.  gravity  (gx, gy)",
                       fontsize=13, fontweight="bold", pad=8,
                       color=_PALETTE["fg"])
        if row == n - 1:
            ax_j.set_xlabel("Time  [s]", fontsize=12)

    _save_or_show(fig, save_path, "balance_joints_vs_gravity.png")


# ──────────────────────────────────────────────────────────────────────────────
# 4e.  CSV export
# ──────────────────────────────────────────────────────────────────────────────

def export_to_csv(
    timestamps: np.ndarray,
    gx: np.ndarray,
    gy: np.ndarray,
    pos_np: dict[str, np.ndarray],
    save_dir: str,
):
    """Save all tracked balance data to a single consolidated CSV file."""
    os.makedirs(save_dir, exist_ok=True)
    filename = os.path.join(save_dir, "balance_data.csv")

    joint_names = sorted(pos_np.keys())
    headers = ["time", "gravity_x", "gravity_y"]
    for j in joint_names:
        headers.append(f"{j}_pos_rad")
        headers.append(f"{j}_pos_deg")

    with open(filename, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(headers)
        for i in range(len(timestamps)):
            row = [timestamps[i], gx[i], gy[i]]
            for j in joint_names:
                row.append(pos_np[j][i])
                row.append(float(np.degrees(pos_np[j][i])))
            writer.writerow(row)

    print(f"[BAL] Data exported to → {filename}")


# ──────────────────────────────────────────────────────────────────────────────
# 5.  Main
# ──────────────────────────────────────────────────────────────────────────────
@hydra_task_config(args_cli.task, args_cli.agent)
def main(
    env_cfg:   ManagerBasedRLEnvCfg | DirectRLEnvCfg | DirectMARLEnvCfg,
    agent_cfg: RslRlBaseRunnerCfg,
):
    """Load a trained checkpoint, force cmd_vel = 0, run and plot balance data."""

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

    # ── discover joint indices ────────────────────────────────────────────────
    base_env  = env.unwrapped
    robot     = base_env.scene["robot"]
    dof_names = robot.data.joint_names   # list[str]

    joint_indices: dict[str, int] = {}
    for jname in _KEY_JOINTS:
        if jname in dof_names:
            joint_indices[jname] = dof_names.index(jname)
        else:
            matches = [i for i, n in enumerate(dof_names)
                       if jname.lower() in n.lower()]
            if matches:
                joint_indices[jname] = matches[0]
                print(f"[BAL] '{jname}' → matched DOF '{dof_names[matches[0]]}' (idx {matches[0]})")
            else:
                print(f"[BAL] WARNING – joint '{jname}' not found in DOF list. Skipping.")

    print(f"[BAL] Monitoring joints : {list(joint_indices.keys())}")
    print(f"[BAL] All DOF names     : {dof_names}\n", flush=True)

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
    gx_log:     list[float] = []   # projected_gravity_b[:, 0]  forward/backward
    gy_log:     list[float] = []   # projected_gravity_b[:, 1]  lateral
    pos_logs:   defaultdict[str, list] = defaultdict(list)

    print(f"[BAL] *** Command velocity FORCED to 0 m/s ***")
    print(f"[BAL] Recording env #{env_idx} for balance analysis.")
    if plot_steps:
        print(f"[BAL] Will run for {plot_steps} steps then plot.\n")

    # ── simulation loop ───────────────────────────────────────────────────────
    while simulation_app.is_running():
        start_time = time.time()

        with torch.inference_mode():
            actions = policy(obs)
            obs, _, dones, _ = env.step(actions)
            policy_nn.reset(dones)

        base_env = env.unwrapped
        robot    = base_env.scene["robot"]

        # ── Force command velocity to 0 m/s ──────────────────────────────────
        # The command manager stores velocity commands in the environment.
        # We zero them out every step so the policy always sees a 0 m/s target.
        try:
            cmd_mgr = base_env.command_manager
            for cmd_name in cmd_mgr._terms.keys():
                term = cmd_mgr._terms[cmd_name]
                if hasattr(term, "vel_command_b"):
                    term.vel_command_b[:] = 0.0
                elif hasattr(term, "_vel_command_b"):
                    term._vel_command_b[:] = 0.0
                # Fallback: zero any raw command tensor directly
                if hasattr(term, "command"):
                    term.command[:] = 0.0
        except Exception:
            pass  # Not every env uses a command manager

        # ── Projected gravity ─────────────────────────────────────────────────
        # projected_gravity_b: (num_envs, 3)  body-frame gravity vector
        # components [0]=x (forward lean), [1]=y (lateral lean), [2]=z (≈-9.81 upright)
        proj_grav = robot.data.projected_gravity_b  # (num_envs, 3)
        gx_log.append(proj_grav[env_idx, 0].item())
        gy_log.append(proj_grav[env_idx, 1].item())

        # ── Joint positions ───────────────────────────────────────────────────
        q_all = robot.data.joint_pos   # (num_envs, n_dof)
        for jname, idx in joint_indices.items():
            pos_logs[jname].append(q_all[env_idx, idx].item())

        timestamps.append(timestep * dt)

        # ── Console status every 200 steps ───────────────────────────────────
        if timestep % 200 == 0:
            gx_now = gx_log[-1]
            gy_now = gy_log[-1]
            rms_now = float(np.sqrt(np.mean(np.array(gx_log)**2 +
                                            np.array(gy_log)**2)))
            print(f"[BAL] step={timestep:5d} | "
                  f"gx={gx_now:+.4f}  gy={gy_now:+.4f}  "
                  f"rms_rad={rms_now:.4f}")

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
        print("[BAL] Not enough data to plot.")
        return

    t_arr = np.asarray(timestamps, dtype=np.float32)
    gx_arr = np.asarray(gx_log,    dtype=np.float32)
    gy_arr = np.asarray(gy_log,    dtype=np.float32)

    pos_np: dict[str, np.ndarray] = {
        j: np.asarray(v, dtype=np.float32) for j, v in pos_logs.items()
    }

    # ── print balance summary ─────────────────────────────────────────────────
    rms_radius = float(np.sqrt(np.mean(gx_arr ** 2 + gy_arr ** 2)))
    max_radius = float(np.sqrt(np.max(gx_arr ** 2 + gy_arr ** 2)))

    print("\n========== BALANCE SUMMARY (robot faces Y-axis) ==========")
    print(f"  Gravity X (lateral lean)    RMS={float(np.sqrt(np.mean(gx_arr**2))):.5f}  "
          f"range=[{gx_arr.min():.4f}, {gx_arr.max():.4f}]")
    print(f"  Gravity Y (fwd/back lean)   RMS={float(np.sqrt(np.mean(gy_arr**2))):.5f}  "
          f"range=[{gy_arr.min():.4f}, {gy_arr.max():.4f}]")
    print(f"  Balance score (RMS radius): {rms_radius:.5f}  (lower = better)")
    print(f"  Max excursion:              {max_radius:.5f}")
    print("=====================================")

    print("\n========== JOINT SUMMARY ==========")
    for jname, q in pos_np.items():
        qd = np.degrees(q)
        print(f"  {jname:20s}  range=[{qd.min():6.2f}°, {qd.max():6.2f}°]"
              f"  peak-to-peak={qd.ptp():.2f}°"
              f"  RMS={float(np.sqrt(np.mean(qd**2))):.2f}°")
    print("====================================\n", flush=True)

    # ── determine output directory ────────────────────────────────────────────
    save_dir = args_cli.save_plot or os.path.join(log_dir, "balance_plots")

    # ── Plot 1: Projected Gravity vs. Time ───────────────────────────────────
    plot_projected_gravity(t_arr, gx_arr, gy_arr, save_path=save_dir)

    # ── Plot 2: Joint Angle vs. Time ─────────────────────────────────────────
    plot_joint_trajectories(t_arr, pos_np, save_path=save_dir)

    # ── Plot 3: Gravity 2-D Scatter ──────────────────────────────────────────
    plot_gravity_scatter(gx_arr, gy_arr, save_path=save_dir)

    # ── Plot 4: KEY_JOINTS overlaid with projected gravity ────────────────────
    plot_joints_and_gravity(t_arr, gx_arr, gy_arr, pos_np, save_path=save_dir)

    # ── Export: CSV ───────────────────────────────────────────────────────────
    export_to_csv(t_arr, gx_arr, gy_arr, pos_np, save_dir=save_dir)

    print(f"[BAL] All plots and data saved to: {save_dir}")


# ──────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    main()
    simulation_app.close()
