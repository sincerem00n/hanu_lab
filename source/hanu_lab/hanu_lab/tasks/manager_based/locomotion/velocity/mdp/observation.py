# Copyright (c) 2024-2026, The RAI Hanumanoid Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Custom observation functions for the Hanu locomotion environment.

All functions follow the IsaacLab convention:
- First argument is always ``env: ManagerBasedRLEnv``.
- Additional keyword arguments must match parameters declared in
  :class:`isaaclab.managers.ObservationTermCfg`.
- Return value is a ``torch.Tensor`` of shape ``(num_envs, dim)``.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

import torch

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


# ---------------------------------------------------------------------------
# Gait phase
# ---------------------------------------------------------------------------

def gait_phase_sin_cos(env: ManagerBasedRLEnv, cycle_time: float) -> torch.Tensor:
    """Sine and cosine encoding of the current gait phase.

    Provides a smooth, periodic signal the policy can use to synchronise
    its leg swing timing without needing an explicit finite-state machine.

    Args:
        env: The RL environment instance.
        cycle_time: Duration of one full gait cycle in seconds.

    Returns:
        Tensor of shape ``(num_envs, 2)`` containing ``[sin(phase), cos(phase)]``.
    """
    # env.episode_length_buf  : int steps elapsed since reset  (num_envs,)
    # env.step_dt             : seconds per decimated physics step (scalar)
    current_time = env.episode_length_buf * env.step_dt  # (num_envs,)
    phase_rad = (current_time % cycle_time) / cycle_time * 2.0 * math.pi

    return torch.stack([torch.sin(phase_rad), torch.cos(phase_rad)], dim=-1)


# ---------------------------------------------------------------------------
# Reference trajectory — procedural sinusoidal walking gait
# ---------------------------------------------------------------------------

def target_joint_positions(
    env: ManagerBasedRLEnv,
    cycle_time: float = 0.64,
    time_offset: float = 0.0,
    hip_pitch_amp: float = 0.30,
    knee_pitch_amp: float = 0.20,
    ankle_pitch_amp: float = 0.12,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Procedural sinusoidal reference joint positions for bipedal walking.

    Generates a contactless reference motion **without** a ContactSensor by
    computing per-joint target angles from a sinusoidal gait clock.  The left
    and right legs are driven 180 ° out of phase (trot / alternating-leg
    pattern), which is the standard approach used in AMP, HumanoidGym, and
    similar frameworks.

    **Gait clock**::

        t       = env.episode_length_buf * env.step_dt  (seconds per env)
        phase_L = 2π · ((t + time_offset) % cycle_time) / cycle_time
        phase_R = phase_L + π          (half-cycle offset → legs alternate)

    **Reference angles per joint** (added on top of the default stance pose):

    =========  ======================================  ==============
    Joint      Signal                                  Amplitude
    =========  ======================================  ==============
    hip_pitch  sin(phase)                              hip_pitch_amp
    knee_pitch abs(sin(phase)) — always positive       knee_pitch_amp
    ankle_pitch –sin(phase)  — opposes hip             ankle_pitch_amp
    hip_yaw    0  (held at default)
    hip_roll   0  (held at default)
    ankle_roll 0  (held at default)
    =========  ======================================  ==============

    The result is also stored in ``env.target_joint_pos_ref`` so that reward
    functions (``joint_pos_tracking_exp``, ``foot_pos_tracking_exp``, etc.)
    can access the same reference without recomputing it.

    Args:
        env: The RL environment instance.
        cycle_time: Duration of one full gait cycle in seconds.
            Defaults to ``0.64`` s (≈ 1.6 Hz walking frequency).
        time_offset: Additional time offset in seconds applied before
            computing the phase.  Use a positive value to look ahead.
            Defaults to ``0.0``.
        hip_pitch_amp: Sinusoidal amplitude (rad) for the hip-pitch joints.
            Defaults to ``0.30`` rad (≈ 17°).
        knee_pitch_amp: Amplitude (rad) for the knee-pitch joints.
            ``abs(sin)`` is used so the knee never extends past default.
            Defaults to ``0.20`` rad (≈ 11°).
        ankle_pitch_amp: Amplitude (rad) for the ankle-pitch joints.
            Defaults to ``0.12`` rad (≈ 7°).
        asset_cfg: Scene entity configuration identifying the robot.
            Defaults to ``SceneEntityCfg("robot")``.

    Returns:
        Tensor of shape ``(num_envs, num_joints)`` with reference joint
        positions in radians, ordered to match ``asset_cfg.joint_ids``.
    """
    asset: Articulation = env.scene[asset_cfg.name]

    # ---- 1. Neutral stance pose (num_envs, num_all_joints) -----------------
    # asset.data.default_joint_pos is populated by IsaacLab from the USD/URDF
    # default state.  We clone so we can write to it safely.
    ref_q: torch.Tensor = asset.data.default_joint_pos.clone()  # (num_envs, J)

    # ---- 2. Gait phase per environment -------------------------------------
    current_time: torch.Tensor = env.episode_length_buf * env.step_dt  # (N,)
    t = (current_time + time_offset) % cycle_time  # wrap into [0, T)
    phase_L: torch.Tensor = 2.0 * math.pi * t / cycle_time  # (N,)
    phase_R: torch.Tensor = phase_L + math.pi               # 180° offset

    sin_L = torch.sin(phase_L)  # (N,)
    sin_R = torch.sin(phase_R)  # (N,)

    # ---- 3. Helper: find joint indices by substring keyword ----------------
    joint_names: list[str] = asset.data.joint_names  # list of str, length J

    def _idx(keyword: str) -> list[int]:
        """Return joint indices whose name contains *keyword* (case-insensitive)."""
        kw = keyword.lower()
        return [i for i, n in enumerate(joint_names) if kw in n.lower()]

    # Identify left / right leg joints by typical naming convention
    # Hanu A4 joints follow: "Joint_l_hip_pitch", "Joint_r_hip_pitch", ...
    # The keywords 'l_hip_pitch' and 'r_hip_pitch' uniquely select each side.
    l_hip_pitch_ids  = _idx("l_hip_pitch")
    r_hip_pitch_ids  = _idx("r_hip_pitch")
    l_knee_pitch_ids = _idx("l_knee_pitch")
    r_knee_pitch_ids = _idx("r_knee_pitch")
    l_ankle_ids      = _idx("l_ankle_pitch")
    r_ankle_ids      = _idx("r_ankle_pitch")

    # ---- 4. Apply sinusoidal deltas ----------------------------------------
    # hip_pitch: swing forward / backward
    for idx in l_hip_pitch_ids:
        ref_q[:, idx] += hip_pitch_amp * sin_L
    for idx in r_hip_pitch_ids:
        ref_q[:, idx] += hip_pitch_amp * sin_R

    # knee_pitch: always flex (abs), so knee never hyper-extends
    for idx in l_knee_pitch_ids:
        ref_q[:, idx] += knee_pitch_amp * torch.abs(sin_L)
    for idx in r_knee_pitch_ids:
        ref_q[:, idx] += knee_pitch_amp * torch.abs(sin_R)

    # ankle_pitch: compensate to keep foot parallel to ground
    for idx in l_ankle_ids:
        ref_q[:, idx] -= ankle_pitch_amp * sin_L
    for idx in r_ankle_ids:
        ref_q[:, idx] -= ankle_pitch_amp * sin_R

    # ---- 5. Expose reference to reward functions via env attribute ----------
    # Rewards such as joint_pos_tracking_exp read env.target_joint_pos_ref.
    env.target_joint_pos_ref = ref_q  # (num_envs, num_all_joints)

    # ---- 6. Return only the joints requested by asset_cfg ------------------
    return ref_q[:, asset_cfg.joint_ids]


# ---------------------------------------------------------------------------
# Zero-padding functions for future observation terms
# ---------------------------------------------------------------------------

GAIT_PHASE_DIM = 2  # Example: [sin(phase), cos(phase)]
TARGET_Q_DIM = 23   # Example: 12 DoF humanoid joints
ACTION_HISTORY_DIM = 12 * 8 # Example: 12 actions * history length of 8

def zero_gait_phase(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Returns a zero tensor matching the shape of the future gait_phase observation."""
    return torch.zeros((env.num_envs, GAIT_PHASE_DIM), device=env.device, dtype=torch.float32)

def zero_target_q(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Returns a zero tensor matching the shape of the future target_q observation."""
    return torch.zeros((env.num_envs, TARGET_Q_DIM), device=env.device, dtype=torch.float32)

def padded_action_history(env: ManagerBasedRLEnv) -> torch.Tensor:
    """
    Returns an action history tensor where the most recent action is real, 
    and the remaining historical steps are zero-padded to reserve space.

    Note: use this instead of normal 'actions'
    """
    # 1. Get the most recent action (t-1)
    last_action = env.action_manager.action  # Shape: (num_envs, action_dim)
    
    # 2. Calculate the padding size
    action_dim = last_action.shape[1]
    history_length = 8
    padding_dim = action_dim * (history_length - 1) # Space for the 7 older actions
    
    # 3. Create zero-padding for the older history
    zero_padding = torch.zeros((env.num_envs, padding_dim), device=env.device, dtype=torch.float32)
    
    # 4. Concatenate: [last_action, 0, 0, 0, 0, 0, 0, 0]
    return torch.cat([last_action, zero_padding], dim=-1)