# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Common functions that can be used to define rewards for the learning environment.

The functions can be passed to the :class:`isaaclab.managers.RewardTermCfg` object to
specify the reward function and its parameters.
"""

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.envs import mdp
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import ContactSensor
from isaaclab.assets import RigidObject, Articulation
from isaaclab.utils.math import quat_apply_inverse, yaw_quat

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def feet_air_time(
    env: ManagerBasedRLEnv, command_name: str, sensor_cfg: SceneEntityCfg, threshold: float
) -> torch.Tensor:
    """Reward long steps taken by the feet using L2-kernel.

    This function rewards the agent for taking steps that are longer than a threshold. This helps ensure
    that the robot lifts its feet off the ground and takes steps. The reward is computed as the sum of
    the time for which the feet are in the air.

    If the commands are small (i.e. the agent is not supposed to take a step), then the reward is zero.
    """
    # extract the used quantities (to enable type-hinting)
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    # compute the reward
    first_contact = contact_sensor.compute_first_contact(env.step_dt)[:, sensor_cfg.body_ids]
    last_air_time = contact_sensor.data.last_air_time[:, sensor_cfg.body_ids]
    reward = torch.sum((last_air_time - threshold) * first_contact, dim=1)
    # no reward for zero command
    reward *= torch.norm(env.command_manager.get_command(command_name)[:, :2], dim=1) > 0.1
    return reward


def feet_air_time_positive_biped(env, command_name: str, threshold: float, sensor_cfg: SceneEntityCfg) -> torch.Tensor:
    """Reward long steps taken by the feet for bipeds.

    This function rewards the agent for taking steps up to a specified threshold and also keep one foot at
    a time in the air.

    If the commands are small (i.e. the agent is not supposed to take a step), then the reward is zero.
    """
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    # compute the reward
    air_time = contact_sensor.data.current_air_time[:, sensor_cfg.body_ids]
    contact_time = contact_sensor.data.current_contact_time[:, sensor_cfg.body_ids]
    in_contact = contact_time > 0.0
    in_mode_time = torch.where(in_contact, contact_time, air_time)
    single_stance = torch.sum(in_contact.int(), dim=1) == 1
    reward = torch.min(torch.where(single_stance.unsqueeze(-1), in_mode_time, 0.0), dim=1)[0]
    reward = torch.clamp(reward, max=threshold)
    # no reward for zero command
    reward *= torch.norm(env.command_manager.get_command(command_name)[:, :2], dim=1) > 0.1
    return reward

# New #

def feet_air_time_negative_biped(env, command_name: str, threshold: float, sensor_cfg: SceneEntityCfg) -> torch.Tensor:
    """Penalize keeping one foot in the air for too long (biped).

    The penalty is applied when the robot is in single-stance (exactly one foot in contact),
    and the swing foot's current air-time exceeds `threshold`.

    If the commands are small (i.e. the agent is not supposed to step), then the penalty is zero.
    """
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    # compute the reward
    air_time = contact_sensor.data.current_air_time[:, sensor_cfg.body_ids]
    contact_time = contact_sensor.data.current_contact_time[:, sensor_cfg.body_ids]
    in_contact = contact_time > 0.0 
    swing_air_time = torch.max(torch.where(~in_contact, air_time, 0.0), dim=1)[0]
    single_stance = torch.sum(in_contact.int(), dim=1) == 1  
    reward = torch.clamp(swing_air_time - threshold, min=0.0) 
    reward *= single_stance
    # no reward for zero command
    reward *= torch.norm(env.command_manager.get_command(command_name)[:, :2], dim=1) > 0.1
    return reward


def feet_slide(env, sensor_cfg: SceneEntityCfg, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Penalize feet sliding.

    This function penalizes the agent for sliding its feet on the ground. The reward is computed as the
    norm of the linear velocity of the feet multiplied by a binary contact sensor. This ensures that the
    agent is penalized only when the feet are in contact with the ground.
    """
    # Penalize feet sliding
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    contacts = contact_sensor.data.net_forces_w_history[:, :, sensor_cfg.body_ids, :].norm(dim=-1).max(dim=1)[0] > 1.0
    asset = env.scene[asset_cfg.name]

    body_vel = asset.data.body_lin_vel_w[:, asset_cfg.body_ids, :2]
    reward = torch.sum(body_vel.norm(dim=-1) * contacts, dim=1)
    return reward

# def feet_height(env)

def track_lin_vel_xy_yaw_frame_exp(
    env, std: float, command_name: str, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    """Reward tracking of linear velocity commands (xy axes) in the gravity aligned robot frame using exponential kernel."""
    # extract the used quantities (to enable type-hinting)
    asset = env.scene[asset_cfg.name]
    vel_yaw = quat_apply_inverse(yaw_quat(asset.data.root_quat_w), asset.data.root_lin_vel_w[:, :3])
    lin_vel_error = torch.sum(
        torch.square(env.command_manager.get_command(command_name)[:, :2] - vel_yaw[:, :2]), dim=1
    )
    return torch.exp(-lin_vel_error / std**2)


def track_ang_vel_z_world_exp(
    env, command_name: str, std: float, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    """Reward tracking of angular velocity commands (yaw) in world frame using exponential kernel."""
    # extract the used quantities (to enable type-hinting)
    asset = env.scene[asset_cfg.name]
    ang_vel_error = torch.square(env.command_manager.get_command(command_name)[:, 2] - asset.data.root_ang_vel_w[:, 2])
    return torch.exp(-ang_vel_error / std**2)


def stand_still_joint_deviation_l1(
    env, command_name: str, command_threshold: float = 0.06, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    """Penalize offsets from the default joint positions when the command is very small."""
    command = env.command_manager.get_command(command_name)
    # Penalize motion when command is nearly zero.
    return mdp.joint_deviation_l1(env, asset_cfg) * (torch.norm(command[:, :2], dim=1) < command_threshold)


def upright_orientation_l2(
    env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    """
    Reward the robot for staying upright by penalizing deviation from upright orientation.

    This uses the L2 squared distance of the projected gravity vector from upright (z-axis).
    """
    # Extract the robot asset
    asset: RigidObject = env.scene[asset_cfg.name]
    # The projected gravity in the base frame
    g_proj = asset.data.projected_gravity_b  # shape: [N, 3]
    # Compute deviation from upright (z-axis direction [0, 0, -1])
    target_gravity = torch.tensor([0.0, 0.0, -1.0], device=env.device)
    deviation = g_proj - target_gravity
    # L2 squared deviation as penalty
    penalty = torch.sum(torch.square(deviation), dim=1)
    # Return negative penalty as reward (higher reward when closer to upright)
    return -penalty

def action_mirror(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg, mirror_joints: list[list[str]]) -> torch.Tensor:
    """
    Penalize asymmetry in effort between specific joint pairs (e.g. Left Hip vs Right Hip).

    This function computes the squared difference between the absolute action magnitudes 
    of matched pairs. This encourages the robot to use equal effort on both sides of the 
    body (preventing limping), regardless of the direction of motion.
    """
    asset: Articulation = env.scene[asset_cfg.name]
    if not hasattr(env, "action_mirror_joints_cache") or env.action_mirror_joints_cache is None:
        # Cache joint positions for all pairs
        env.action_mirror_joints_cache = [
            [asset.find_joints(joint_name) for joint_name in joint_pair] for joint_pair in mirror_joints
        ]
    reward = torch.zeros(env.num_envs, device=env.device)
    # Iterate over all joint pairs
    for joint_pair in env.action_mirror_joints_cache:
        # Calculate the difference for each pair and add to the total reward
        diff = torch.sum(
            torch.square(
                torch.abs(env.action_manager.action[:, joint_pair[0][0]])
                - torch.abs(env.action_manager.action[:, joint_pair[1][0]])
            ),
            dim=-1,
        )
        reward += diff
    reward *= 1 / len(mirror_joints) if len(mirror_joints) > 0 else 0
    reward *= torch.clamp(-env.scene["robot"].data.projected_gravity_b[:, 2], 0, 0.7) / 0.7
    return reward


def action_sync(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg, joint_groups: list[list[str]]) -> torch.Tensor:
    """
    Penalize variance within groups of joints.

    This function calculates the variance of the absolute action magnitudes within each 
    specified group. This encourages all joints in a group (e.g., all 4 hip motors, 
    or duplicate motors on a single joint) to exert consistent levels of effort 
    relative to each other.
    """
    asset: Articulation = env.scene[asset_cfg.name]

    # Cache joint indices if not already done
    if not hasattr(env, "action_sync_joint_cache") or env.action_sync_joint_cache is None:
        env.action_sync_joint_cache = [
            [asset.find_joints(joint_name) for joint_name in joint_group] for joint_group in joint_groups
        ]

    reward = torch.zeros(env.num_envs, device=env.device)
    # Iterate over each joint group
    for joint_group in env.action_sync_joint_cache:
        if len(joint_group) < 2:
            continue  # need at least 2 joints to compare

        # Get absolute actions for all joints in this group
        actions = torch.stack(
            [torch.abs(env.action_manager.action[:, joint[0]]) for joint in joint_group], dim=1
        )  # shape: (num_envs, num_joints_in_group)

        # Calculate mean action for each environment
        mean_actions = torch.mean(actions, dim=1, keepdim=True)

        # Calculate variance from mean for each joint
        variance = torch.mean(torch.square(actions - mean_actions), dim=1)

        # Add to reward (we want to minimize this variance)
        reward += variance.squeeze()
    reward *= 1 / len(joint_groups) if len(joint_groups) > 0 else 0
    reward *= torch.clamp(-env.scene["robot"].data.projected_gravity_b[:, 2], 0, 0.7) / 0.7
    return reward

# New #

def feet_step_sequence_biped(env, 
                             command_name: str, 
                             sensor_cfg: SceneEntityCfg,
                             threshold_steps: float = 1.5
                             ) -> torch.Tensor:
    """Reward alternating left-right stepping sequence for bipeds.

    Gives reward only when a new swing-foot event happens AND
    the swing foot is different from the previous one.
    No penalty is applied for wrong sequence.
    """
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]

    # first_contact = contact_sensor.compute_first_contact(env.step_dt)[:, sensor_cfg.body_ids]
    air_time = contact_sensor.data.current_air_time[:, sensor_cfg.body_ids]
    toe_off = (air_time > 0.0) & (air_time < env.step_dt * threshold_steps) 
    valid_event = torch.sum(toe_off.int(), dim=1) == 1
    swing_foot = torch.argmax(toe_off.int(), dim=1)

    if not hasattr(env, "last_swing_foot"):
        env.last_swing_foot = -torch.ones(env.num_envs, device=env.device)
    correct_sequence = swing_foot != env.last_swing_foot
    moving = torch.norm(env.command_manager.get_command(command_name)[:, :2], dim=1) > 0.1
    reward = (valid_event & correct_sequence & moving).float()
    env.last_swing_foot = torch.where(
        valid_event, swing_foot, env.last_swing_foot
    )

    return reward

# New #

def feet_lateral_separation_reward(
    env,
    sensor_cfg: SceneEntityCfg,
    threshold: float = 0.13,     # = foot width (0.11) + clearance (~0.02)
    margin: float = 0.03,        # soft zone ~ 2–3 cm
    command_name: str | None = "base_velocity",
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Reward that penalizes feet getting too close laterally.
    Returns 0 when OK, negative when feet are too close.
    """
    asset = env.scene[asset_cfg.name]

    # world positions of left/right foot link centers
    foot_pos_w = asset.data.body_pos_w[:, sensor_cfg.body_ids, :]  # (N, 2, 3)
    left_w = foot_pos_w[:, 0, :3]
    right_w = foot_pos_w[:, 1, :3]

    # transform into yaw-aligned base frame
    base_yaw = yaw_quat(asset.data.root_quat_w)
    delta_yaw = quat_apply_inverse(base_yaw, left_w - right_w)

    # lateral (left-right) separation
    sep = torch.abs(delta_yaw[:, 1])

    # soft penalty when too close
    penalty = torch.clamp((threshold - sep) / margin, min=0.0)
    penalty = penalty ** 2

    # reward is negative penalty
    reward = -penalty

    # apply only when robot is commanded to move
    if command_name is not None:
        moving = torch.norm(
            env.command_manager.get_command(command_name)[:, :2], dim=1
        ) > 0.1
        reward = reward * moving.float()

    return reward

# New #

def arms_lateral_open_pose(
    env,
    arm_sensor_cfg: SceneEntityCfg,
    min_lateral_dist: float = 0.22,
    max_backward_dist: float = 0.08,
    lateral_margin: float = 0.05,
    backward_margin: float = 0.05,
    command_name: str | None = "base_velocity",
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Encourage arms to stay laterally away from the torso and not tucked backward."""

    asset = env.scene[asset_cfg.name]

    # (N, 2, 3)
    arm_pos_w = asset.data.body_pos_w[:, arm_sensor_cfg.body_ids, :3]
    root_pos_w = asset.data.root_pos_w[:, :3]

    # relative arm positions in world frame
    arm_rel_w = arm_pos_w - root_pos_w.unsqueeze(1)   # (N, 2, 3)

    # yaw quaternion of base
    base_yaw = yaw_quat(asset.data.root_quat_w)       # (N, 4)

    # flatten 2 arms -> (N*2, 3)
    arm_rel_w_flat = arm_rel_w.reshape(-1, 3)         # (N*2, 3)

    # repeat quaternion for left/right arm -> (N*2, 4)
    base_yaw_rep = base_yaw.unsqueeze(1).repeat(1, arm_rel_w.shape[1], 1).reshape(-1, 4)

    # transform into yaw-aligned base frame
    arm_rel_flat = quat_apply_inverse(base_yaw_rep, arm_rel_w_flat)   # (N*2, 3)
    arm_rel = arm_rel_flat.view(arm_rel_w.shape[0], arm_rel_w.shape[1], 3)  # (N, 2, 3)

    # x = forward/backward, y = lateral
    lateral = torch.abs(arm_rel[:, :, 1])
    backward = torch.clamp(-arm_rel[:, :, 0], min=0.0)

    lateral_penalty = torch.clamp((min_lateral_dist - lateral) / lateral_margin, min=0.0) ** 2
    backward_penalty = torch.clamp((backward - max_backward_dist) / backward_margin, min=0.0) ** 2

    reward = -(torch.mean(lateral_penalty, dim=1) + 0.5 * torch.mean(backward_penalty, dim=1))

    if command_name is not None:
        moving = torch.norm(env.command_manager.get_command(command_name)[:, :2], dim=1) > 0.1
        reward = reward * moving.float()

    return reward

# ======================== Reference Motion Tracking ======================================


def joint_pos_tracking_exp(
    env: ManagerBasedRLEnv,
    std: float = 0.25,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    joint_weights: list[float] | None = None,
) -> torch.Tensor:
    """Reward joint position tracking of a reference motion using an exponential kernel.

    The reward is computed as ``exp(-error / std^2)`` where ``error`` is the
    (optionally weighted) mean-squared error between the current joint positions
    and the reference joint positions.

    The reference positions are obtained from ``env.ref_motion.get_joint_positions(env)``
    if a reference motion manager is attached to the environment.  If no such
    manager exists the robot's default joint positions are used as the reference,
    which effectively encourages the robot to stay in its rest pose.

    Args:
        env: The RL environment.
        std: Standard deviation that controls the width of the exponential kernel.
             Smaller values make the reward sharper (more sensitive to errors).
        asset_cfg: Scene entity configuration for the robot articulation.
        joint_weights: Optional per-joint weights applied to the squared error
            before averaging.  Length must match the number of joints selected by
            ``asset_cfg``.  ``None`` means all joints are weighted equally.

    Returns:
        Tensor of shape ``(num_envs,)`` with per-environment rewards in [0, 1].
    """
    asset: Articulation = env.scene[asset_cfg.name]

    # Current joint positions (relative to default / zero pos depends on asset setup)
    # joint_pos_rel already subtracts default_joint_pos, so use raw joint_pos here.
    current_q = asset.data.joint_pos  # (N, num_joints)

    # ── Reference joint positions ──────────────────────────────────────────────
    if hasattr(env, "ref_motion") and env.ref_motion is not None:
        # External reference motion manager must implement get_joint_positions(env)
        # and return a tensor of shape (N, num_joints).
        ref_q = env.ref_motion.get_joint_positions(env)
    else:
        # Fallback: use the robot's default joint positions as a static reference.
        ref_q = asset.data.default_joint_pos  # (N, num_joints) or (1, num_joints)
        if ref_q.shape[0] == 1:
            ref_q = ref_q.expand(env.num_envs, -1)

    # ── Subset of joints (if body_ids / joint_ids are configured) ─────────────
    if asset_cfg.joint_ids is not None and asset_cfg.joint_ids != slice(None):
        current_q = current_q[:, asset_cfg.joint_ids]
        ref_q = ref_q[:, asset_cfg.joint_ids]

    # ── Per-joint squared error ────────────────────────────────────────────────
    sq_error = torch.square(current_q - ref_q)  # (N, num_joints)

    if joint_weights is not None:
        weights = torch.tensor(joint_weights, dtype=sq_error.dtype, device=env.device)
        sq_error = sq_error * weights

    # Mean squared error across joints
    mse = torch.mean(sq_error, dim=1)  # (N,)

    return torch.exp(-mse / std**2)


def joint_pos_tracking_l2(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    joint_weights: list[float] | None = None,
) -> torch.Tensor:
    """Penalize joint position deviation from a reference motion using L2 norm.

    Unlike the exponential variant, this returns a *penalty* (negative values)
    equal to the (optionally weighted) root-mean-square error between the current
    and reference joint positions.  Pair it with a negative reward weight in the
    config.

    The reference positions are obtained from ``env.ref_motion.get_joint_positions(env)``
    if available, otherwise the robot's default positions are used.

    Args:
        env: The RL environment.
        asset_cfg: Scene entity configuration for the robot articulation.
        joint_weights: Optional per-joint weights for the squared error.

    Returns:
        Tensor of shape ``(num_envs,)`` — RMSE penalty (non-negative).
        Use a **negative** reward weight when registering this term.
    """
    asset: Articulation = env.scene[asset_cfg.name]

    current_q = asset.data.joint_pos  # (N, num_joints)

    if hasattr(env, "ref_motion") and env.ref_motion is not None:
        ref_q = env.ref_motion.get_joint_positions(env)
    else:
        ref_q = asset.data.default_joint_pos
        if ref_q.shape[0] == 1:
            ref_q = ref_q.expand(env.num_envs, -1)

    if asset_cfg.joint_ids is not None and asset_cfg.joint_ids != slice(None):
        current_q = current_q[:, asset_cfg.joint_ids]
        ref_q = ref_q[:, asset_cfg.joint_ids]

    sq_error = torch.square(current_q - ref_q)  # (N, num_joints)

    if joint_weights is not None:
        weights = torch.tensor(joint_weights, dtype=sq_error.dtype, device=env.device)
        sq_error = sq_error * weights

    # RMSE across joints — always non-negative, use negative reward weight
    return torch.sqrt(torch.mean(sq_error, dim=1))


# ======================== Reference Velocity Tracking ======================================


def joint_vel_tracking_exp(
    env: ManagerBasedRLEnv,
    std: float = 1.0,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    joint_weights: list[float] | None = None,
) -> torch.Tensor:
    """Reward joint velocity tracking of a reference motion using an exponential kernel.

    Matching joint velocities ensures the robot mimics the momentum and flow of the
    reference motion — not just snapping to target positions but actually moving
    through them at the right speed.

    The reward is computed as ``exp(-MSE / std^2)`` where ``MSE`` is the
    (optionally weighted) mean squared error between the current joint velocities
    and the reference joint velocities.

    The reference velocities are obtained from ``env.ref_motion.get_joint_velocities(env)``
    if a reference motion manager is attached. If no such manager exists, zero velocities
    are used as the reference (penalising unnecessary joint motion).

    Args:
        env: The RL environment.
        std: Standard deviation controlling the kernel width. Velocity errors are
             naturally larger in magnitude than position errors — values in the
             range 0.5 – 2.0 rad/s are typical starting points.
        asset_cfg: Scene entity configuration for the robot articulation.
        joint_weights: Optional per-joint weights applied to the squared error
            before averaging. ``None`` means uniform weighting.

    Returns:
        Tensor of shape ``(num_envs,)`` with per-environment rewards in [0, 1].
    """
    asset: Articulation = env.scene[asset_cfg.name]

    current_dq = asset.data.joint_vel  # (N, num_joints)

    # ── Reference joint velocities ─────────────────────────────────────────────
    if hasattr(env, "ref_motion") and env.ref_motion is not None:
        # Reference motion manager must implement get_joint_velocities(env)
        # and return a tensor of shape (N, num_joints).
        ref_dq = env.ref_motion.get_joint_velocities(env)
    else:
        # Fallback: reference velocity is zero (rest / no-motion reference).
        ref_dq = torch.zeros_like(current_dq)

    # ── Subset of joints ───────────────────────────────────────────────────────
    if asset_cfg.joint_ids is not None and asset_cfg.joint_ids != slice(None):
        current_dq = current_dq[:, asset_cfg.joint_ids]
        ref_dq = ref_dq[:, asset_cfg.joint_ids]

    # ── Per-joint squared error ────────────────────────────────────────────────
    sq_error = torch.square(current_dq - ref_dq)  # (N, num_joints)

    if joint_weights is not None:
        weights = torch.tensor(joint_weights, dtype=sq_error.dtype, device=env.device)
        sq_error = sq_error * weights

    mse = torch.mean(sq_error, dim=1)  # (N,)

    return torch.exp(-mse / std**2)


def joint_vel_tracking_l2(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    joint_weights: list[float] | None = None,
) -> torch.Tensor:
    """Penalize joint velocity deviation from a reference motion using L2 norm.

    Returns a *penalty* (non-negative RMSE) between the current joint velocities
    and the reference joint velocities.  Pair with a **negative** reward weight in
    the config to penalise velocity mismatch.

    The reference velocities are obtained from ``env.ref_motion.get_joint_velocities(env)``
    if available, otherwise zero velocities are used.

    Args:
        env: The RL environment.
        asset_cfg: Scene entity configuration for the robot articulation.
        joint_weights: Optional per-joint weights for the squared error.

    Returns:
        Tensor of shape ``(num_envs,)`` — RMSE penalty (non-negative).
        Use a **negative** reward weight when registering this term.
    """
    asset: Articulation = env.scene[asset_cfg.name]

    current_dq = asset.data.joint_vel  # (N, num_joints)

    if hasattr(env, "ref_motion") and env.ref_motion is not None:
        ref_dq = env.ref_motion.get_joint_velocities(env)
    else:
        ref_dq = torch.zeros_like(current_dq)

    if asset_cfg.joint_ids is not None and asset_cfg.joint_ids != slice(None):
        current_dq = current_dq[:, asset_cfg.joint_ids]
        ref_dq = ref_dq[:, asset_cfg.joint_ids]

    sq_error = torch.square(current_dq - ref_dq)  # (N, num_joints)

    if joint_weights is not None:
        weights = torch.tensor(joint_weights, dtype=sq_error.dtype, device=env.device)\
        
        sq_error = sq_error * weights

    # RMSE across joints — always non-negative, use negative reward weight
    return torch.sqrt(torch.mean(sq_error, dim=1))


# ======================== End-Effector (Foot) Cartesian Tracking ======================================


def foot_pos_tracking_exp(
    env: ManagerBasedRLEnv,
    foot_cfg: SceneEntityCfg,
    std: float = 0.05,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Reward foot Cartesian position tracking against a reference motion using an exponential kernel.

    Even when joint angles are close, kinematic chain errors can accumulate so that
    feet end up in the wrong position relative to the base, causing the robot to trip.
    This reward directly penalises end-effector position error in the yaw-aligned base
    frame, catching those cumulative FK errors that joint-space tracking alone misses.

    The reward is ``exp(-MSE / std^2)`` where ``MSE`` is the mean squared Cartesian
    distance (averaged over all tracked feet) between the current and reference foot
    positions expressed in the **yaw-aligned base frame**.

    Reference foot positions are resolved in priority order:
      1. ``env.ref_motion.get_foot_positions_b(env)`` — positions already in base frame,
         shape ``(N, num_feet, 3)``.
      2. ``env.ref_motion.get_foot_positions_w(env)`` — world-frame positions,
         shape ``(N, num_feet, 3)``; auto-transformed to the yaw-aligned base frame.
      3. Fallback: the foot positions snapshotted at the start of each episode
         (from ``env.extras["foot_pos_b_init"]``).  These are computed and cached
         on the first call, giving a static "stand-still" reference.

    Args:
        env: The RL environment.
        foot_cfg: ``SceneEntityCfg`` for the **robot** asset with ``body_names`` set
            to the foot links (e.g. ``SceneEntityCfg("robot", body_names=".*foot.*")``).
        std: Kernel width in metres. Typical values: 0.02–0.10 m.
        asset_cfg: Scene entity config for the robot (used for root pose).

    Returns:
        Tensor of shape ``(num_envs,)`` with per-environment rewards in [0, 1].
    """
    asset: Articulation = env.scene[asset_cfg.name]

    # ── Current foot positions in yaw-aligned base frame ──────────────────────
    # body_pos_w : (N, num_bodies, 3)
    foot_pos_w = asset.data.body_pos_w[:, foot_cfg.body_ids, :3]  # (N, F, 3)

    root_pos_w = asset.data.root_pos_w[:, :3]               # (N, 3)
    base_yaw_quat = yaw_quat(asset.data.root_quat_w)        # (N, 4)

    # Relative foot positions in world frame, then rotate to base frame
    N, F, _ = foot_pos_w.shape
    rel_w = foot_pos_w - root_pos_w.unsqueeze(1)             # (N, F, 3)
    rel_w_flat = rel_w.reshape(N * F, 3)
    yaw_rep = base_yaw_quat.unsqueeze(1).expand(N, F, 4).reshape(N * F, 4)
    foot_pos_b = quat_apply_inverse(yaw_rep, rel_w_flat).reshape(N, F, 3)  # (N, F, 3)

    # ── Reference foot positions ───────────────────────────────────────────────
    if hasattr(env, "ref_motion") and env.ref_motion is not None:
        if hasattr(env.ref_motion, "get_foot_positions_b"):
            # Already in base frame
            ref_foot_b = env.ref_motion.get_foot_positions_b(env)            # (N, F, 3)
        elif hasattr(env.ref_motion, "get_foot_positions_w"):
            # World-frame → transform to yaw-aligned base frame
            ref_w = env.ref_motion.get_foot_positions_w(env)                 # (N, F, 3)
            ref_rel_w = ref_w - root_pos_w.unsqueeze(1)
            ref_rel_flat = ref_rel_w.reshape(N * F, 3)
            ref_foot_b = quat_apply_inverse(yaw_rep, ref_rel_flat).reshape(N, F, 3)
        else:
            ref_foot_b = foot_pos_b.detach()
    else:
        # Fallback: cache the initial foot positions at episode start
        if not hasattr(env, "_foot_pos_b_init") or env._foot_pos_b_init is None:
            env._foot_pos_b_init = foot_pos_b.detach().clone()
        # When the environment resets, reset the cache for those envs
        reset_mask = env.episode_length_buf == 1          # first step after reset
        if reset_mask.any():
            env._foot_pos_b_init[reset_mask] = foot_pos_b[reset_mask].detach()
        ref_foot_b = env._foot_pos_b_init

    # ── Squared Cartesian error per foot ──────────────────────────────────────
    # (N, F, 3) → squared distance per foot → (N, F)
    sq_dist = torch.sum(torch.square(foot_pos_b - ref_foot_b), dim=-1)

    # Mean over all feet → (N,)
    mse = torch.mean(sq_dist, dim=-1)

    return torch.exp(-mse / std**2)


def foot_pos_tracking_l2(
    env: ManagerBasedRLEnv,
    foot_cfg: SceneEntityCfg,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Penalise foot Cartesian position deviation from a reference motion using L2 norm.

    Returns the mean Euclidean foot-position error (in metres) across all tracked feet,
    expressed in the yaw-aligned base frame.  Pair with a **negative** reward weight.

    Uses the same reference resolution logic as :func:`foot_pos_tracking_exp`.

    Args:
        env: The RL environment.
        foot_cfg: ``SceneEntityCfg`` with ``body_names`` pointing to the foot links.
        asset_cfg: Scene entity config for the robot.

    Returns:
        Tensor of shape ``(num_envs,)`` — mean foot position error in metres.
        Use a **negative** reward weight when registering this term.
    """
    asset: Articulation = env.scene[asset_cfg.name]

    foot_pos_w = asset.data.body_pos_w[:, foot_cfg.body_ids, :3]  # (N, F, 3)
    root_pos_w = asset.data.root_pos_w[:, :3]
    base_yaw_quat = yaw_quat(asset.data.root_quat_w)

    N, F, _ = foot_pos_w.shape
    rel_w = foot_pos_w - root_pos_w.unsqueeze(1)
    rel_w_flat = rel_w.reshape(N * F, 3)
    yaw_rep = base_yaw_quat.unsqueeze(1).expand(N, F, 4).reshape(N * F, 4)
    foot_pos_b = quat_apply_inverse(yaw_rep, rel_w_flat).reshape(N, F, 3)

    # ── Reference foot positions ───────────────────────────────────────────────
    if hasattr(env, "ref_motion") and env.ref_motion is not None:
        if hasattr(env.ref_motion, "get_foot_positions_b"):
            ref_foot_b = env.ref_motion.get_foot_positions_b(env)
        elif hasattr(env.ref_motion, "get_foot_positions_w"):
            ref_w = env.ref_motion.get_foot_positions_w(env)
            ref_rel_w = ref_w - root_pos_w.unsqueeze(1)
            ref_rel_flat = ref_rel_w.reshape(N * F, 3)
            ref_foot_b = quat_apply_inverse(yaw_rep, ref_rel_flat).reshape(N, F, 3)
        else:
            ref_foot_b = foot_pos_b.detach()
    else:
        if not hasattr(env, "_foot_pos_b_init") or env._foot_pos_b_init is None:
            env._foot_pos_b_init = foot_pos_b.detach().clone()
        reset_mask = env.episode_length_buf == 1
        if reset_mask.any():
            env._foot_pos_b_init[reset_mask] = foot_pos_b[reset_mask].detach()
        ref_foot_b = env._foot_pos_b_init

    # ── Mean Euclidean error across feet ──────────────────────────────────────
    # (N, F, 3) → L2 per foot → (N, F) → mean → (N,)
    dist = torch.norm(foot_pos_b - ref_foot_b, dim=-1)   # (N, F)
    return torch.mean(dist, dim=-1)                        # (N,)