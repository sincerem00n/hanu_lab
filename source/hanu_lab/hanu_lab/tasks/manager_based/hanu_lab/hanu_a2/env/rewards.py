# Copyright (c) 2025, RAI Hanumanoid Project Developers.
# All rights reserved.

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.assets import RigidObject
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import ContactSensor

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

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


