# Copyright (c) 2025, RAI Hanumanoid Project Developers.
# All rights reserved.

# SPDX-License-Identifier: BSD-3-Clause

from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass

from .rough_env_cfg import HanuA2RoughEnvCfgV0, HanuA2RoughEnvCfgV1

@configclass
class HanuA2FlatEnvCfgV0(HanuA2RoughEnvCfgV0):
    """Configuration for the flat environment in the RAI Hanumanoid project. -- Version 0."""
    def __post_init__(self):
        super().__post_init__()

        self.scene.terrain.terrain_type = "plane"
        self.scene.terrain.terrain_generator = None

        self.scene.height_scanner = None
        self.observations.policy.height_scan = None

        self.curriculum.terrain_levels = None

@configclass
class HanuA2FlatEnvCfgV1(HanuA2RoughEnvCfgV1):
    """Configuration for the flat environment in the RAI Hanumanoid project. -- Version 1."""
    def __post_init__(self):
        super().__post_init__()

        self.scene.terrain.terrain_type = "plane"
        self.scene.terrain.terrain_generator = None

        self.scene.height_scanner = None
        self.observations.policy.height_scan = None

        self.curriculum.terrain_levels = None
