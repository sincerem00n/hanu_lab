# Copyright (c) 2025, RAI Hanumanoid Project Developers.
# All rights reserved.

# SPDX-License-Identifier: BSD-3-Clause

from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass

from .rough_env_cfg import HanuA4RoughEnvCfgV0, HanuA4RoughEnvCfgV1

@configclass
class HanuA4FlatEnvCfgV0(HanuA4RoughEnvCfgV0):
    """Configuration for the flat environment in the RAI Hanumanoid project. -- Version 0."""
    def __post_init__(self):
        super().__post_init__()

        self.scene.terrain.terrain_type = "plane"
        self.scene.terrain.terrain_generator = None

        self.scene.height_scanner = None
        self.observations.policy.height_scan = None
        self.observations.critic.height_scan = None

        self.curriculum.terrain_levels = None


from isaaclab.utils import configclass

@configclass
class HanuA4FlatEnvCfgV1(HanuA4RoughEnvCfgV1):
    """Configuration for the flat environment in the RAI Hanumanoid project. -- Version 1."""
    def __post_init__(self):
        super().__post_init__()

        self.scene.terrain.terrain_type = "plane"
        self.scene.terrain.terrain_generator = None

        self.scene.height_scanner = None
        self.observations.policy.height_scan = None
        self.observations.critic.height_scan = None

        self.curriculum.terrain_levels = None
