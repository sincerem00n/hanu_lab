# Copyright (c) 2025, RAI Hanumanoid Project Developers.
# All rights reserved.

# SPDX-License-Identifier: BSD-3-Clause

from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass

from .rough_env_cfg import HanuA4RoughEnvCfgV0, HanuA4RoughEnvCfgV1, HanuA4RoughEnvCfgV2, HanuA4RoughEnvCfgV3, HanuA4RoughEnvCfgV4, HanuA4RoughEnvCfgV5, HanuA4RoughEnvCfgV6, HanuA4RoughEnvCfgV7

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

@configclass
class HanuA4FlatEnvCfgV2(HanuA4RoughEnvCfgV2):
    """Configuration for the flat environment in the RAI Hanumanoid project. -- For Play."""
    def __post_init__(self):
        super().__post_init__()

        self.scene.terrain.terrain_type = "plane"
        self.scene.terrain.terrain_generator = None

        self.scene.height_scanner = None
        self.observations.policy.height_scan = None
        self.observations.critic.height_scan = None

        self.curriculum.terrain_levels = None

@configclass
class HanuA4FlatEnvCfgV3(HanuA4RoughEnvCfgV3):
    """Configuration for the flat environment in the RAI Hanumanoid project. -- For Play."""
    def __post_init__(self):
        super().__post_init__()

        self.scene.terrain.terrain_type = "plane"
        self.scene.terrain.terrain_generator = None

        self.scene.height_scanner = None
        self.observations.policy.height_scan = None
        self.observations.critic.height_scan = None

        self.curriculum.terrain_levels = None

@configclass
class HanuA4FlatEnvCfgV4(HanuA4RoughEnvCfgV4):
    """Configuration for the flat environment in the RAI Hanumanoid project. -- New baseline"""
    def __post_init__(self):
        super().__post_init__()

        self.scene.terrain.terrain_type = "plane"
        self.scene.terrain.terrain_generator = None

        self.scene.height_scanner = None
        self.observations.policy.height_scan = None
        self.observations.critic.height_scan = None

        self.curriculum.terrain_levels = None


@configclass
class HanuA4FlatEnvCfgV5(HanuA4RoughEnvCfgV5):
    """Configuration for the flat environment in the RAI Hanumanoid project. -- target q + Imu"""
    def __post_init__(self):
        super().__post_init__()

        self.scene.terrain.terrain_type = "plane"
        self.scene.terrain.terrain_generator = None

        self.scene.height_scanner = None
        self.observations.policy.height_scan = None
        self.observations.critic.height_scan = None

        self.curriculum.terrain_levels = None

@configclass
class HanuA4FlatEnvCfgV6(HanuA4RoughEnvCfgV6):
    """Configuration for the flat environment in the RAI Hanumanoid project. -- delay , history length"""
    def __post_init__(self):
        super().__post_init__()

        self.scene.terrain.terrain_type = "plane"
        self.scene.terrain.terrain_generator = None

        self.scene.height_scanner = None
        self.observations.policy.height_scan = None
        self.observations.critic.height_scan = None

        self.curriculum.terrain_levels = None

@configclass
class HanuA4FlatEnvCfgV7(HanuA4RoughEnvCfgV7):
    """Configuration for the flat environment in the RAI Hanumanoid project. -- delay , history length"""
    def __post_init__(self):
        super().__post_init__()

        self.scene.terrain.terrain_type = "plane"
        self.scene.terrain.terrain_generator = None

        self.scene.height_scanner = None
        self.observations.policy.height_scan = None
        self.observations.critic.height_scan = None

        self.curriculum.terrain_levels = None