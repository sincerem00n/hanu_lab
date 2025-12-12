# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

# Copyright (c) 2025, RAI Hanumanoid Project Developers.
# All rights reserved.

# SPDX-License-Identifier: BSD-3-Clause

import math

import isaacsim.asset.importer.urdf
import omni.usd
from pxr import UsdPhysics

###########################
# Pre-defined configs
###########################
from hanu_lab.assets import HANU_A0_CFG

from isaaclab.assets import Articulation
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass

import hanu_lab.tasks.manager_based.locomotion.velocity.mdp as mdp
from hanu_lab.tasks.manager_based.locomotion.velocity.velocity_env_cfg import LocomotionVelocityRoughEnvCfg, RewardsCfg


@configclass
class HanuRewardsCfg(RewardsCfg):
    termination_penalty = RewTerm(func=mdp.is_terminated, weight=-200.0)
    feet_air_time = RewTerm(
        func=mdp.feet_air_time_positive_biped,
        weight=0.25,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=["L13_Foot_1_1", "R13_Foot_1_1"]),
            "command_name": "base_velocity",
            "threshold": 0.4,
        },
    )
    joint_deviation_hip = RewTerm(
        func=mdp.joint_deviation_l1,
        weight=-0.7,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["base_to_.*"])},
    )
    joint_deviation_toes = RewTerm(
        func=mdp.joint_deviation_l1,
        weight=-0.1,  # cassie:-0.2
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["L6_to_L13_Rev", "R6_to_R13_Rev"])},
    )
    # penalize toe joint limits
    dof_pos_limits = RewTerm(
        func=mdp.joint_pos_limits,
        weight=-0.2,  # cassie:-1.0
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["L6_to_L13_Rev", "R6_to_R13_Rev"])},
    )

    base_height_l2 = RewTerm(
        func=mdp.base_height_l2,
        weight=0.9,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=["base_link"]),
            "target_height": 0.83,
        },  # "target": 0.35         target not a param of base_pos_z
    )

    joint_deviation_knee = RewTerm(
        func=mdp.joint_deviation_l1,
        weight=-0.2,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["L4_to_L5_Rev", "R4_to_R5_Rev"])},
    )

    track_ang_vel_z_exp = RewTerm(
        func=mdp.track_ang_vel_z_world_exp,
        weight=0.5,
        params={
            "command_name": "base_velocity",
            "std": 0.5,
            "asset_cfg": SceneEntityCfg("robot", body_names=["base_link"]),
        },
    )
    track_lin_vel_xy_exp = RewTerm(
        func=mdp.track_lin_vel_xy_yaw_frame_exp,
        weight=2.0,
        params={
            "command_name": "base_velocity",
            "std": 0.5,
            "asset_cfg": SceneEntityCfg("robot", body_names=["base_link"]),
        },
    )


@configclass
class HanumanoidA0RoughEnvCfg(LocomotionVelocityRoughEnvCfg):

    rewards: HanuRewardsCfg = HanuRewardsCfg()
    # scene: SceneCfg = MySceneCfg(num_envs=4096, env_spacing=2.5)

    def __post_init__(self):
        super().__post_init__()

        # scene
        # self.scene.robot.prim_path = "{ENV_REGEX_NS}/LegV5_URDF_Export"
        self.scene.robot = HANU_A0_CFG.replace(prim_path="{ENV_REGEX_NS}/LegV5_URDF_Export")  #! TODO: check dir
        # self.scene.robot = HANU_A0_CFG

        self.scene.height_scanner.prim_path = (  # TODO: change Robot name
            "{ENV_REGEX_NS}/LegV5_URDF_Export/LegV5_URDF_Export/base_link"
        )
        # /World/LegV5_URDF_Export/base_link

        self.scene.contact_forces.prim_path = "{ENV_REGEX_NS}/LegV5_URDF_Export/LegV5_URDF_Export/.*"

        # check if the rigid bodies are loaded correctly
        # stage = omni.usd.get_context().get_stage()
        # world_prim = stage.GetPrimAtPath("/World")
        # print("All Prims in /World:")
        # for prim in world_prim.GetChildren():
        #     print(prim.GetPath().pathString)

        # for prim in stage.Traverse():
        #     print(f"Prim: {prim.GetPath().pathString}")
        # base_link_path = "/World/LegV5_URDF_Export/base_link"
        # base_link = stage.GetPrimAtPath(base_link_path)
        # if base_link.IsValid():
        #     print(f"Base link found: {base_link.GetName()}")
        # else:
        #     print(f"Base link not found at path: {base_link_path}")

        # for prim in stage.Traverse():
        #     if prim.IsA(UsdPhysics.RigidBodyAPI):
        #         print(f"Rigid body found: {prim.GetName()}")

        self.scene.terrain.terrain_generator.sub_terrains["boxes"].grid_height_range = (0.025, 0.1)
        self.scene.terrain.terrain_generator.sub_terrains["random_rough"].noise_range = (0.01, 0.06)
        self.scene.terrain.terrain_generator.sub_terrains["random_rough"].noise_step = 0.01

        # action scale
        self.actions.joint_pos.scale = 0.5
        # self.actions.joint_vel.scale = 1.0
        # self.actions.joint_torque.scale = 1.0

        # events
        self.events.push_robot = None
        self.events.add_base_mass.params["mass_distribution_params"] = (-1.0, 3.0)
        self.events.add_base_mass.params["asset_cfg"].body_names = "base_link"  # TODO: check "base" link
        self.events.base_external_force_torque.params["asset_cfg"].body_names = "base_link"
        self.events.reset_robot_joints.params["position_range"] = (1.0, 1.0)
        self.events.reset_base.params = {
            "pose_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5), "yaw": (-3.14, 3.14)},
            "velocity_range": {
                "x": (0.0, 0.0),
                "y": (0.0, 0.0),
                "z": (0.0, 0.0),
                "roll": (0.0, 0.0),
                "pitch": (0.0, 0.0),
                "yaw": (0.0, 0.0),
            },
        }

        # rewards
        # self.rewards.feet_air_time.params["sensor_cfg"].body_names = ["L13_Foot_1_1", "R13_Foot_1_1"] # TODO: check ".*_foot" link
        # self.rewards.feet_air_time.weight = 0.3 # default: 0.125
        self.rewards.lin_vel_z_l2.weight = 0.0
        self.rewards.undesired_contacts.params["sensor_cfg"].body_names = [
            "base_link",
            "L1_Hip_1_1",
            "R1_Hip_1_1",
            "L2_Hip_1_1",
            "R2_Hip_1_1",
            "L3_Upper_1_1",
            "R3_Upper_1_1",
            "L4_Lower_1_1",
            "R4_Lower_1_1",
            "L11_Knee_Pitch_1_1",
            "L5_Knee_Roll_1_1",
            "L6_U_Joint_1_1",
            "R11_Knee_Pitch_1_1",
            "R5_Knee_Roll_1_1",
            "R6_U_Joint_1_1",
        ]
        # self.rewards.undesired_contacts.params["sensor_cfg"].body_names = "base_link" # TODO: check ".*_thigh" link
        self.rewards.undesired_contacts.weight = -0.1  # default: -0.1
        self.rewards.flat_orientation_l2.weight = -1.0
        self.rewards.action_rate_l2.weight = -0.005
        self.rewards.dof_acc_l2.weight = -1.25e-7
        self.rewards.dof_torques_l2.weight = -0.0002

        self.rewards.feet_air_time.weight = 0.25
        self.rewards.base_height_l2.weight = 2.0

        # commands
        self.commands.base_velocity.ranges.lin_vel_x = (-0.5, 0.5)
        self.commands.base_velocity.ranges.lin_vel_y = (-1.0, 0.0)
        self.commands.base_velocity.ranges.ang_vel_z = (-1.0, 1.0)

        # observations
        self.observations.policy.enable_corruption = False

        # terminations
        self.terminations.base_contact.params["sensor_cfg"].body_names = [
            "base_link",
            "L1_Hip_1_1",
            "R1_Hip_1_1",
            "L2_Hip_1_1",
            "R2_Hip_1_1",
            "L3_Upper_1_1",
            "R3_Upper_1_1",
            "L4_Lower_1_1",
            "R4_Lower_1_1",
            "L11_Knee_Pitch_1_1",
            "L5_Knee_Roll_1_1",
            "L6_U_Joint_1_1",
            "R11_Knee_Pitch_1_1",
            "R5_Knee_Roll_1_1",
            "R6_U_Joint_1_1",
        ]


@configclass
class HanumanoidA0RoughEnvCfg_PLAY(HanumanoidA0RoughEnvCfg):
    def __post_init__(self) -> None:
        super().__post_init__()

        self.scene.num_envs = 32
        self.scene.env_spacing = 2.5

        self.scene.terrain.max_init_terrain_level = None

        if self.scene.terrain.terrain_generator is not None:
            self.scene.terrain.terrain_generator.num_rows = 5
            self.scene.terrain.terrain_generator.num_cols = 5
            self.scene.terrain.terrain_generator.curriculum = False

        self.commands.base_velocity.ranges.lin_vel_x = (-0.5, 0.5)
        self.commands.base_velocity.ranges.lin_vel_y = (-1.0, 0.0)
        # self.commands.base_velocity.ranges.ang_vel_z = (-1.0, 1.0)
        self.commands.base_velocity.ranges.heading = (0.0, 0.0)

        self.observations.policy.enable_corruption = False
        # remove random pushing event
        self.events.base_external_force_torque = None
        self.events.push_robot = None
