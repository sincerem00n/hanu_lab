# Copyright (c) 2025, RAI Hanumanoid Project Developers.
# All rights reserved.

# SPDX-License-Identifier: BSD-3-Clause

import math

# import isaacsim.asset.importer.urdf
import omni.usd
from pxr import UsdPhysics

###########################
# Pre-defined configs
###########################
from isaaclab_assets import HANU_A2_CFG


from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass

import isaaclab_tasks.manager_based.locomotion.velocity.mdp as mdp
from isaaclab_tasks.manager_based.locomotion.velocity.velocity_env_cfg import LocomotionVelocityRoughEnvCfg, RewardsCfg, TerminationsCfg, CommandsCfg, EventCfg

from .env import rewards

@configclass
class HanuA2RewardsCfg(RewardsCfg):
    termination_penalty = RewTerm(func=mdp.is_terminated, weight=-200.0)
    feet_air_time = RewTerm(
        func=mdp.feet_air_time_positive_biped,
        weight=0.25,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*_foot_.*"),
            "command_name": "base_velocity",
            "threshold": 0.4,
        },
    )
    track_lin_vel_xy_exp = RewTerm(
        func=mdp.track_lin_vel_xy_yaw_frame_exp,
        weight=1.0,
        params={
            "command_name": "base_velocity",
            "std": 0.5,
            # "asset_cfg": SceneEntityCfg("robot", body_names=["Hip_1"]),
        },
    )
    track_ang_vel_z_exp = RewTerm(
        func=mdp.track_ang_vel_z_world_exp, 
        weight=2.0,
        params={
            "command_name": "base_velocity",
            "std": 0.5,
            # "asset_cfg": SceneEntityCfg("robot", body_names=["Hip_1"]),
        }
    )
    feet_slide = RewTerm(
        func=mdp.feet_slide,
        weight=-0.1,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*_foot_.*"),
            "asset_cfg": SceneEntityCfg("robot", body_names=".*_foot_.*"),
        },
    )

    upright_orientation = RewTerm(
        func=rewards.upright_orientation_l2,
        weight=1.0,
        params={
            "asset_cfg": SceneEntityCfg("robot"), 
        }
    )

    # ----- ankle joint limits penalty
    ankle_dof_pos_limits = RewTerm(
        func=mdp.joint_pos_limits,
        weight=-1.0,
        params={
            "asset_cfg": SceneEntityCfg(
                "robot", 
                joint_names=".*_ankle_.*"
            ),
        },
    )
    # ----- joint deviation penalty
    # joint_deviation_neck = RewTerm(
    #     func=mdp.joint_deviation_l1,
    #     weight=-0.2,
    #     params={
    #         "asset_cfg": SceneEntityCfg("robot", joint_names=["neck_to_torso_Pitch"])
    #     },
    # )
    joint_deviation_arms = RewTerm(
        func=mdp.joint_deviation_l1,
        weight=-0.1,
        params={
            "asset_cfg": SceneEntityCfg(
                "robot",
                joint_names=[
                    ".*_shoulder_.*",
                    ".*_elbow_.*",
                    ".*_wrist_.*",
                ],
            )
        },
    )
    # joint_deviation_torso = RewTerm(
    #     func=mdp.joint_deviation_l1,
    #     weight=-0.1,
    #     params={
    #         "asset_cfg": SceneEntityCfg(
    #             "robot", 
    #             joint_names=[
    #                 "abdomen_to_hip_Pitch",
    #                 "base_to_neck_yaw",
    #                 "neck_to_torso_Pitch",
    #                 "torso_to_LA1_Shoulder_Pitch",
    #                 "torso_to_RA1_Shoulder_Pitch",
    #                 "torso_to_abdomen_Yaw",
    #             ]
    #         )
    #     },
    # )
    

@configclass
class HanuA2TerminationsCfg(TerminationsCfg):
    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    base_contact = DoneTerm(
        func=mdp.illegal_contact,
        params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names="base"), "threshold": 1.0},
    )
    # robot_fallen = DoneTerm(
    #     func=mdp.bad_orientation,
    #     params={
    #         "asset_cfg": SceneEntityCfg("robot"),
    #         "limit_angle": math.pi/3,  # 60 degrees
    #     },
    # )
    # minimum_height = DoneTerm(
    #     func=mdp.root_height_below_minimum,
    #     params={
    #         "asset_cfg": SceneEntityCfg("robot"),
    #         "minimum_height": 0.45
    #     }
    # )


@configclass
class HanuA2EventsCfg(EventCfg):
    """Events configuration for Hanumanoid A2."""
    base_external_force_torque = EventTerm(
        func=mdp.apply_external_force_torque,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="base"),
            "force_range": (0.0, 0.0),
            "torque_range": (-0.0, 0.0),
        },
    )

    reset_base = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5), "yaw": (-3.14, 3.14)},
            "velocity_range": {
                "x": (-0.5, 0.5),
                "y": (-0.5, 0.5),
                "z": (-0.5, 0.5),
                "roll": (-0.5, 0.5),
                "pitch": (-0.5, 0.5),
                "yaw": (-0.5, 0.5),
            },
        },
    )

    reset_robot_joints = EventTerm(
        func=mdp.reset_joints_by_scale,
        mode="reset",
        params={
            "position_range": (0.5, 1.5),
            "velocity_range": (0.0, 0.0),
        },
    )

@configclass
class HanuA2RoughEnvCfg(LocomotionVelocityRoughEnvCfg):
    """Environment configuration for Hanumanoid A2 in rough terrain."""

    rewards: HanuA2RewardsCfg = HanuA2RewardsCfg()
    terminations: HanuA2TerminationsCfg = HanuA2TerminationsCfg()

    foot_link_name = ".*_foot_.*"

    def __post_init__(self):
        super().__post_init__()

        # ------ Scene configuration --------
        self.scene.robot = HANU_A2_CFG.replace(prim_path="{ENV_REGEX_NS}/robot")   
        self.scene.height_scanner.prim_path = "/World/envs/env_.*/robot/hanumanoid/fixed_E1R"
        self.scene.contact_forces.prim_path = "{ENV_REGEX_NS}/robot/hanumanoid/.*"

        self.scene.terrain.terrain_generator.sub_terrains["boxes"].grid_height_range = (0.025, 0.1)
        self.scene.terrain.terrain_generator.sub_terrains["random_rough"].noise_range = (0.01, 0.06)
        self.scene.terrain.terrain_generator.sub_terrains["random_rough"].noise_step = 0.01

        # ------ Events configuration --------
        # self.events.reset_robot_joints = None
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

        # ------ Rewards configuration --------
        # penalties
        self.rewards.track_lin_vel_xy_exp.weight = 1.5
        self.rewards.track_ang_vel_z_exp.weight = 1.0
        self.rewards.lin_vel_z_l2.weight = -0.2
        self.rewards.undesired_contacts.params["sensor_cfg"].body_names = [
            ".*_groin_.*",
            ".*_glute_.*",
        ]
        self.rewards.undesired_contacts.weight = -0.2
        self.rewards.feet_air_time.weight = 0.1 # g1_flat: 0.75
        self.rewards.feet_air_time.params["threshold"] = 0.4 # g1 flat
        # self.rewards.base_height_l2.weight = 1.5
        self.rewards.action_rate_l2.weight = -0.005
        self.rewards.dof_acc_l2.weight = -1.0e-7 # g1_flat
        self.rewards.dof_acc_l2.params["asset_cfg"] = SceneEntityCfg(
            "robot", 
            joint_names=[
                ".*_hip_pitch",
            ]
        )
        self.rewards.dof_torques_l2.weight = -2.0e-6 # g1_flat
        self.rewards.dof_torques_l2.params["asset_cfg"] = SceneEntityCfg(
            "robot", 
            joint_names=[
                ".*_hip_.*",
                ".*_knee_.*",
                ".*_ankle_.*",
            ]
        )
        # self.rewards.termination_penalty.weight = -2.0
        

        # ------ Commands configuration --------
        self.commands.base_velocity.ranges.lin_vel_x = (-0.0, 0.0)
        self.commands.base_velocity.ranges.lin_vel_y = (-0.0, 1.0) # (-1.0, 0.0)
        self.commands.base_velocity.ranges.ang_vel_z = (-0.0, 0.0)
        # self.commands.base_velocity.rel_standing_envs = 0.5

        # ------ Observations configuration --------
        self.observations.policy.enable_corruption = False

        # ------ Terminations configuration --------
        self.terminations.base_contact.params["sensor_cfg"].body_names = "torso_.*"
        # self.terminations.base_contact.params["sensor_cfg"].body_names = [f"^(?!.*{self.foot_link_name}).*"]

@configclass
class HanuA2RoughEnvCfgV0(HanuA2RoughEnvCfg):
    """
    Environment configuration for Hanumanoid A2 in rough terrain - Version 0.
    Changes made in V0:
    - No events
    """
    def __post_init__(self):
        super().__post_init__()

        # ------ Events configuration --------
        self.events.push_robot = None
        self.events.add_base_mass = None
        self.events.base_external_force_torque = None
        self.events.base_com = None

        # ------ Rewards configuration --------
        self.rewards.action_rate_l2.weight = -0.01
        # ------ Terminations configuration --------
        self.terminations.base_contact.params["sensor_cfg"].body_names = [f"^(?!.*{self.foot_link_name}).*"]


@configclass
class HanuA2RoughEnvCfgV1(HanuA2RoughEnvCfg):
    """
    Environment configuration for Hanumanoid A2 in rough terrain - Version 1.
    Changes made in V1:
    - Add events
    """
    def __post_init__(self):
        super().__post_init__()

        # ------ Events configuration --------
        self.events.add_base_mass.params["asset_cfg"].body_names = "torso_.*"
        self.events.add_base_mass.params["mass_distribution_params"] = (-0.5, 1.5)
        # self.events.base_com.params["asset_cfg"].body_names = "torso_.*"
        # self.events.base_com.params["com_range"] = {
        #     "x": (-0.05, 0.05), 
        #     "y": (-0.05, 0.05), 
        #     "z": (-0.01, 0.01)
        # }
        self.events.base_com = None  # disabling for now
        self.events.base_external_force_torque.params["asset_cfg"].body_names = "torso_.*"
        self.events.base_external_force_torque.params["force_range"] = (-0.75, 1.25)

        self.events.reset_robot_joints.params["position_range"] = (0.5, 1.5)
        self.events.reset_base.params = {
            "pose_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5), "yaw": (-3.14, 3.14)},
            "velocity_range": {
                "x": (-0.5, 0.5),
                "y": (-0.5, 0.5),
                "z": (-0.5, 0.5),
                "roll": (-0.5, 0.5),
                "pitch": (-0.5, 0.5),
                "yaw": (-0.5, 0.5),
            },
        }

        # ------- Rewards configuration --------
        self.rewards.feet_air_time.weight = 0.3
        self.rewards.feet_air_time.params["threshold"] = 0.2
        self.rewards.feet_slide.weight = -0.2

        # ------ Commands configuration --------
        self.commands.base_velocity.ranges.lin_vel_y = (-0.0, 1.5) # (-1.0, 0.0)
        self.commands.base_velocity.ranges.ang_vel_z = (-0.0, 0.0)

        # ------ Terminations configuration --------
        self.terminations.base_contact.params["sensor_cfg"].body_names = [f"^(?!.*{self.foot_link_name}).*"]