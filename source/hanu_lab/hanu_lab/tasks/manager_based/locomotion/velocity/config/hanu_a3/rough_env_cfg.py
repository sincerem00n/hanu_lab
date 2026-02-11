# Copyright (c) 2025, RAI Hanumanoid Project Developers.
# All rights reserved.

# SPDX-License-Identifier: BSD-3-Clause
from __future__ import annotations
import math

# import isaacsim.asset.importer.urdf
import omni.usd
from pxr import UsdPhysics

###########################
# Pre-defined configs
###########################
from hanu_lab.assets import HANU_A3_CFG


from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass

import hanu_lab.tasks.manager_based.locomotion.velocity.mdp as mdp
from hanu_lab.tasks.manager_based.locomotion.velocity.velocity_env_cfg import LocomotionVelocityRoughEnvCfg, RewardsCfg, TerminationsCfg, CommandsCfg, EventCfg



@configclass
class HanuA3RewardsCfg(RewardsCfg):
    termination_penalty = RewTerm(func=mdp.is_terminated, weight=-200.0)
    feet_air_time = RewTerm(
        func=mdp.feet_air_time_positive_biped,
        weight=0.05,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*_foot_.*"),
            "command_name": "base_velocity",
            "threshold": 0.12,
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
        weight=-0.2,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*_foot_.*"),
            "asset_cfg": SceneEntityCfg("robot", body_names=".*_foot_.*"),
        },
    )

    feet_mirror = RewTerm(
        func=mdp.action_mirror,
        weight=-0.08,
        params={
            "asset_cfg": SceneEntityCfg("robot"),
            "mirror_joints": [
                ["Joint_l_hip_pitch", "Joint_r_hip_pitch"],
                ["Joint_l_knee_pitch", "Joint_r_knee_pitch"],
                ["Joint_l_ankle_pitch", "Joint_r_ankle_pitch"],
            ],
        },
    )

    upright_orientation = RewTerm(
        func=mdp.upright_orientation_l2,
        weight=3.0,
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

    joint_vel_neck = RewTerm(
        func=mdp.joint_vel_l2,
        weight=-0.1,
        params={
            "asset_cfg": SceneEntityCfg(
                "robot",
                joint_names=[
                    ".*_neck_.*",
                ],
            )
        },
    )
    joint_vel_legs = RewTerm(
        func=mdp.joint_vel_l2,
        weight=-0.1,
        params={
            "asset_cfg": SceneEntityCfg(
                "robot",
                joint_names=[
                    ".*_hip_yaw",
                ],
            )
        },
    )
    joint_deviation_neck = RewTerm(
        func=mdp.joint_deviation_l1,
        weight=-0.1,
        params={
            "asset_cfg": SceneEntityCfg(
                "robot",
                joint_names=[
                    ".*_neck_.*",
                ],
            )
        },
    )

    # New #
        
#    feet_step_sequence = RewTerm(
#        func=mdp.feet_step_sequence_biped,
#        weight=0.1,
#        params={
#            "sensor_cfg": SceneEntityCfg(
#                "contact_forces",
#                body_names=“.*_foot.*”,
#                preserve_order=True,
#            ),
#            "command_name": "base_velocity",
#        },
#    )


    feet_air_time_penalty = RewTerm(
        func=mdp.feet_air_time_negative_biped,
        weight=-0.05, 
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*_foot_.*"),
            "command_name": "base_velocity",
            "threshold": 0.12,
        },
    )


@configclass
class HanuA3TerminationsCfg(TerminationsCfg):
    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    base_contact = DoneTerm(
        func=mdp.illegal_contact,
        params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names="base_link"), "threshold": 1.0},
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
class HanuA3EventsCfg(EventCfg):
    """Events configuration for Hanumanoid A3."""
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

import math
from isaaclab.utils import configclass

@configclass
class HanuA3RoughEnvCfg(LocomotionVelocityRoughEnvCfg):
    """Environment configuration for Hanumanoid A3 in rough terrain."""

    rewards: HanuA3RewardsCfg = HanuA3RewardsCfg()
    terminations: HanuA3TerminationsCfg = HanuA3TerminationsCfg()

    foot_link_name = ".*_foot_.*"

    def __post_init__(self):
        super().__post_init__()

        # ------ Scene configuration --------
        # self.scene.robot = HANU_A3_CFG.replace(prim_path="{ENV_REGEX_NS}/robot")
        # self.scene.height_scanner.prim_path = "/World/envs/env_.*/robot/hanu_a3/E1R_1"
        # self.scene.contact_forces.prim_path = "{ENV_REGEX_NS}/robot/hanu_a3/.*"
        # self.scene.imu_sensor.prim_path = "{ENV_REGEX_NS}/robot/hanu_a3/base_link"


        # ------ Scene configuration --------
        self.scene.robot = HANU_A3_CFG.replace(prim_path="{ENV_REGEX_NS}/robot")

        if self.scene.height_scanner is not None:
            self.scene.height_scanner.prim_path = "/World/envs/env_.*/robot/hanu_a3/E1R_1"

        self.scene.contact_forces.prim_path = "{ENV_REGEX_NS}/robot/hanu_a3/.*"
         # ------  

        self.scene.terrain.terrain_generator.sub_terrains["boxes"].grid_height_range = (0.0, 0.02)
        self.scene.terrain.terrain_generator.sub_terrains["random_rough"].noise_range = (0.0, 0.02)
        self.scene.terrain.terrain_generator.sub_terrains["random_rough"].noise_step = 0.002

        # ------ Events configuration --------
        # self.events.reset_robot_joints = None
        self.events.reset_robot_joints.params["position_range"] = (0.95, 1.05)
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
        self.commands.base_velocity.ranges.lin_vel_x = (0.25, 0.55)
        self.commands.base_velocity.ranges.lin_vel_y = (0.0, 0.0)
        self.commands.base_velocity.ranges.ang_vel_z = (0.0, 0.0)
        self.commands.base_velocity.rel_standing_envs = 0.0
        # self.commands.base_velocity.rel_standing_envs = 0.5

        # ------ Observations configuration --------
        self.observations.policy.enable_corruption = False

        # ------ Terminations configuration --------
        self.terminations.base_contact.params["sensor_cfg"].body_names = "base_.*"
        # self.terminations.base_contact.params["sensor_cfg"].body_names = [f"^(?!.*{self.foot_link_name}).*"]


# =============================================================================


from isaaclab.utils import configclass

@configclass
class HanuA3RoughEnvCfgV0(HanuA3RoughEnvCfg):

    def __post_init__(self):
        super().__post_init__()

        # ==========================================================
        # OBSERVATIONS CONFIGURATION
        # ==========================================================
        # Scale observations for stable learning
        self.observations.policy.base_lin_vel.scale = 2.0
        self.observations.policy.base_ang_vel.scale = 0.25
        self.observations.policy.joint_pos.scale = 1.0
        self.observations.policy.joint_vel.scale = 0.05

        # Disable height scan to keep obs dim = 102
        self.observations.policy.height_scan = None
        if self.observations.critic is not None:
            self.observations.critic.height_scan = None

        # ==========================================================
        # ACTIONS CONFIGURATION
        # ==========================================================
        # Smaller action scale = smoother gait, less hopping
        self.actions.joint_pos.scale = 0.25
        self.actions.joint_pos.clip = {".*": (-100.0, 100.0)}

        # ==========================================================
        # EVENTS / DOMAIN RANDOMIZATION
        # ==========================================================
        # Randomize base mass slightly (robustness)
        self.events.add_base_mass.params["asset_cfg"].body_names = "base_.*"
        self.events.add_base_mass.params["mass_distribution_params"] = (-0.3, 0.8)

        # Disable COM and external force randomization (stability)
        self.events.base_com = None
        self.events.base_external_force_torque = None

        # Randomize joint reset slightly
        self.events.reset_robot_joints.params["position_range"] = (0.8, 1.2)

        # IMPORTANT:
        # Do NOT randomize yaw at reset → prevents sideways walking illusion
        self.events.reset_base.params = {
            "pose_range": {
                "x": (-0.5, 0.5),
                "y": (-0.5, 0.5),
                "yaw": (0.0, 0.0),   # keep heading straight
            },
            "velocity_range": {
                "x": (0.0, 0.0),
                "y": (0.0, 0.0),
                "z": (0.0, 0.0),
                "roll": (0.0, 0.0),
                "pitch": (0.0, 0.0),
                "yaw": (0.0, 0.0),
            },
        }

        # ==========================================================
        # COMMANDS CONFIGURATION
        # ==========================================================
        self.commands.base_velocity.ranges.lin_vel_y = (-0.0, 1.0) # (-1.0, 0.0)
        self.commands.base_velocity.ranges.ang_vel_z = (-0.5, 0.5)
        self.commands.base_velocity.ranges.ang_vel_z = (0.0, 0.0)

        # No standing environments
        self.commands.base_velocity.rel_standing_envs = 0.0

        # ==========================================================
        # REWARDS CONFIGURATION
        # ==========================================================

        # --- Tracking rewards (main objective) ---
        self.rewards.track_lin_vel_xy_exp.weight = 1.5
        self.rewards.track_ang_vel_z_exp.weight = 1.0

        # --- Upright posture ---
        self.rewards.upright_orientation.weight = 3.0

        # --- Anti-hopping / anti-jumping ---
        self.rewards.lin_vel_z_l2.weight = -1.0      # penalize vertical motion
        self.rewards.ang_vel_xy_l2.weight = -0.2     # penalize roll & pitch rates

        # Reduce incentive to keep feet in the air
        self.rewards.feet_air_time.weight = 0.05
        self.rewards.feet_air_time.params["threshold"] = 0.15

        # --- Foot behavior ---
        self.rewards.feet_slide.weight = -0.4
        self.rewards.feet_mirror.weight = -0.0

        # --- Smoothness penalties ---
        self.rewards.action_rate_l2.weight = -0.002
        self.rewards.joint_vel_legs.weight = -0.15
        self.rewards.joint_vel_neck.weight = -0.2

        # --- Joint safety / realism ---
        self.rewards.ankle_dof_pos_limits.weight = -1.0
        self.rewards.knee_pose_deviation.weight = -0.08
        self.rewards.knee_dof_pos_limits.weight = -0.2
        self.rewards.joint_deviation_arms.weight = -0.1
        self.rewards.joint_deviation_neck.weight = -0.1

        # --- Termination penalty ---
        self.rewards.termination_penalty.weight = -200.0

        # ==========================================================
        # TERMINATIONS CONFIGURATION
        # ==========================================================
        # Terminate when non-foot body parts touch the ground
        self.terminations.base_contact.params["sensor_cfg"].body_names = [
            f"^(?!.*{self.foot_link_name}).*"
        ]


# ------------------------------------------------------------------------------------------

from isaaclab.utils import configclass

@configclass
class HanuA3RoughEnvCfgV1(HanuA3RoughEnvCfg):

    def __post_init__(self):
        super().__post_init__()

        # ------ Observations configuration --------
        self.observations.policy.base_lin_vel.scale = 2.0
        self.observations.policy.base_ang_vel.scale = 0.25
        self.observations.policy.joint_pos.scale = 1.0
        self.observations.policy.joint_vel.scale = 0.05

        # Disable unused observations
        self.observations.policy.base_lin_vel = None
        self.observations.policy.height_scan = None

        # ------ Actions configuration --------
        self.actions.joint_pos.scale = 0.25
        self.actions.joint_pos.clip = {".*": (-100.0, 100.0)}

        # ------ Events configuration --------
        self.events.add_base_mass.params["asset_cfg"].body_names = "base_.*"
        self.events.add_base_mass.params["mass_distribution_params"] = (-0.5, 1.5)

        self.events.base_com = None
        self.events.base_external_force_torque = None

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
        self.rewards.track_lin_vel_xy_exp.weight = 1.5
        self.rewards.feet_air_time.weight = 1.0
        self.rewards.feet_air_time.params["threshold"] = 0.18
        self.rewards.feet_slide.weight = -0.1
        self.rewards.feet_mirror.weight = -0.0
        self.rewards.action_rate_l2.weight = -0.005

        #self.rewards.knee_pose_deviation.weight = -0.0
        #self.rewards.knee_dof_pos_limits.weight = -0.0
        self.rewards.joint_vel_legs.weight = -0.3
        self.rewards.joint_vel_neck.weight = -0.5

        # ---- Added gait rewards only ----
        #self.rewards.feet_step_sequence.weight = 0.4
        self.rewards.feet_air_time_penalty.weight = -0.03
        self.rewards.feet_air_time_penalty.params["threshold"] = 0.28

        # ------ Commands configuration --------
        self.commands.base_velocity.ranges.lin_vel_y = (-0.0, 1.0) # (-1.0, 0.0)
        self.commands.base_velocity.ranges.ang_vel_z = (-0.5, 0.5)
        self.commands.base_velocity.rel_standing_envs = 0.3

        # ------ Terminations configuration --------
        # self.terminations.base_contact.params["sensor_cfg"].body_names = "base_.*"
        self.terminations.base_contact.params["sensor_cfg"].body_names = [f"^(?!.*{self.foot_link_name}).*"]

