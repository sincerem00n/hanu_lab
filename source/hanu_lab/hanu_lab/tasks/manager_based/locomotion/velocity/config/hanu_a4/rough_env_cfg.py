# Copyright (c) 2025, RAI Hanumanoid Project Developers.
# All rights reserved.

# SPDX-License-Identifier: BSD-3-Clause
from __future__ import annotations
import math

from torch import func

# import isaacsim.asset.importer.urdf
from hanu_lab.tasks.manager_based.locomotion.velocity.mdp.observation import zero_gait_phase
import omni.usd
from pxr import UsdPhysics

###########################
# Pre-defined configs
###########################
from hanu_lab.assets import HANU_A4_CFG, HANU_A4_IM_CFG, HANU_A4_TEST_CFG, HANU_A4_FW_CFG, HANU_A4_N_BASE_CFG, HANU_A4_EX_CFG

from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise

import hanu_lab.tasks.manager_based.locomotion.velocity.mdp as mdp
from hanu_lab.tasks.manager_based.locomotion.velocity.velocity_env_cfg import LocomotionVelocityRoughEnvCfg, RewardsCfg, TerminationsCfg, CommandsCfg, EventCfg, ObservationsCfg



@configclass
class HanuA4RewardsCfg(RewardsCfg):
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
                    # ".*_wrist_.*",
                ],
            )
        },
    )

    joint_deviation_legs = RewTerm(
        func=mdp.joint_deviation_l1,
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

    joint_vel_abs = RewTerm(
        func=mdp.joint_vel_l2,
        weight=-0.5,
        params={
            "asset_cfg":SceneEntityCfg(
            "robot",
            joint_names=[
                ".*_abdomen_.*",
            ]
            )
        },
    )

    joint_vel_arms = RewTerm(
        func=mdp.joint_vel_l2,
        weight=-0.1,
        params={
            "asset_cfg":SceneEntityCfg(
                "robot",
                joint_names=[
                    ".*_shoulder_.*",
                    ".*_elbow_.*",
                    # ".*_wrist_.*",
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

    feet_step_sequence = RewTerm(
        func=mdp.feet_step_sequence_biped,
        weight=0.1,
        params={
            "sensor_cfg": SceneEntityCfg(
                "contact_forces",
                body_names=".*_foot_.*",
                preserve_order=True,
            ),
            "command_name": "base_velocity",
            "threshold_steps": 1.5,
        },
    )
        


    feet_air_time_penalty = RewTerm(
        func=mdp.feet_air_time_negative_biped,
        weight=-0.05,
        params={
            "sensor_cfg": SceneEntityCfg(
                "contact_forces",
                body_names=[".*_foot_.*"],
            ),
            "command_name": "base_velocity",
            "threshold": 0.12,
        },
    )

    feet_lateral_sep_reward = RewTerm(
        func=mdp.feet_lateral_separation_reward,
        weight=0.2,
        params={
            "sensor_cfg": SceneEntityCfg(
                "robot",
                body_names=[".*_calf_pitch_.*"],  
            ),
            "command_name": "base_velocity",
            "threshold": 0.15,
            "margin": 0.03,
        },
    )


    arms_away_from_body = RewTerm(
        func=mdp.arms_lateral_open_pose,
        weight=0.35,
        params={
            "arm_sensor_cfg": SceneEntityCfg(
                "robot",
                body_names=[".*_lowerarm_.*"]
            ),
            "min_lateral_dist": 0.22,
            "max_backward_dist": 0.10,
            "lateral_margin": 0.05,
            "backward_margin": 0.05,
            "command_name": "base_velocity",
        },
    )

    # ============= NO CONTACT SENSOR ==============

    ref_joint_pos = RewTerm(
        func=mdp.joint_pos_tracking_exp,
        weight=2.0,
        params={"std": 0.2},
    )
    ref_joint_vel = RewTerm(
        func=mdp.joint_vel_tracking_exp,
        weight=0.1,
        params={"std": 1.5},
    )

    foot_pos_tracking = RewTerm(
        func=mdp.foot_pos_tracking_exp,
        weight=1.0,
        params={
            "foot_cfg": SceneEntityCfg(
                "robot", 
                body_names=".*_foot_.*",
                ),
            "std": 0.05,
        },
    )


@configclass
class HanuA4TerminationsCfg(TerminationsCfg):
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
class HanuA4EventsCfg(EventCfg):
    """Events configuration for Hanumanoid A4."""
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
class HanuA4RoughEnvCfg(LocomotionVelocityRoughEnvCfg):
    """Environment configuration for Hanumanoid A4 in rough terrain."""

    rewards: HanuA4RewardsCfg = HanuA4RewardsCfg()
    terminations: HanuA4TerminationsCfg = HanuA4TerminationsCfg()

    foot_link_name = ".*_foot_.*"

    def __post_init__(self):
        super().__post_init__()

        # ------ Scene configuration --------
        # self.scene.robot = HANU_A4_CFG.replace(prim_path="{ENV_REGEX_NS}/robot")
        # self.scene.height_scanner.prim_path = "/World/envs/env_.*/robot/hanu_a4/E1R_1"
        # self.scene.contact_forces.prim_path = "{ENV_REGEX_NS}/robot/hanu_a4/.*"
        # self.scene.imu_sensor.prim_path = "{ENV_REGEX_NS}/robot/hanu_a4/base_link"


        # ------ Scene configuration --------
        self.scene.robot = HANU_A4_CFG.replace(prim_path="{ENV_REGEX_NS}/robot")

        if self.scene.height_scanner is not None:
            self.scene.height_scanner.prim_path = "/World/envs/env_.*/robot/hanu_a4/E1R_1"

        self.scene.contact_forces.prim_path = "{ENV_REGEX_NS}/robot/hanu_a4/.*"
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

@configclass
class HanuA4RoughEnvCfgV0(HanuA4RoughEnvCfg):

    def __post_init__(self):
        super().__post_init__()

        # self.scene.robot = HANU_A4_TEST_CFG.replace(prim_path="{ENV_REGEX_NS}/robot")

        # ==========================================================
        # OBSERVATIONS CONFIGURATION
        # ==========================================================
        # Scale observations for stable learning
        # self.observations.policy.base_lin_vel.scale = 2.0
        self.observations.policy.base_ang_vel.scale = 0.25
        self.observations.policy.joint_pos.scale = 1.0
        self.observations.policy.joint_vel.scale = 0.05

        self.observations.policy.height_scan = None

        # ==========================================================
        # ACTIONS CONFIGURATION
        # ==========================================================
        self.actions.joint_pos.scale = 0.25
        self.actions.joint_pos.clip = {".*": (-100.0, 100.0)}

        # ==========================================================
        # EVENTS / DOMAIN RANDOMIZATION
        # ==========================================================
        self.events.add_base_mass.params["asset_cfg"].body_names = "base_.*"
        self.events.add_base_mass.params["mass_distribution_params"] = (-0.5, 0.15)

        self.events.base_com = None
        self.events.base_external_force_torque = None

        # Randomize joint reset slightly
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

        # ==========================================================
        # COMMANDS CONFIGURATION
        # ==========================================================
        self.commands.base_velocity.ranges.lin_vel_x = (0.0, 0.0)
        self.commands.base_velocity.ranges.lin_vel_y = (0.2, 0.5)  # (-1.0, 0.0)
        self.commands.base_velocity.ranges.ang_vel_z = (0.0, 0.0)

        self.commands.base_velocity.rel_standing_envs = 0.02

        # ==========================================================
        # REWARDS CONFIGURATION
        # ==========================================================
        self.rewards.track_lin_vel_xy_exp.weight = 2.0
        self.rewards.track_ang_vel_z_exp.weight = 0.5
        self.rewards.flat_orientation_l2 = None
        self.rewards.upright_orientation.weight = 2.0
        self.rewards.lin_vel_z_l2.weight = -0.5
        self.rewards.ang_vel_xy_l2.weight = -0.3

        # ------------------------------------------
        # self.rewards.feet_air_time.weight = 0.5
        # self.rewards.feet_air_time.params["threshold"] = 0.4

        # self.rewards.feet_air_time_penalty.weight = -0.01
        # self.rewards.feet_air_time_penalty.params["threshold"] = 0.38
        # ------------------------------------------
        self.rewards.feet_air_time = None
        self.rewards.feet_air_time_penalty = None

        self.rewards.feet_lateral_sep_reward = None
        self.rewards.arms_away_from_body = None
        # self.rewards.feet_lateral_sep_reward.weight = 0.15
        # self.rewards.arms_away_from_body.weight = 0.30
        # self.rewards.arms_away_from_body.params["min_lateral_dist"] = 0.20

        self.rewards.feet_slide = None
        self.rewards.feet_mirror = None
        # self.rewards.feet_mirror.weight = -0.02

        self.rewards.action_rate_l2.weight = -0.005
        # self.rewards.dof_acc_l2 = None
        self.rewards.dof_torques_l2.weight = -5.0e-7
        self.rewards.joint_vel_legs.weight = -0.2
        self.rewards.joint_vel_neck.weight = -0.25
        self.rewards.joint_vel_arms = None

        self.rewards.ankle_dof_pos_limits.weight = -0.2
        # self.rewards.knee_pose_deviation.weight = -0.0
        # self.rewards.knee_dof_pos_limits.weight = -0.0
        self.rewards.joint_deviation_arms.weight = -0.2
        self.rewards.joint_deviation_neck.weight = -0.1
        self.rewards.joint_deviation_legs.weight = -0.3

        self.rewards.feet_step_sequence = None

        # --- Termination penalty ---
        self.rewards.termination_penalty.weight = -200.0

        # ==========================================================
        # TERMINATIONS CONFIGURATION
        # ==========================================================
        self.terminations.base_contact.params["sensor_cfg"].body_names = [
            f"^(?!.*{self.foot_link_name}).*"
        ]



# ------------------------------------------------------------------------------------------

@configclass
class HanuA4RoughEnvCfgV1(HanuA4RoughEnvCfg):

    """
    Training Environment for motion tracking, new observations
    """

    def __post_init__(self):
        super().__post_init__()

        self.scene.robot = HANU_A4_IM_CFG.replace(prim_path="{ENV_REGEX_NS}/robot")

        # ==========================================================
        # OBSERVATIONS CONFIGURATION
        # ==========================================================
        # Scale observations for stable learning
        # self.observations.policy.base_lin_vel.scale = 2.0
        self.observations.policy.base_ang_vel.scale = 0.25
        self.observations.policy.joint_pos.scale = 1.0
        self.observations.policy.joint_vel.scale = 0.05

        self.observations.policy.height_scan = None

        # ==========================================================
        # ACTIONS CONFIGURATION
        # ==========================================================
        self.actions.joint_pos.scale = 0.25
        self.actions.joint_pos.clip = {".*": (-100.0, 100.0)}

        # ==========================================================
        # EVENTS / DOMAIN RANDOMIZATION
        # ==========================================================
        self.events.add_base_mass.params["asset_cfg"].body_names = "base_.*"
        self.events.add_base_mass.params["mass_distribution_params"] = (-0.5, 0.15)

        self.events.base_com = None
        self.events.base_external_force_torque = None

        # Randomize joint reset slightly
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

        # ==========================================================
        # COMMANDS CONFIGURATION
        # ==========================================================
        self.commands.base_velocity.ranges.lin_vel_x = (0.0, 0.0)
        self.commands.base_velocity.ranges.lin_vel_y = (0.0, 0.5)  # (-1.0, 0.0)
        self.commands.base_velocity.ranges.ang_vel_z = (0.0, 0.0)

        self.commands.base_velocity.rel_standing_envs = 0.02

        # ==========================================================
        # REWARDS CONFIGURATION
        # ==========================================================
        self.rewards.track_lin_vel_xy_exp.weight = 2.0
        self.rewards.track_ang_vel_z_exp.weight = 0.5
        self.rewards.flat_orientation_l2 = None
        self.rewards.upright_orientation.weight = 2.0
        self.rewards.lin_vel_z_l2.weight = -0.5
        self.rewards.ang_vel_xy_l2.weight = -0.3

        # ------------------------------------------
        # self.rewards.feet_air_time.weight = 0.5
        # self.rewards.feet_air_time.params["threshold"] = 0.4

        # self.rewards.feet_air_time_penalty.weight = -0.01
        # self.rewards.feet_air_time_penalty.params["threshold"] = 0.38

        self.rewards.feet_air_time = None
        self.rewards.feet_air_time_penalty = None
        # ------------------------------------------

        # ------------------------------------------
        # self.rewards.ref_joint_pos = None
        # self.rewards.ref_joint_vel = None
        # self.rewards.foot_pos_tracking = None
        # ------------------------------------------


        self.rewards.feet_lateral_sep_reward = None
        self.rewards.arms_away_from_body = None
        # self.rewards.feet_lateral_sep_reward.weight = 0.15
        # self.rewards.arms_away_from_body.weight = 0.30
        # self.rewards.arms_away_from_body.params["min_lateral_dist"] = 0.20

        # -----------------------------------------
        self.rewards.feet_slide = None
        self.rewards.feet_mirror = None
        self.rewards.feet_step_sequence = None
        # self.rewards.feet_slide.weight = -0.07
        # self.rewards.feet_mirror.weight = -0.02
        # self.rewards.feet_step_sequence.weight = -0.01
        # -----------------------------------------

        self.rewards.action_rate_l2.weight = -0.005
        # self.rewards.dof_acc_l2 = None
        self.rewards.dof_torques_l2.weight = -5.0e-7
        self.rewards.joint_vel_legs.weight = -0.2
        self.rewards.joint_vel_neck.weight = -0.25
        self.rewards.joint_vel_arms = None

        self.rewards.ankle_dof_pos_limits.weight = -0.2
        # self.rewards.knee_pose_deviation.weight = -0.0
        # self.rewards.knee_dof_pos_limits.weight = -0.0
        self.rewards.joint_deviation_arms.weight = -0.2
        self.rewards.joint_deviation_neck.weight = -0.1
        self.rewards.joint_deviation_legs.weight = -0.3

        # --- Termination penalty ---
        self.rewards.termination_penalty.weight = -200.0

        # ==========================================================
        # TERMINATIONS CONFIGURATION
        # ==========================================================
        self.terminations.base_contact.params["sensor_cfg"].body_names = [
            f"^(?!.*{self.foot_link_name}).*"
        ]

class HanuA4RoughEnvCfgV2(HanuA4RoughEnvCfg):

    """
    Configuration for the rough environment in the RAI Hanumanoid project. -- For Play.
    
    """

    def __post_init__(self):
        super().__post_init__()

        self.scene.robot = HANU_A4_TEST_CFG.replace(prim_path="{ENV_REGEX_NS}/robot")

        # ==========================================================
        # OBSERVATIONS CONFIGURATION
        # ==========================================================
        # Scale observations for stable learning
        # self.observations.policy.base_lin_vel.scale = 2.0
        self.observations.policy.base_ang_vel.scale = 0.25
        self.observations.policy.joint_pos.scale = 1.0
        self.observations.policy.joint_vel.scale = 0.05

        self.observations.policy.height_scan = None

        self.observations.policy.gait_phase = None
        self.observations.policy.target_q = None
        self.observations.policy.imu_ang_vel = None
        self.observations.policy.imu_lin_acc = None
        self.observations.critic.imu_ang_vel = None
        self.observations.critic.imu_lin_acc = None


        # ==========================================================
        # ACTIONS CONFIGURATION
        # ==========================================================
        self.actions.joint_pos.scale = 0.25
        self.actions.joint_pos.clip = {".*": (-100.0, 100.0)}

        # ==========================================================
        # EVENTS / DOMAIN RANDOMIZATION
        # ==========================================================
        self.events.add_base_mass.params["asset_cfg"].body_names = "base_.*"
        self.events.add_base_mass.params["mass_distribution_params"] = (-0.5, 0.15)

        self.events.base_com = None
        self.events.base_external_force_torque = None

        # Randomize joint reset slightly
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

        # ==========================================================
        # COMMANDS CONFIGURATION
        # ==========================================================
        self.commands.base_velocity.ranges.lin_vel_x = (0.0, 0.0)
        self.commands.base_velocity.ranges.lin_vel_y = (0.0, 0.5)  # (-1.0, 0.0)
        self.commands.base_velocity.ranges.ang_vel_z = (0.0, 0.0)

        self.commands.base_velocity.rel_standing_envs = 0.02

        # ==========================================================
        # REWARDS CONFIGURATION
        # ==========================================================
        self.rewards.track_lin_vel_xy_exp.weight = 2.0
        self.rewards.track_ang_vel_z_exp.weight = 0.5
        self.rewards.flat_orientation_l2 = None
        self.rewards.upright_orientation.weight = 2.0
        self.rewards.lin_vel_z_l2.weight = -0.5
        self.rewards.ang_vel_xy_l2.weight = -0.3

        # ------------------------------------------
        # self.rewards.feet_air_time.weight = 0.5
        # self.rewards.feet_air_time.params["threshold"] = 0.4

        # self.rewards.feet_air_time_penalty.weight = -0.01
        # self.rewards.feet_air_time_penalty.params["threshold"] = 0.38
        # ------------------------------------------
        self.rewards.feet_air_time = None
        self.rewards.feet_air_time_penalty = None

        self.rewards.feet_lateral_sep_reward = None
        self.rewards.arms_away_from_body = None
        # self.rewards.feet_lateral_sep_reward.weight = 0.15
        # self.rewards.arms_away_from_body.weight = 0.30
        # self.rewards.arms_away_from_body.params["min_lateral_dist"] = 0.20

        self.rewards.feet_slide = None
        self.rewards.feet_mirror = None
        # self.rewards.feet_mirror.weight = -0.02

        self.rewards.action_rate_l2.weight = -0.005
        # self.rewards.dof_acc_l2 = None
        self.rewards.dof_torques_l2.weight = -5.0e-7
        self.rewards.joint_vel_legs.weight = -0.2
        self.rewards.joint_vel_neck.weight = -0.25
        self.rewards.joint_vel_arms = None

        self.rewards.ankle_dof_pos_limits.weight = -0.2
        # self.rewards.knee_pose_deviation.weight = -0.0
        # self.rewards.knee_dof_pos_limits.weight = -0.0
        self.rewards.joint_deviation_arms.weight = -0.2
        self.rewards.joint_deviation_neck.weight = -0.1
        self.rewards.joint_deviation_legs.weight = -0.3

        self.rewards.feet_step_sequence = None

        # --- Termination penalty ---
        self.rewards.termination_penalty.weight = -200.0

        # ==========================================================
        # TERMINATIONS CONFIGURATION
        # ==========================================================
        self.terminations.base_contact.params["sensor_cfg"].body_names = [
            f"^(?!.*{self.foot_link_name}).*"
        ]

@configclass
class HanuA4RoughEnvCfgV3(HanuA4RoughEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        self.scene.robot = HANU_A4_FW_CFG.replace(prim_path="{ENV_REGEX_NS}/robot")

        # ==========================================================
        # OBSERVATIONS CONFIGURATION
        # ==========================================================
        # Scale observations for stable learning
        # self.observations.policy.base_lin_vel.scale = 2.0
        self.observations.policy.base_ang_vel.scale = 0.25
        self.observations.policy.joint_pos.scale = 1.0
        self.observations.policy.joint_vel.scale = 0.05

        self.observations.policy.height_scan = None
        self.observations.policy.gait_phase = None
        self.observations.policy.target_q = None

        # ==========================================================
        # ACTIONS CONFIGURATION
        # ==========================================================
        self.actions.joint_pos.scale = 0.25
        self.actions.joint_pos.clip = {".*": (-100.0, 100.0)}

        # ==========================================================
        # EVENTS / DOMAIN RANDOMIZATION
        # ==========================================================
        self.events.add_base_mass.params["asset_cfg"].body_names = "base_.*"
        self.events.add_base_mass.params["mass_distribution_params"] = (-0.5, 0.15)

        self.events.base_com = None
        self.events.base_external_force_torque = None

        # Randomize joint reset slightly
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

        # ==========================================================
        # COMMANDS CONFIGURATION
        # ==========================================================
        self.commands.base_velocity.ranges.lin_vel_x = (0.0, 0.0)
        self.commands.base_velocity.ranges.lin_vel_y = (0.2, 0.5)  # (-1.0, 0.0)
        self.commands.base_velocity.ranges.ang_vel_z = (0.0, 0.0)

        self.commands.base_velocity.rel_standing_envs = 0.02

        # ==========================================================
        # REWARDS CONFIGURATION
        # ==========================================================
        self.rewards.track_lin_vel_xy_exp.weight = 2.0
        self.rewards.track_ang_vel_z_exp.weight = 0.5
        self.rewards.flat_orientation_l2 = None
        self.rewards.upright_orientation.weight = 2.0
        self.rewards.lin_vel_z_l2.weight = -0.5
        self.rewards.ang_vel_xy_l2.weight = -0.3

        # ------------------------------------------
        self.rewards.feet_air_time.weight = 0.5
        self.rewards.feet_air_time.params["threshold"] = 0.4

        self.rewards.feet_air_time_penalty.weight = -0.01
        self.rewards.feet_air_time_penalty.params["threshold"] = 0.38
        # ------------------------------------------
        # self.rewards.feet_air_time = None
        # self.rewards.feet_air_time_penalty = None

        self.rewards.feet_lateral_sep_reward = None
        self.rewards.arms_away_from_body = None
        # self.rewards.feet_lateral_sep_reward.weight = 0.15
        # self.rewards.arms_away_from_body.weight = 0.30
        # self.rewards.arms_away_from_body.params["min_lateral_dist"] = 0.20

        # self.rewards.feet_slide = None
        self.rewards.feet_slide.weight = -0.07
        self.rewards.feet_mirror = None
        # self.rewards.feet_mirror.weight = -0.02

        self.rewards.action_rate_l2.weight = -0.005
        # self.rewards.dof_acc_l2 = None
        self.rewards.dof_torques_l2.weight = -5.0e-7
        self.rewards.joint_vel_legs.weight = -0.2
        self.rewards.joint_vel_neck.weight = -0.25
        self.rewards.joint_vel_arms.weight = -0.1

        self.rewards.ankle_dof_pos_limits.weight = 0.0
        # self.rewards.knee_pose_deviation.weight = -0.0
        # self.rewards.knee_dof_pos_limits.weight = -0.0
        self.rewards.joint_deviation_arms.weight = -0.2
        self.rewards.joint_deviation_neck.weight = -0.1
        self.rewards.joint_deviation_legs.weight = -0.3

        self.rewards.feet_step_sequence.weight = 0.1

        self.rewards.ref_joint_pos = None
        self.rewards.ref_joint_vel = None
        self.rewards.foot_pos_tracking = None

        # --- Termination penalty ---
        self.rewards.termination_penalty.weight = -200.0

        # ==========================================================
        # TERMINATIONS CONFIGURATION
        # ==========================================================
        self.terminations.base_contact.params["sensor_cfg"].body_names = [
            f"^(?!.*{self.foot_link_name}).*"
        ]


@configclass
class HanuA4RoughEnvCfgV4(HanuA4RoughEnvCfg):
    """Configuration for the rough environment in the RAI Hanumanoid project. -- New baseline"""

    def __post_init__(self):
        super().__post_init__()

        self.scene.robot = HANU_A4_N_BASE_CFG.replace(prim_path="{ENV_REGEX_NS}/robot")
        self.scene.imu_sensor = None

        # ==========================================================
        # OBSERVATIONS CONFIGURATION
        # ==========================================================
        # Scale observations for stable learning
        # self.observations.policy.base_lin_vel.scale = 2.0
        self.observations.policy.base_ang_vel.scale = 0.25
        self.observations.policy.joint_pos.scale = 1.0
        self.observations.policy.joint_vel.scale = 0.05

        self.observations.policy.height_scan = None
        self.observations.policy.gait_phase = None
        self.observations.policy.target_q = None

        self.observations.policy.imu_ang_vel = None
        self.observations.policy.imu_lin_acc = None
        self.observations.critic.imu_ang_vel = None
        self.observations.critic.imu_lin_acc = None

        self.observations.policy.actions.history_length = 0
        self.observations.critic.actions.history_length = 0

        # ==========================================================
        # ACTIONS CONFIGURATION
        # ==========================================================
        self.actions.joint_pos.scale = 0.25
        self.actions.joint_pos.clip = {".*": (-100.0, 100.0)}

        # ==========================================================
        # EVENTS / DOMAIN RANDOMIZATION
        # ==========================================================
        self.events.add_base_mass.params["asset_cfg"].body_names = "base_.*"
        self.events.add_base_mass.params["mass_distribution_params"] = (-0.5, 0.15)

        self.events.base_com = None
        self.events.base_external_force_torque = None

        # Randomize joint reset slightly
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

        # ==========================================================
        # COMMANDS CONFIGURATION
        # ==========================================================
        self.commands.base_velocity.ranges.lin_vel_x = (0.0, 0.0)
        self.commands.base_velocity.ranges.lin_vel_y = (0.0, 0.5)  # (-1.0, 0.0)
        self.commands.base_velocity.ranges.ang_vel_z = (0.0, 0.0)

        self.commands.base_velocity.rel_standing_envs = 0.02

        # ==========================================================
        # REWARDS CONFIGURATION
        # ==========================================================
        self.rewards.track_lin_vel_xy_exp.weight = 2.0
        self.rewards.track_ang_vel_z_exp.weight = 0.5
        self.rewards.flat_orientation_l2 = None
        self.rewards.upright_orientation.weight = 2.0
        self.rewards.lin_vel_z_l2.weight = -0.5
        self.rewards.ang_vel_xy_l2.weight = -0.3

        # ------------------------------------------
        self.rewards.feet_air_time.weight = 0.5
        self.rewards.feet_air_time.params["threshold"] = 0.4

        self.rewards.feet_air_time_penalty.weight = -0.01
        self.rewards.feet_air_time_penalty.params["threshold"] = 0.38
        # ------------------------------------------
        # self.rewards.feet_air_time = None
        # self.rewards.feet_air_time_penalty = None

        self.rewards.feet_lateral_sep_reward = None
        self.rewards.arms_away_from_body = None
        # self.rewards.feet_lateral_sep_reward.weight = 0.15
        # self.rewards.arms_away_from_body.weight = 0.30
        # self.rewards.arms_away_from_body.params["min_lateral_dist"] = 0.20

        # self.rewards.feet_slide = None
        self.rewards.feet_slide.weight = -0.07
        self.rewards.feet_mirror = None
        # self.rewards.feet_mirror.weight = -0.02

        self.rewards.action_rate_l2.weight = -0.005
        # self.rewards.dof_acc_l2 = None
        self.rewards.dof_torques_l2.weight = -5.0e-7
        self.rewards.joint_vel_legs.weight = -0.2
        self.rewards.joint_vel_neck.weight = -0.25
        self.rewards.joint_vel_arms.weight = -0.1

        self.rewards.ankle_dof_pos_limits.weight = 0.0
        # self.rewards.knee_pose_deviation.weight = -0.0
        # self.rewards.knee_dof_pos_limits.weight = -0.0
        self.rewards.joint_deviation_arms.weight = -0.2
        self.rewards.joint_deviation_neck.weight = -0.1
        self.rewards.joint_deviation_legs.weight = -0.3

        self.rewards.feet_step_sequence.weight = 0.1

        self.rewards.ref_joint_pos = None
        self.rewards.ref_joint_vel = None
        self.rewards.foot_pos_tracking = None

        # --- Termination penalty ---
        self.rewards.termination_penalty.weight = -200.0

        # ==========================================================
        # TERMINATIONS CONFIGURATION
        # ==========================================================
        self.terminations.base_contact.params["sensor_cfg"].body_names = [
            f"^(?!.*{self.foot_link_name}).*"
        ]

@configclass
class HanuA4RoughEnvCfgV5(HanuA4RoughEnvCfg):
    """Configuration for the rough environment in the RAI Hanumanoid project. -- target q + Imu"""

    def __post_init__(self):
        super().__post_init__()

        self.scene.robot = HANU_A4_N_BASE_CFG.replace(prim_path="{ENV_REGEX_NS}/robot")
        self.scene.imu_sensor.prim_path = "{ENV_REGEX_NS}/robot/hanu_a4/base_link"

        # ==========================================================
        # OBSERVATIONS CONFIGURATION
        # ==========================================================
        # Scale observations for stable learning
        # self.observations.policy.base_lin_vel.scale = 2.0
        # self.observations.policy.base_ang_vel.scale = 0.25
        self.observations.policy.joint_pos.scale = 1.0
        self.observations.policy.joint_vel.scale = 0.05
        self.observations.policy.imu_lin_acc.scale = 0.25
        self.observations.policy.imu_ang_vel.scale = 0.25

        self.observations.policy.height_scan = None
        # self.observations.policy.gait_phase = None
        # self.observations.policy.target_q = None
        self.observations.policy.base_ang_vel = None
        self.observations.critic.base_ang_vel = None
        self.observations.policy.projected_gravity = None
        self.observations.critic.projected_gravity = None
        # self.observations.policy.projected_gravity.noise = Unoise(n_min=-0.05, n_max=0.05)
        # self.observations.critic.projected_gravity.noise = Unoise(n_min=-0.05, n_max=0.05)

        self.observations.policy.actions.history_length = 1
        self.observations.critic.actions.history_length = 1

        # ==========================================================
        # ACTIONS CONFIGURATION
        # ==========================================================
        self.actions.joint_pos.scale = 0.25
        self.actions.joint_pos.clip = {".*": (-100.0, 100.0)}

        # ==========================================================
        # EVENTS / DOMAIN RANDOMIZATION
        # ==========================================================
        self.events.add_base_mass.params["asset_cfg"].body_names = "base_.*"
        self.events.add_base_mass.params["mass_distribution_params"] = (-0.5, 0.15)

        self.events.base_com = None
        self.events.base_external_force_torque = None

        # Randomize joint reset slightly
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

        # ==========================================================
        # COMMANDS CONFIGURATION
        # ==========================================================
        self.commands.base_velocity.ranges.lin_vel_x = (0.0, 0.0)
        self.commands.base_velocity.ranges.lin_vel_y = (0.0, 0.5)  # (-1.0, 0.0)
        self.commands.base_velocity.ranges.ang_vel_z = (0.0, 0.0)

        self.commands.base_velocity.rel_standing_envs = 0.02

        # ==========================================================
        # REWARDS CONFIGURATION
        # ==========================================================
        self.rewards.track_lin_vel_xy_exp.weight = 2.0
        self.rewards.track_ang_vel_z_exp.weight = 0.5
        self.rewards.flat_orientation_l2 = None
        self.rewards.upright_orientation.weight = 2.0
        self.rewards.lin_vel_z_l2.weight = -0.5
        self.rewards.ang_vel_xy_l2.weight = -0.3

        # ------------------------------------------
        # self.rewards.feet_air_time.weight = 0.5
        # self.rewards.feet_air_time.params["threshold"] = 0.4

        # self.rewards.feet_air_time_penalty.weight = -0.01
        # self.rewards.feet_air_time_penalty.params["threshold"] = 0.38

        # self.rewards.feet_slide.weight = -0.07

        # self.rewards.feet_step_sequence.weight = 0.1
        # ------------------------------------------
        self.rewards.feet_air_time = None
        self.rewards.feet_air_time_penalty = None
        self.rewards.feet_slide = None
        self.rewards.feet_step_sequence = None

        self.rewards.feet_lateral_sep_reward = None
        self.rewards.arms_away_from_body = None
        # self.rewards.feet_lateral_sep_reward.weight = 0.15
        # self.rewards.arms_away_from_body.weight = 0.30
        # self.rewards.arms_away_from_body.params["min_lateral_dist"] = 0.20

        # self.rewards.feet_slide = None

        self.rewards.feet_mirror = None
        # self.rewards.feet_mirror.weight = -0.02

        self.rewards.action_rate_l2.weight = -0.005
        # self.rewards.dof_acc_l2 = None
        self.rewards.dof_torques_l2.weight = -5.0e-7
        self.rewards.joint_vel_legs.weight = -0.2
        self.rewards.joint_vel_neck.weight = -0.25
        self.rewards.joint_vel_arms.weight = -0.1

        self.rewards.ankle_dof_pos_limits.weight = 0.0
        # self.rewards.knee_pose_deviation.weight = -0.0
        # self.rewards.knee_dof_pos_limits.weight = -0.0
        self.rewards.joint_deviation_arms.weight = -0.2
        self.rewards.joint_deviation_neck.weight = -0.1
        self.rewards.joint_deviation_legs.weight = -0.3


        # self.rewards.ref_joint_pos = None
        # self.rewards.ref_joint_vel = None
        # self.rewards.foot_pos_tracking = None

        # --- Termination penalty ---
        self.rewards.termination_penalty.weight = -200.0

        # ==========================================================
        # TERMINATIONS CONFIGURATION
        # ==========================================================
        self.terminations.base_contact.params["sensor_cfg"].body_names = [
            f"^(?!.*{self.foot_link_name}).*"
        ]

class HanuA4RoughEnvCfgV6(HanuA4RoughEnvCfg):

    """
    Configuration for the rough environment in the RAI Hanumanoid project. -- For delay , history length.
    
    """

    def __post_init__(self):
        super().__post_init__()

        self.scene.robot = HANU_A4_EX_CFG.replace(prim_path="{ENV_REGEX_NS}/robot")
        self.scene.imu_sensor = None

        # ==========================================================
        # OBSERVATIONS CONFIGURATION
        # ==========================================================
        # Scale observations for stable learning
        # self.observations.policy.base_lin_vel.scale = 2.0
        self.observations.policy.base_ang_vel.scale = 0.25
        self.observations.policy.joint_pos.scale = 1.0
        self.observations.policy.joint_vel.scale = 0.05

        self.observations.policy.height_scan = None

        self.observations.policy.gait_phase = None
        self.observations.policy.target_q = None
        self.observations.critic.gait_phase = None
        self.observations.critic.target_q = None
        
        self.observations.policy.imu_ang_vel = None
        self.observations.policy.imu_lin_acc = None
        self.observations.critic.imu_ang_vel = None
        self.observations.critic.imu_lin_acc = None


        # ==========================================================
        # ACTIONS CONFIGURATION
        # ==========================================================
        self.actions.joint_pos.scale = 0.25
        self.actions.joint_pos.clip = {".*": (-100.0, 100.0)}

        # ==========================================================
        # EVENTS / DOMAIN RANDOMIZATION
        # ==========================================================
        self.events.add_base_mass.params["asset_cfg"].body_names = "base_.*"
        self.events.add_base_mass.params["mass_distribution_params"] = (-0.5, 0.15)

        self.events.base_com = None
        self.events.base_external_force_torque = None

        # Randomize joint reset slightly
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

        # ==========================================================
        # COMMANDS CONFIGURATION
        # ==========================================================
        self.commands.base_velocity.ranges.lin_vel_x = (0.0, 0.0)
        self.commands.base_velocity.ranges.lin_vel_y = (0.0, 0.5)  # (-1.0, 0.0)
        self.commands.base_velocity.ranges.ang_vel_z = (0.0, 0.0)

        self.commands.base_velocity.rel_standing_envs = 0.02

        # ==========================================================
        # REWARDS CONFIGURATION
        # ==========================================================
        self.rewards.track_lin_vel_xy_exp.weight = 2.0
        self.rewards.track_ang_vel_z_exp.weight = 0.5
        self.rewards.flat_orientation_l2 = None
        self.rewards.upright_orientation.weight = 2.0
        self.rewards.lin_vel_z_l2.weight = -0.5
        self.rewards.ang_vel_xy_l2.weight = -0.3

        # ------------------------------------------
        self.rewards.feet_air_time.weight = 0.5
        self.rewards.feet_air_time.params["threshold"] = 0.4

        self.rewards.feet_air_time_penalty.weight = -0.01
        self.rewards.feet_air_time_penalty.params["threshold"] = 0.38
        # ------------------------------------------
        # self.rewards.feet_air_time = None
        # self.rewards.feet_air_time_penalty = None

        self.rewards.feet_lateral_sep_reward = None
        self.rewards.arms_away_from_body = None
        # self.rewards.feet_lateral_sep_reward.weight = 0.15
        # self.rewards.arms_away_from_body.weight = 0.30
        # self.rewards.arms_away_from_body.params["min_lateral_dist"] = 0.20

        self.rewards.feet_slide.weight = -0.07
        self.rewards.feet_mirror = None
        # self.rewards.feet_mirror.weight = -0.02

        self.rewards.action_rate_l2.weight = -0.005
        # self.rewards.dof_acc_l2 = None
        self.rewards.dof_torques_l2.weight = -5.0e-7
        self.rewards.joint_vel_legs.weight = -0.2
        self.rewards.joint_vel_neck.weight = -0.25
        self.rewards.joint_vel_arms = None

        self.rewards.ankle_dof_pos_limits.weight = -0.2
        # self.rewards.knee_pose_deviation.weight = -0.0
        # self.rewards.knee_dof_pos_limits.weight = -0.0
        self.rewards.joint_deviation_arms.weight = -0.2
        self.rewards.joint_deviation_neck.weight = -0.1
        self.rewards.joint_deviation_legs.weight = -0.3

        self.rewards.feet_step_sequence = None

        self.rewards.ref_joint_pos = None
        self.rewards.ref_joint_vel = None
        self.rewards.foot_pos_tracking = None

        # --- Termination penalty ---
        self.rewards.termination_penalty.weight = -200.0

        # ==========================================================
        # TERMINATIONS CONFIGURATION
        # ==========================================================
        self.terminations.base_contact.params["sensor_cfg"].body_names = [
            f"^(?!.*{self.foot_link_name}).*"
        ]


# @configclass
# class HanuA4ObservationsCfgV7(ObservationsCfg):
#     """ buffer for later phase """
#     gait_phase = ObsTerm(
#         func=mdp.zero_gait_phase,
#     )
#     target_q = ObsTerm(
#         func=mdp.zero_gait_phase,
#     )
    
#     action_pad = ObsTerm(
#         func=mdp.padded_action_history,
#     )
    
    
@configclass
class HanuA4ObservationsCfgV7:
    """Observation specifications for the MDP. add buffer for later phasee"""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group."""

        # observation terms (order preserved)
        # base_lin_vel = ObsTerm(
        #     func=mdp.base_lin_vel,
        #     noise=Unoise(n_min=-0.1, n_max=0.1), 
        #     clip=(-100.00, 100.00),
        #     scale=1.0,
        # )
        base_ang_vel = ObsTerm(
            func=mdp.base_ang_vel,
            noise=Unoise(n_min=-0.2, n_max=0.2),
            clip=(-100.0, 100.0),
            scale=1.0,
        )
        # imu_lin_acc = ObsTerm(
        #     func=mdp.imu_lin_acc,
        #     params={"asset_cfg": SceneEntityCfg("imu_sensor")},
        #     noise=Unoise(n_min=-0.1, n_max=0.1),
        # )
        # imu_ang_vel = ObsTerm(
        #     func=mdp.imu_ang_vel,
        #     params={"asset_cfg": SceneEntityCfg("imu_sensor")},
        #     noise=Unoise(n_min=-0.2, n_max=0.2),
        # )
        projected_gravity = ObsTerm(
            func=mdp.projected_gravity,
            noise=Unoise(n_min=-0.05, n_max=0.05),
            clip=(-100.0, 100.0),
            scale=1.0,
        )
        velocity_commands = ObsTerm(
            func=mdp.generated_commands, 
            params={"command_name": "base_velocity"},
            clip=(-100.0, 100.0),
            scale=1.0,
        )
        joint_pos = ObsTerm(
            func=mdp.joint_pos_rel,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=".*", preserve_order=True)},
            noise=Unoise(n_min=-0.01, n_max=0.01),
            clip=(-100.0, 100.0),
            scale=1.0, 
        )
        joint_vel = ObsTerm(
            func=mdp.joint_vel_rel,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=".*", preserve_order=True)},
            noise=Unoise(n_min=-1.5, n_max=1.5),
            clip=(-100.0, 100.0),
            scale=1.0,
        )
        # actions = ObsTerm(
        #     func=mdp.last_action,
        #     clip=(-100.0, 100.0),
        #     scale=1.0,
        #     history_length=8,
        #     flatten_history_dim=True,
        # )
        action_pad = ObsTerm(
            func=mdp.padded_action_history,
        )

        height_scan = ObsTerm(
            func=mdp.height_scan,
            params={"sensor_cfg": SceneEntityCfg("height_scanner")},
            noise=Unoise(n_min=-0.1, n_max=0.1),
            clip=(-100.0, 100.0),   
        )

        gait_phase = ObsTerm(
            func=mdp.gait_phase_sin_cos,
            params={
                "cycle_time": 1.0,
            }
        )
        target_q = ObsTerm(
            func=mdp.target_joint_positions,
            params={
                "time_offset": 0.0,
            },
        )

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True
    
    @configclass
    class CriticCfg(ObsGroup):
        """Observations for critic group"""

        # observation terms (order preserved)
        base_lin_vel = ObsTerm(
            func=mdp.base_lin_vel, 
            clip=(-100.00, 100.00),
            scale=1.0,
        )
        base_ang_vel = ObsTerm(
            func=mdp.base_ang_vel,
            clip=(-100.0, 100.0),
            scale=1.0,
        )
        # imu_lin_acc = ObsTerm(
        #     func=mdp.imu_lin_acc,
        #     params={"asset_cfg": SceneEntityCfg("imu_sensor")},
        #     noise=Unoise(n_min=-0.1, n_max=0.1),
        # )
        # imu_ang_vel = ObsTerm(
        #     func=mdp.imu_ang_vel,
        #     params={"asset_cfg": SceneEntityCfg("imu_sensor")},
        #     noise=Unoise(n_min=-0.2, n_max=0.2),
        # )
        projected_gravity = ObsTerm(
            func=mdp.projected_gravity,
            clip=(-100.0, 100.0),
            scale=1.0,
        )
        velocity_commands = ObsTerm(
            func=mdp.generated_commands, 
            params={"command_name": "base_velocity"},
            clip=(-100.0, 100.0),
            scale=1.0,
        )
        joint_pos = ObsTerm(
            func=mdp.joint_pos_rel,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=".*", preserve_order=True)},
            clip=(-100.0, 100.0),
            scale=1.0, 
        )
        joint_vel = ObsTerm(
            func=mdp.joint_vel_rel,
           params={"asset_cfg": SceneEntityCfg("robot", joint_names=".*", preserve_order=True)},
            clip=(-100.0, 100.0),
            scale=1.0,
        )
        # actions = ObsTerm(
        #     func=mdp.last_action,
        #     clip=(-100.0, 100.0),
        #     scale=1.0,
        #     history_length=8,
        #     flatten_history_dim=True,
        # )
        action_pad = ObsTerm(
            func=mdp.padded_action_history,
        )
        height_scan = ObsTerm(
            func=mdp.height_scan,
            params={"sensor_cfg": SceneEntityCfg("height_scanner")},
            clip=(-100.0, 100.0),
        )

        gait_phase = ObsTerm(
            func=mdp.gait_phase_sin_cos,
            params={
                "cycle_time": 1.0,
            }
        )
        target_q = ObsTerm(
            func=mdp.target_joint_positions,
            params={
                "time_offset": 0.0,
            },
        )

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    # observation groups
    policy: PolicyCfg = PolicyCfg()
    critic: CriticCfg = CriticCfg()
         



@configclass
class HanuA4RoughEnvCfgV7(HanuA4RoughEnvCfg):
    """Configuration for the rough environment in the RAI Hanumanoid project. -- New baseline with buffers"""

    observations: HanuA4ObservationsCfgV7 = HanuA4ObservationsCfgV7()

    def __post_init__(self):
        super().__post_init__()

        self.scene.robot = HANU_A4_N_BASE_CFG.replace(prim_path="{ENV_REGEX_NS}/robot")
        self.scene.imu_sensor = None

        # ==========================================================
        # OBSERVATIONS CONFIGURATION
        # ==========================================================
        # Scale observations for stable learning
        # self.observations.policy.base_lin_vel.scale = 2.0
        self.observations.policy.base_ang_vel.scale = 0.25
        self.observations.policy.joint_pos.scale = 1.0
        self.observations.policy.joint_vel.scale = 0.05

        # self.observations.policy.height_scan = None
        # self.observations.policy.gait_phase = None
        # self.observations.policy.target_q = None

        # self.observations.policy.imu_ang_vel = None
        # self.observations.policy.imu_lin_acc = None
        # self.observations.critic.imu_ang_vel = None
        # self.observations.critic.imu_lin_acc = None

        # self.observations.policy.actions.history_length = 0
        # self.observations.critic.actions.history_length = 0

        # self.observations.policy.actions = None
        # self.observations.critic.actions = None

        # ==========================================================
        # ACTIONS CONFIGURATION
        # ==========================================================
        self.actions.joint_pos.scale = 0.25
        self.actions.joint_pos.clip = {".*": (-100.0, 100.0)}

        # ==========================================================
        # EVENTS / DOMAIN RANDOMIZATION
        # ==========================================================
        self.events.add_base_mass.params["asset_cfg"].body_names = "base_.*"
        self.events.add_base_mass.params["mass_distribution_params"] = (-0.5, 0.15)

        self.events.base_com = None
        self.events.base_external_force_torque = None

        # Randomize joint reset slightly
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

        # ==========================================================
        # COMMANDS CONFIGURATION
        # ==========================================================
        self.commands.base_velocity.ranges.lin_vel_x = (0.0, 0.0)
        self.commands.base_velocity.ranges.lin_vel_y = (0.0, 0.5)  # (-1.0, 0.0)
        self.commands.base_velocity.ranges.ang_vel_z = (0.0, 0.0)

        self.commands.base_velocity.rel_standing_envs = 0.02

        # ==========================================================
        # REWARDS CONFIGURATION
        # ==========================================================
        self.rewards.track_lin_vel_xy_exp.weight = 2.0
        self.rewards.track_ang_vel_z_exp.weight = 0.5
        self.rewards.flat_orientation_l2 = None
        self.rewards.upright_orientation.weight = 2.0
        self.rewards.lin_vel_z_l2.weight = -0.5
        self.rewards.ang_vel_xy_l2.weight = -0.3

        # ------------------------------------------
        self.rewards.feet_air_time.weight = 0.5
        self.rewards.feet_air_time.params["threshold"] = 0.4

        self.rewards.feet_air_time_penalty.weight = -0.01
        self.rewards.feet_air_time_penalty.params["threshold"] = 0.38
        # ------------------------------------------
        # self.rewards.feet_air_time = None
        # self.rewards.feet_air_time_penalty = None

        self.rewards.feet_lateral_sep_reward = None
        self.rewards.arms_away_from_body = None
        # self.rewards.feet_lateral_sep_reward.weight = 0.15
        # self.rewards.arms_away_from_body.weight = 0.30
        # self.rewards.arms_away_from_body.params["min_lateral_dist"] = 0.20

        # self.rewards.feet_slide = None
        self.rewards.feet_slide.weight = -0.07
        self.rewards.feet_mirror = None
        # self.rewards.feet_mirror.weight = -0.02

        self.rewards.action_rate_l2.weight = -0.005
        # self.rewards.dof_acc_l2 = None
        self.rewards.dof_torques_l2.weight = -5.0e-7
        self.rewards.joint_vel_legs.weight = -0.2
        self.rewards.joint_vel_neck.weight = -0.25
        self.rewards.joint_vel_arms.weight = -0.1

        self.rewards.ankle_dof_pos_limits.weight = 0.0
        # self.rewards.knee_pose_deviation.weight = -0.0
        # self.rewards.knee_dof_pos_limits.weight = -0.0
        self.rewards.joint_deviation_arms.weight = -0.2
        self.rewards.joint_deviation_neck.weight = -0.1
        self.rewards.joint_deviation_legs.weight = -0.3

        self.rewards.feet_step_sequence.weight = 0.1

        self.rewards.ref_joint_pos = None
        self.rewards.ref_joint_vel = None
        self.rewards.foot_pos_tracking = None

        # --- Termination penalty ---
        self.rewards.termination_penalty.weight = -200.0

        # ==========================================================
        # TERMINATIONS CONFIGURATION
        # ==========================================================
        self.terminations.base_contact.params["sensor_cfg"].body_names = [
            f"^(?!.*{self.foot_link_name}).*"
        ]