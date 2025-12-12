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
from hanu_lab.assets import HANU_A1_QUAD_CFG


from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass

import hanu_lab.tasks.manager_based.locomotion.velocity.mdp as mdp
from hanu_lab.tasks.manager_based.locomotion.velocity.velocity_env_cfg import LocomotionVelocityRoughEnvCfg, RewardsCfg, TerminationsCfg, CommandsCfg, EventCfg


@configclass
class HanuA1QuadRewardCfg(RewardsCfg):
    """Reward terms for the MDP."""
    termination_penalty = RewTerm(func=mdp.is_terminated, weight=-200.0)
    feet_air_time = RewTerm(
        func=mdp.feet_air_time,
        weight=0.01,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=["LL6_Foot_Roll_1","RL6_Foot_Roll_1","LA7_Hand_Roll_1","RA7_Hand_Roll_1"]),
            "command_name": "base_velocity",
            "threshold": 0.3,
        },
    )
    flat_orientation_l2 = RewTerm(
        func=mdp.flat_orientation_l2,
        weight=-0.1,
    )


@configclass
class HanuA1QuadRoughEnvCfg(LocomotionVelocityRoughEnvCfg):
    """Environment configuration for the Hanumanoid A1 performing quadrupedal locomotion in a rough terrain."""
    
    rewards: HanuA1QuadRewardCfg = HanuA1QuadRewardCfg()

    foot_link_name = [
        "LL6_Foot_Roll_1",
        "RL6_Foot_Roll_1"
        "LA7_Hand_Roll_1",
        "RA7_Hand_Roll_1",
    ]

    def __post_init__(self):
        super().__post_init__()
    
        # ------ Scene configuration --------
        self.scene.robot = HANU_A1_QUAD_CFG.replace(prim_path="{ENV_REGEX_NS}/robot")
        self.scene.height_scanner.prim_path = "/World/envs/env_.*/robot/full_body/base_link"
        self.scene.contact_forces.prim_path = "{ENV_REGEX_NS}/robot/full_body/.*"

        self.scene.terrain.terrain_generator.sub_terrains["boxes"].grid_height_range = (0.025, 0.1)
        self.scene.terrain.terrain_generator.sub_terrains["random_rough"].noise_range = (0.01, 0.06)
        self.scene.terrain.terrain_generator.sub_terrains["random_rough"].noise_step = 0.01

        # reduce action scale
        self.actions.joint_pos.scale = 0.25

        # ------ Events configuration --------
        self.events.push_robot = None
        self.events.add_base_mass = None
        self.events.base_external_force_torque = None
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
        self.events.base_com = None

        # ------ Rewards configuration --------
        self.rewards.feet_air_time.weight = 0.01
        self.rewards.undesired_contacts.weight = -1.0
        self.rewards.undesired_contacts.params["sensor_cfg"].body_names = [
            "LL4_Calf_Pitch_1",
            "RL4_Calf_Pitch_1",
            "LA6_Wrist_Pitch_1",
            "RA6_Wrist_Pitch_1",
        ]
        self.rewards.dof_torques_l2.weight = -2e-7
        self.rewards.track_lin_vel_xy_exp.weight = 1.5
        self.rewards.track_ang_vel_z_exp.weight = 0.75
        self.rewards.dof_acc_l2.weight = -2.5e-8
        self.rewards.action_rate_l2.weight = -1.0e-4

        # ------ Commands configuration --------
        self.commands.base_velocity.ranges.lin_vel_x = (-0.0, 0.0)
        self.commands.base_velocity.ranges.lin_vel_y = (-1.0, 0.0) # (-1.0, 0.0)
        self.commands.base_velocity.ranges.ang_vel_z = (-0.0, 0.0)


        # ------ Observations configuration --------
        self.observations.policy.enable_corruption = False

        # ------ Terminations configuration --------
        # self.terminations.base_contact.params["sensor_cfg"].body_names = ["Hip_1","Torso_1"]
        self.terminations.base_contact.params["sensor_cfg"].body_names = [
            "base_link",
            "Neck_1",
            "LA1_Shoulder_Pitch_1",
            "RA1_Shoulder_Pitch_1",
            "LA2_Shoulder_Roll_1",
            "RA2_Shoulder_Roll_1",
            "LA3_Upper_Arm_Yaw_1",
            "RA3_Upper_Arm_Yaw_1",
            "LA4_Lower_Arm_Pitch_1",
            "RA4_Lower_Arm_Pitch_1",
            "Torso_1",
            "abdomen_1",
            "Hip_1",
            "LL1_Groin_Yaw_1",
            "RL1_Groin_Yaw_1",
            "LL2_Buttock_Pitch_1",
            "RL2_Buttock_Pitch_1",
            "LL3_Thigh_Roll_1",
            "RL3_Thigh_Roll_1",
        ]