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
from hanu_lab.assets import HANU_A1_CFG


from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass

import hanu_lab.tasks.manager_based.locomotion.velocity.mdp as mdp
from hanu_lab.tasks.manager_based.locomotion.velocity.velocity_env_cfg import LocomotionVelocityRoughEnvCfg, RewardsCfg, TerminationsCfg, CommandsCfg, EventCfg



@configclass
class HanuA1RewardsCfg(RewardsCfg):
    termination_penalty = RewTerm(func=mdp.is_terminated, weight=-200.0)
    feet_air_time = RewTerm(
        func=mdp.feet_air_time_positive_biped,
        weight=0.25,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=["RL6_Foot_Roll_1", "LL6_Foot_Roll_1"]),
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
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=["RL6_Foot_Roll_1", "LL6_Foot_Roll_1"]),
            "asset_cfg": SceneEntityCfg("robot", body_names=["RL6_Foot_Roll_1", "LL6_Foot_Roll_1"]),
        },
    )
    #* cancel base_height out according to g1 reward cfg
    # base_height_l2 = RewTerm(
    #     func=mdp.base_height_l2,
    #     weight=0.9,
    #     params={
    #         "asset_cfg": SceneEntityCfg("robot"),
    #         "target_height": 1.7, # robot height: 1.66m (1.66/2 + fall height) 
    #     },  # "target": 0.35         target not a param of base_pos_z
    # )

    upright_orientation = RewTerm(
        func=mdp.upright_orientation_l2,
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
                joint_names=["LL4_Calf_to_LL5_ankle_Pitch",
                             "RL4_Calf_to_RL5_ankle_Pitch",
                             "LL5_ankle_to_LL6_Foot_Roll",
                             "RL5_ankle_to_RL6_Foot_Roll",
                ]
            ),
        },
    )
    # ----- joint deviation penalty
    joint_deviation_neck = RewTerm(
        func=mdp.joint_deviation_l1,
        weight=-0.2,
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=["neck_to_torso_Pitch"])
        },
    )
    joint_deviation_arms = RewTerm(
        func=mdp.joint_deviation_l1,
        weight=-0.1,
        params={
            "asset_cfg": SceneEntityCfg(
                "robot",
                joint_names=[
                    "LA1_Shoulder_to_LA2_Shoulder_Roll",
                    "LA2_Shoulder_to_LA3_Upper_Arm_Yaw",
                    "LA3_Upper_Arm_to_LA4_Lower_Arm_Pitch",
                    "LA4_Lower_Arm_to_LA5_Wrist_Yaw",
                    "LA5_Wrist_to_LA6_Wrist_Pitch",
                    "LA6_Wrist_to_LA7_Hand_Roll",
                    "RA1_Shoulder_to_RA2_Shoulder_Roll_",
                    "RA2_Shoulder_to_RA3_Upper_Arm_Yaw",
                    "RA3_Upper_Arm_to_RA4_Lower_Arm_Pitch_",
                    "RA4_Lower_Arm_to_RA5_Wrist_Yaw",
                    "RA5_Wrist_to_RA6_Wrist_Pitch",
                    "RA6_Wrist_to_RA7_Hand_Roll",
                ],
            )
        },
    )
    joint_deviation_torso = RewTerm(
        func=mdp.joint_deviation_l1,
        weight=-0.1,
        params={
            "asset_cfg": SceneEntityCfg(
                "robot", 
                joint_names=[
                    "abdomen_to_hip_Pitch",
                    "base_to_neck_yaw",
                    "neck_to_torso_Pitch",
                    "torso_to_LA1_Shoulder_Pitch",
                    "torso_to_RA1_Shoulder_Pitch",
                    "torso_to_abdomen_Yaw",
                ]
            )
        },
    )
    

@configclass
class HanuA1TerminationsCfg(TerminationsCfg):
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
class HanuA1IdleCommandsCfg(CommandsCfg):
    """Idle commands configuration for Hanumanoid A1."""
    null = mdp.NullCommandCfg()

@configclass
class HanuA1EventsCfg(EventCfg):
    """Events configuration for Hanumanoid A1."""
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
class HanuA1IdleEventsCfg(EventCfg):
    """Idle events configuration for Hanumanoid A1."""
    # ------- Reset events --------
    reset_scene = EventTerm(
        func=mdp.reset_scene_to_default,
        mode="reset",
    )


@configclass
class HanuA1RoughEnvCfg(LocomotionVelocityRoughEnvCfg):
    """Environment configuration for Hanumanoid A1 in rough terrain."""

    rewards: HanuA1RewardsCfg = HanuA1RewardsCfg()
    terminations: HanuA1TerminationsCfg = HanuA1TerminationsCfg()
    # commands: HanuA1IdleCommandsCfg = HanuA1IdleCommandsCfg()
    # events: HanuA1IdleEventsCfg = HanuA1IdleEventsCfg()

    foot_link_name = ".*_Foot_.*"

    def __post_init__(self):
        super().__post_init__()

        # ------ Scene configuration --------
        self.scene.robot = HANU_A1_CFG.replace(prim_path="{ENV_REGEX_NS}/robot")
        # self.scene.robot.spawn.activate_contact_sensors = True
        self.scene.height_scanner.prim_path = "/World/envs/env_.*/robot/full_body/Torso_1"
        # /World/Origin1/full_body/full_body/RL6_Foot_Roll_1
        self.scene.contact_forces.prim_path = "{ENV_REGEX_NS}/robot/full_body/.*"
        # self.scene.contact_forces.prim_paths = [
        #     "{ENV_REGEX_NS}/robot/full_body/LL6_Foot_Roll_1",
        #     "{ENV_REGEX_NS}/robot/full_body/RL6_Foot_Roll_1"
        # ]

        # check if the robot is spawned correctly
        # stage = omni.usd.get_context().get_stage()
        # world_prim = stage.GetPrimAtPath("/World")
        # print("All Prims in /World:")
        # for prim in world_prim.GetChildren():
        #     print(prim.GetPath().pathString)

        self.scene.terrain.terrain_generator.sub_terrains["boxes"].grid_height_range = (0.025, 0.1)
        self.scene.terrain.terrain_generator.sub_terrains["random_rough"].noise_range = (0.01, 0.06)
        self.scene.terrain.terrain_generator.sub_terrains["random_rough"].noise_step = 0.01

        # ------ Events configuration --------
        self.events.push_robot = None
        self.events.add_base_mass = None
        # self.events.add_base_mass.params["mass_distribution_params"] = (-1.0, 3.0)
        self.events.base_external_force_torque = None
        # self.events.base_external_force_torque.params["asset_cfg"].body_names = "Hip_1"
        # self.events.base_external_force_torque.params["force_range"] = (-1.0, 1.0)
        
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
        self.events.base_com = None

        # ------ Rewards configuration --------
        # penalties
        self.rewards.track_lin_vel_xy_exp.weight = 1.5
        self.rewards.track_ang_vel_z_exp.weight = 1.0
        self.rewards.lin_vel_z_l2.weight = -0.2
        self.rewards.undesired_contacts.params["sensor_cfg"].body_names = [
            "LL1_Groin_Yaw_1",
            "LL2_Buttock_Pitch_1",
            "RL1_Groin_Yaw_1",
            "RL2_Buttock_Pitch_1"
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
                "torso_to_abdomen_Yaw",
                "LL1_Groin_to_LL2_Buttock_Pitch",
                "RL1_Groin_to_RL2_Buttock_Pitch",
                "LL3_Thigh_to_LL4_Calf_Pitch",
                "RL3_Thigh_to_RL4_Calf_Pitch",
            ]
        )
        self.rewards.dof_torques_l2.weight = -2.0e-6 # g1_flat
        self.rewards.dof_torques_l2.params["asset_cfg"] = SceneEntityCfg(
            "robot", 
            joint_names=[
                "torso_to_abdomen_Yaw",
                "LL1_Groin_to_LL2_Buttock_Pitch",
                "RL1_Groin_to_RL2_Buttock_Pitch",
                "LL3_Thigh_to_LL4_Calf_Pitch",
                "RL3_Thigh_to_RL4_Calf_Pitch",
                # "LL4_Calf_to_LL5_ankle_Pitch",
                # "RL4_Calf_to_RL5_ankle_Pitch",
            ]
        )
        # self.rewards.termination_penalty.weight = -2.0
        

        # ------ Commands configuration --------
        self.commands.base_velocity.ranges.lin_vel_x = (-0.0, 0.0)
        self.commands.base_velocity.ranges.lin_vel_y = (-1.5, 0.0) # (-1.0, 0.0)
        self.commands.base_velocity.ranges.ang_vel_z = (-0.0, 0.0)
        # self.commands.base_velocity.rel_standing_envs = 0.5

        # ------ Observations configuration --------
        self.observations.policy.enable_corruption = False

        # ------ Terminations configuration --------
        # self.terminations.base_contact.params["sensor_cfg"].body_names = ["Hip_1","Torso_1"]
        self.terminations.base_contact.params["sensor_cfg"].body_names = [f"^(?!.*{self.foot_link_name}).*"]

        # self.terminations.base_contact.params["sensor_cfg"].body_names = [
        #     "base_link",
        #     "Neck_1",
        #     "LA1_Shoulder_Pitch_1",
        #     "RA1_Shoulder_Pitch_1",
        #     "LA2_Shoulder_Roll_1",
        #     "RA2_Shoulder_Roll_1",
        #     "LA3_Upper_Arm_Yaw_1",
        #     "RA3_Upper_Arm_Yaw_1",
        #     "LA4_Lower_Arm_Pitch_1",
        #     "RA4_Lower_Arm_Pitch_1",
        #     "LA5_Wrist_Yaw_1",
        #     "RA5_Wrist_Yaw_1",
        #     "LA6_Wrist_Pitch_1",
        #     "RA6_Wrist_Pitch_1",
        #     "LA7_Hand_Roll_1",
        #     "RA7_Hand_Roll_1",
        #     "Torso_1",
        #     "abdomen_1",
        #     "Hip_1",
        #     "LL1_Groin_Yaw_1",
        #     "RL1_Groin_Yaw_1",
        #     "LL2_Buttock_Pitch_1",
        #     "RL2_Buttock_Pitch_1",
        #     "LL3_Thigh_Roll_1",
        #     "RL3_Thigh_Roll_1",
        #     "LL4_Calf_Pitch_1",
        #     "RL4_Calf_Pitch_1",
        # ]
        