"""Test script: spawn hanu_a4 mid-air (~0.5 m) and command all joints to default position (qtarget=0).

Usage:
    ./isaaclab.sh -p scripts/pd_hanu_a4.py
"""

"""Launch Isaac Sim Simulator first."""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Spawn hanu_a4 mid-air and hold qtarget=0.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch
import matplotlib.pyplot as plt

import isaacsim.core.utils.prims as prim_utils

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation
from isaaclab.sim import SimulationContext

from hanu_lab.assets import HANU_A4_CFG


def design_scene() -> tuple[dict, list[list[float]]]:
    """Design the scene: ground plane, light, and one hanu_a4 robot."""
    # Ground plane
    cfg = sim_utils.GroundPlaneCfg(
        physics_material=sim_utils.RigidBodyMaterialCfg(
            static_friction=3.0,
            dynamic_friction=3.0,
            restitution=0.0,
        )
    )
    cfg.func("/World/defaultGroundPlane", cfg)

    # Dome light
    cfg = sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    cfg.func("/World/Light", cfg)

    # Single origin
    origins = [[0.0, 0.0, 0.0]]
    prim_utils.create_prim("/World/Origin1", "Xform", translation=origins[0])

    # Articulation – spawn the robot inside Origin1
    hanu_cfg = HANU_A4_CFG.copy()
    hanu_cfg.prim_path = "/World/Origin.*/hanu_a4"

    # Override the spawn height so the robot starts 0.5 m above the ground
    hanu_cfg.init_state.pos = (0.0, 0.0, 1.3)
    hanu_cfg.init_state.joint_pos = {
        # bend legs
        ".*_hip_pitch": -0.14, 
        ".*_knee_pitch": 0.17, 
        ".*_ankle_pitch": 0.08,
        # wider legs
        ".*_l_hip_roll": 0.02,
        ".*_r_hip_roll": 0.02,
        ".*_l_ankle_roll": -0.02,
        ".*_r_ankle_roll": 0.02,
        ".*_l_hip_yaw": -0.08,
        ".*_r_hip_yaw": -0.08, 
        # bend arms
        ".*_shoulder_pitch": -0.06, # -> degree 14.32394
        # ".*_elbow_pitch": -0.9, # -> degree -51.5669
        # wider arms
        ".*_shoulder_roll": 0.25, # -> degree 17.18872
    }
    # hanu_cfg.init_state.joint_vel = {".*": 0.0}

    robot = Articulation(cfg=hanu_cfg)
    return {"hanu": robot}, origins


# def run_simulator(
#     sim: SimulationContext,
#     entities: dict[str, Articulation],
#     origins: torch.Tensor,
# ):
#     """Simulation loop: reset to mid-air pose and hold qtarget=0 every 500 steps."""
#     robot = entities["hanu"]
#     sim_dt = sim.get_physics_dt()
#     count = 0

#     print("Default Joint Positions being commanded:")
#     print(robot.data.default_joint_pos)

#     while simulation_app.is_running():
#         # ── Reset ──────────────────────────────────────────────────────────
#         if count % 500 == 0:
#             count = 0

#             # Root pose: use the default root state but override the position
#             root_state = robot.data.default_root_state.clone()
#             root_state[:, :3] += origins  # add origin offset (keeps orientation)
#             robot.write_root_pose_to_sim(root_state[:, :7])
#             robot.write_root_velocity_to_sim(root_state[:, 7:])

#             joint_pos = robot.data.default_joint_pos.clone()
#             joint_vel = torch.zeros_like(robot.data.default_joint_vel)
#             robot.write_joint_state_to_sim(joint_pos, joint_vel)

#             robot.reset()
#             print("[INFO]: Resetting robot to mid-air, qtarget=0 …")

#         # ── Command: hold qtarget = 0 ──────────────────────────────────────
#         # Use PD position targets to hold all joints at 0
#         target_pos = robot.data.default_joint_pos.clone()
#         robot.set_joint_position_target(target_pos)

#         # Write commands and step
#         robot.write_data_to_sim()
#         sim.step()
#         count += 1
#         robot.update(sim_dt)


def run_simulator(
    sim: SimulationContext,
    entities: dict[str, Articulation],
    origins: torch.Tensor,
):
    """Simulation loop: Drop the robot and graph the knee joint response."""
    robot = entities["hanu"]
    sim_dt = sim.get_physics_dt()
    count = 0

    # ── 1. Setup Data Logging ──────────────────────────────────────────
    # Find the exact index for one of the knee joints to track
    knee_indices, knee_names = robot.find_joints(".*_knee_pitch")
    track_idx = knee_indices[0]  # Just track the first knee it finds (e.g., left knee)
    
    # Get the target angle we are expecting the knee to hold
    target_knee_angle = robot.data.default_joint_pos[0, track_idx].item()

    time_log = []
    position_log = []
    
    # Record for the first 2.0 seconds
    record_time = 2.0 
    
    print(f"[INFO]: Dropping robot. Tracking joint: {knee_names[0]}")
    print(f"[INFO]: Target holding position: {target_knee_angle:.4f} rad")

    # ── 2. The Physics Loop ────────────────────────────────────────────
    while simulation_app.is_running():
        current_time = count * sim_dt
        
        # Stop recording and show the plot after 'record_time' seconds
        if current_time >= record_time:
            print("[INFO]: Test complete. Rendering graph...")
            break 

        # ── Command: hold qtarget = default pose ──────────
        target_pos = robot.data.default_joint_pos.clone()
        robot.set_joint_position_target(target_pos)

        # ── The Invisible Tether: Keep Torso Upright ───────────────
        root_state = robot.data.root_state_w.clone()
        # Force the quaternion orientation to remain perfectly upright [w, x, y, z]
        root_state[:, 3:7] = torch.tensor([1.0, 0.0, 0.0, 0.0], device=sim.device)
        root_state[:, 10:13] = 0.0 # Kill angular velocity
        root_state[:, :2] = origins[0, :2] 
        root_state[:, 7:9] = 0.0 # Kill X and Y linear velocity
        
        # Write the stabilized root state back to the simulator
        robot.write_root_pose_to_sim(root_state[:, :7])
        robot.write_root_velocity_to_sim(root_state[:, 7:])
        # ───────────────────────────────────────────────────────────

        # Write commands and step
        robot.write_data_to_sim()
        sim.step()
        
        # ── 3. Record the Data ─────────────────────────────────────────
        # Append the current time and the actual physical angle of the knee
        time_log.append(current_time)
        actual_knee_angle = robot.data.joint_pos[0, track_idx].item()
        position_log.append(actual_knee_angle)

        count += 1
        robot.update(sim_dt)

    # ── 4. Generate the Plot ───────────────────────────────────────────
    plt.figure(figsize=(10, 6))
    
    # Plot the actual movement of the robot's knee
    plt.plot(time_log, position_log, label="Actual Knee Position", color='#1f77b4', linewidth=2)
    
    # Plot a straight dashed line showing where the knee SHOULD be
    plt.axhline(y=target_knee_angle, color='#d62728', linestyle='--', linewidth=2, label="Target Position (Default Pose)")
    
    plt.title(f"Drop Test Response: {knee_names[0]}", fontsize=14, fontweight='bold')
    plt.xlabel("Time (seconds)", fontsize=12)
    plt.ylabel("Joint Angle (radians)", fontsize=12)
    plt.grid(True, linestyle=':', alpha=0.7)
    plt.legend(loc="best", fontsize=11)
    
    # Add a text box with instructions
    plt.text(0.5, 0.05, "Tuning Guide:\n"
                       "- Sags heavily below target: Increase Kp (Stiffness)\n"
                       "- Wavy/Bouncing: Increase Kd (Damping)\n"
                       "- Takes too long to reach target line: Decrease Kd (Damping)", 
             transform=plt.gca().transAxes, fontsize=10, 
             bbox=dict(facecolor='white', alpha=0.8, edgecolor='gray'))
    
    plt.tight_layout()
    plt.show()  # This pauses the script and opens the UI window

def main():
    sim_cfg = sim_utils.SimulationCfg(
        device=args_cli.device, 
        dt=0.005,
        physx=sim_utils.PhysxCfg(
            bounce_threshold_velocity=0.2,
        )
    )
    sim = SimulationContext(sim_cfg)
    sim.set_camera_view([2.5, 0.0, 2.5], [0.0, 0.0, 1.0])

    scene_entities, scene_origins = design_scene()
    scene_origins = torch.tensor(scene_origins, device=sim.device)

    sim.reset()
    print("[INFO]: Setup complete. Starting simulation …")
    run_simulator(sim, scene_entities, scene_origins)


if __name__ == "__main__":
    main()
    simulation_app.close()
