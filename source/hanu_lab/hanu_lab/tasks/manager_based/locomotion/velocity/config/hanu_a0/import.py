# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import argparse

from isaaclab.app import AppLauncher

# create argparser
parser = argparse.ArgumentParser(description="Tutorial on creating an empty stage.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()
# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# ==========================================================================================

from hanu_lab.assets import HANU_A0_CFG

from isaaclab.sim import SimulationCfg, SimulationContext

# ==========================================================================================

# Initialize the simulation context
sim_cfg = SimulationCfg(dt=0.01)
sim = SimulationContext(sim_cfg)
# Set main camera
sim.set_camera_view([2.5, 2.5, 2.5], [0.0, 0.0, 0.0])

# Play the simulator
sim.reset()
# Now we are ready!
print("[INFO]: Setup complete...")

# Simulate physics
while simulation_app.is_running():
    # perform step
    sim.step()

# close sim app
simulation_app.close()
