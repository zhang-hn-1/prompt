# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for a simple Cartpole robot."""

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg
from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR

##
# Configuration
##

ROTUNBOT_WHEEL = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"./rotunbot_wheel_sim_model_0.4/Rotunbot_wheel.usd",
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.4), 
        lin_vel=(0.0, 0.0, 0.0),
        joint_pos={"joint1": 0.0, "joint2": 0.0, "joint3": 0.0, }
    ),
    actuators={
        "joint1_actuator": ImplicitActuatorCfg(
            joint_names_expr=["joint1"],
            effort_limit_sim=160.0,
            velocity_limit_sim=5*360/(180/3.14), # rad
            stiffness=0.0,
            damping=0.0,
            friction=0.01,
        ),
        "joint2_actuator": ImplicitActuatorCfg(
            joint_names_expr=["joint2"], 
            effort_limit_sim=200.0, 
            velocity_limit_sim=33.01/(180/3.14),
            stiffness=0.0, 
            damping=0.0,
            friction=0.01,
        ),
        "joint3_actuator": ImplicitActuatorCfg(
            joint_names_expr=["joint3"], 
            effort_limit_sim=18.0, 
            velocity_limit_sim=229/(180/3.14),
            stiffness=0.0, 
            damping=5.0,
            friction=0.01,
        ),
    },
)
"""Configuration for a simple Cartpole robot."""


print('-')
