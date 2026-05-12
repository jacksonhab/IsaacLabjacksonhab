# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab.utils import configclass

from .rough_env_cfg import HexapodRoughEnvCfg
#import os
import math

@configclass
class HexapodFlatEnvCfg(HexapodRoughEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        
        #os.environ["WANDB_DISABLE_SYMLINKS"] = "true"
        #self.actions.joint_pos.scale = 0.75
        # override body values
        #self.events.add_base_mass.params["asset_cfg"].body_names="CenterLink"
        self.events.add_base_mass.params["asset_cfg"].body_names=["CenterLink", "BackLink", "FrontLink"]
        self.events.add_base_mass.params["mass_distribution_params"] = (0.0, 0.0) #just added this to see if (-5.0, 5.0) is the problem
        self.events.base_com.params["asset_cfg"].body_names="CenterLink"
        #self.events.base_com.params["asset_cfg"].body_names=["CenterLink", "BackLink", "FrontLink"]
        self.events.base_com.params["com_range"]: {
            "x": (0.03,0.03),
            "y": (0.0,0.0),
            "z": (0.0,0.0),
        }
        self.events.physics_material.params["static_friction_range"]: (0.8, 0.8)
        self.events.physics_material.params["dynamic_friction_range"]: (0.5, 0.5)
        #self.scene.terrain.physics_material.static_friction= 0.8
        #self.scene.terrain.physics_material.dynamic_friction= 0.6

        self.events.base_external_force_torque.params["asset_cfg"].body_names = "CenterLink"

        self.rewards.feet_air_time.params["sensor_cfg"].body_names=["MiddleLeft","MiddleRight","BackLeft","BackRight","FrontLeft","FrontRight"]

        self.rewards.undesired_contacts.params["sensor_cfg"].body_names=["CenterLink", "BackLink", "FrontLink"]

        self.terminations.base_contact.params["sensor_cfg"].body_names="CenterLink"

        #reset position:
        self.events.reset_base.params = {
            "pose_range": {
                "x":   (0.0, 0.0),
                "y":   (0.0, 0.0),
                "yaw": (0.0,  0.0),   # ~±6°
                # include "z" here if your event supports it; e.g., ("z": (0.18, 0.22))
            },
            "velocity_range": {
                "x":    (-0.01, 0.01),
                "y":    (0.0, 0.0),
                "z":    (0.0, 0.0),
                "roll": (0.0,  0.0),
                "pitch":(0.0,  0.0),
                "yaw":  (00.0,  0.0),
            },
        }

        # override velocity ranges -> based on actual velocities
        # max velocity of the physics gaits is 0.14 m/s - so these values should be in that range
        self.commands.base_velocity.ranges.lin_vel_y=(0.07,0.07)
        self.commands.base_velocity.ranges.lin_vel_x=(0.0,0.0) # 0.16 m/s for six legged gaits
        self.commands.base_velocity.ranges.ang_vel_z=(0.0,0.0)
        self.commands.base_velocity.debug_vis = False

        # override rewards
        self.rewards.track_lin_vel_xy_exp.weight = 1.0
        self.rewards.track_lin_vel_xy_exp.params["std"] = math.sqrt(0.25)*0.2

        self.rewards.track_ang_vel_z_exp.weight = 0.4 #trying to keep it a little more straight
        self.rewards.track_ang_vel_z_exp.params["std"] = math.sqrt(0.25)*0.15

        # Penalties

        #added these
        # self.rewards.ang_vel_z_l2.weight = 0.0
        #self.rewards.base_yaw_drift_l2.weight = -0.12

        #standard
        self.rewards.dof_torques_l2.weight = -10.5e-5 #was -2.5e-5

        self.rewards.feet_air_time.weight = 0.125 #was 0.22
        self.rewards.feet_air_time.params["threshold"] = 0.1  #rewarding a duty cycle of 2/3 -- trying to also get use of six legs
        
        self.rewards.dof_pos_limits.weight = -1.0
        self.rewards.dof_acc_l2.weight= 0.0 #-8.5e-7 #new
        self.rewards.ang_vel_xy_l2.weight = 0.0 #-0.00000001
        self.rewards.undesired_contacts.weight = -1.0
        self.rewards.lin_vel_z_l2.weight = -0.00000001 #-0.0001
        self.rewards.action_rate_l2.weight = -1.0e-12 #-0.0000001 # tune this

        # change terrain to flat
        self.scene.terrain.terrain_type = "plane"
        self.scene.terrain.terrain_generator = None

        # no height scan
        self.scene.height_scanner = None

        # override to make observations only previous actions -- open loop gait
        self.observations.policy.height_scan = None
        self.observations.policy.base_lin_vel = None
        self.observations.policy.base_ang_vel = None    #include
        self.observations.policy.projected_gravity = None   #include in future
        self.observations.policy.joint_pos = None   #include
        self.observations.policy.joint_vel = None   #include

        self.observations.policy.actions.history_length = 14 # was 5 @ 12_12_25 10-20-22
        
        # no terrain curriculum
        self.curriculum.terrain_levels = None

        # camera settings
        #self.viewer.eye = (-0.5, 0.0, 0.1)
        self.viewer.eye = (0.0, 0.5, 0.1)
        self.viewer.lookat = (0.0, 0.5, 0.0)
        self.viewer.origin_type = "asset_root"
        #self.viewer.origin_type = "world"
        self.viewer.asset_name = "robot"
        


class HexapodFlatEnvCfg_PLAY(HexapodFlatEnvCfg):
    def __post_init__(self) -> None:
        # post init of parent
        super().__post_init__()

        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # disable randomization for play
        self.observations.policy.enable_corruption = False
        # remove random pushing event
        self.events.base_external_force_torque = None
        self.events.push_robot = None

        # Play on the max velocity
        self.commands.base_velocity.ranges.lin_vel_x=(0.0,0.0)
        self.commands.base_velocity.ranges.ang_vel_z=(0.0,0.0)
        self.commands.base_velocity.ranges.lin_vel_y=(0.07,0.07)

        self.commands.base_velocity.resampling_time_range = (1000.0, 1000.0)

        self.events.physics_material.params["static_friction_range"]: (0.8, 0.8)
        self.events.physics_material.params["dynamic_friction_range"]: (0.6, 0.6)
        #self.scene.terrain.physics_material.static_friction= 0.8
        #self.scene.terrain.physics_material.dynamic_friction= 0.6

        # camera settings
        self.viewer.eye = (0.0, 0.5, 1.0) #overhead camera
        self.viewer.lookat = (0.0, 0.5, 0.00)
        self.viewer.origin_type = "world"
        self.viewer.asset_name = "robot"
