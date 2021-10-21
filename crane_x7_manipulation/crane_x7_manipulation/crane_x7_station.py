from pydrake.manipulation.planner import DifferentialInverseKinematicsIntegrator, DifferentialInverseKinematicsParameters
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, MultibodyPlant
from pydrake.multibody.parsing import Parser
from pydrake.systems.controllers import PidController, InverseDynamicsController

import os
import numpy as np


class CraneX7Station(object):
    def __init__(self, urdf_path):
        self.urdf_path = urdf_path
        self.end_effector_name = 'crane_x7_gripper_base_link'
        self.gripper_a_joint_name = 'crane_x7_gripper_finger_a_joint'
        self.gripper_b_joint_name = 'crane_x7_gripper_finger_b_joint'
        self.plant = None
        self.model = None
        self.scene_graph = None
        self.diff_ik_params = DifferentialInverseKinematicsParameters(
            num_positions=9, num_velocities=9) 
        self.diff_ik = None
        self.pid_controller = None
        self.invdyn_controller = None
        self.num_positions = 9
        self.num_velocities = 9
        self.num_arm_positions = 7
        self.num_arm_velocities = 7
        self.num_gripper_positions = 2
        self.num_gripper_velocities = 2
    
    def get_home_position(self):
        return np.array([0., 0.3979, 0., -2.1994, 0.0563, 0.4817, 0., 0., 0.])

    def create_crane_x7_plant(self, time_step=0.01, finalize=False):
        self.plant = MultibodyPlant(time_step=time_step)
        self.model = Parser(self.plant).AddModelFromFile(os.path.abspath(self.urdf_path))
        if finalize:
            self.plant.Finalize()
        return self.plant, self.model

    def get_end_effector_frame(self):
        return self.plant.GetBodyByName(self.end_effector_name).body_frame()

    def create_crane_x7_plant_scene_graph(self, builder, time_step=0.01, finalize=False):
        self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=time_step)
        self.model = Parser(self.plant, self.scene_graph).AddModelFromFile(os.path.abspath(self.urdf_path))
        if finalize:
            self.plant.Finalize()
        return self.plant, self.model, self.scene_graph

    def create_diff_ik(self, time_step=None, robot_context=None, end_effector_velocity_gain=None):
        if time_step is None:
            time_step = self.plant.time_step()
        self.diff_ik_params.set_joint_position_limits([self.plant.GetPositionLowerLimits(),
                                                       self.plant.GetPositionUpperLimits()])
        self.diff_ik_params.set_joint_velocity_limits([self.plant.GetVelocityLowerLimits(),
                                                       self.plant.GetVelocityUpperLimits()])
        self.diff_ik_params.set_nominal_joint_position(self.get_home_position())
        if end_effector_velocity_gain is not None:
            self.diff_ik_params.set_end_effector_velocity_gain(end_effector_velocity_gain)
        self.diff_ik = DifferentialInverseKinematicsIntegrator(self.plant, self.get_end_effector_frame(),
                                                               time_step, self.diff_ik_params, robot_context)
        return self.diff_ik

    def create_pid_controller(self, kp=None, ki=None, kd=None):
        if kp is None:
            kp = 1.0 * np.ones(9)
        if ki is None:
            ki = 0.0 * np.ones(9)
        if kd is None:
            kd = 0.0 * np.ones(9)
        self.pid_controller = PidController(kp, ki, kd)
        return self.pid_controller

    def create_invdyn_controller(self, kp=None, ki=None, kd=None, has_reference_acceleration=False):
        if kp is None:
            kp = 100.0 * np.ones(9)
        if ki is None:
            ki = 1.0 * np.ones(9)
        if kd is None:
            kd = 20.0 * np.ones(9)
        self.invdyn_controller = InverseDynamicsController(self.plant, kp, ki, kd, has_reference_acceleration)
        return self.invdyn_controller