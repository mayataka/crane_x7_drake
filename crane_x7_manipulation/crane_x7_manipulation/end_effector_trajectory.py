from pydrake.systems.framework import LeafSystem
from pydrake.common.value import AbstractValue
from pydrake.math import RigidTransform

import numpy as np


class EndEffectorTrajectory(LeafSystem):
    def __init__(self, station, traj_gripper, gripper_command=None):
        LeafSystem.__init__(self)
        self.plant = station.plant
        self.end_effector_frame = station.plant.GetBodyByName(station.end_effector_name)
        self.gripper_a_joint = station.plant.GetJointByName(station.gripper_a_joint_name)
        self.gripper_b_joint = station.plant.GetJointByName(station.gripper_b_joint_name)
        self.traj_gripper = traj_gripper
        self.gripper_command = gripper_command
        self.plant_context = station.plant.CreateDefaultContext()
        self.DeclareAbstractOutputPort("X_WE", 
                                       lambda: AbstractValue.Make(RigidTransform()),
                                       self.CalcOutput)

    def CalcOutput(self, context, output):
        t = context.get_time()
        output.set_value(self.traj_gripper.GetPose(t))