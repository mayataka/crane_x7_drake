from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.analysis import Simulator
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.multibody.parsing import Parser

import numpy as np
import os


builder = DiagramBuilder()

urdf_path = os.path.abspath('crane_x7_description/crane_x7.urdf')
plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.01)
model = Parser(plant, scene_graph).AddModelFromFile(urdf_path)

# builder.Connect(scene_graph.get_query_output_port(),
#                 plant.get_geometry_query_input_port())

plant.Finalize()



diagram = builder.Build()
context = diagram.CreateDefaultContext()
# print(context)

context = plant.CreateDefaultContext()
plant.SetPositions(context, [-1.57, 0.1, 0, 0, 0, 1.6, 0, 0, 0]) # 7DOF (arm) + 2DOF (gripper)
plant.GetJointByName("crane_x7_upper_arm_revolute_part_rotate_joint").set_angle(context, -1.2)
# print(context)
plant.get_actuation_input_port().FixValue(context, np.zeros(8)) # 7DOF (arm) + 1DOF (gripper)
simulator = Simulator(plant, context)
simulator.AdvanceTo(5.0)
# print(context)
