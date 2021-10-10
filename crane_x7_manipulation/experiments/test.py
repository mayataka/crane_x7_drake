import os
import xacro


from pydrake.systems.framework import DiagramBuilder
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
# from pydrake.common import FindResourceOrThrow
from pydrake.multibody.parsing import Parser

#  FindResourceOrThrow, GenerateHtml, InverseDynamicsController, 
#     MultibodyPlant, Parser, Simulator, MeshcatVisualizerCpp)


builder = DiagramBuilder()
time_step = 0.01

urdf_path = os.path.abspath("crane_x7_description/crane_x7.urdf")

plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=time_step)
model = Parser(plant, scene_graph).AddModelFromFile(urdf_path)
