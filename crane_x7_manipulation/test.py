from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.analysis import Simulator
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, MultibodyPlant
from pydrake.multibody.parsing import Parser
from pydrake.geometry import Meshcat
from pydrake.all import MeshcatVisualizerCpp, Integrator # where is this class declared ?

import numpy as np

from crane_x7_manipulation import crane_x7_station, meshcat_utils


URDF_PATH = 'rsc/crane_x7_description/crane_x7.urdf'


def create_model(urdf_path):
    station = crane_x7_station.CraneX7Station(urdf_path)
    plant, model = station.create_crane_x7_plant()
    plant.Finalize()
    context = plant.CreateDefaultContext()

    plant.SetPositions(context, [-1.57, 0.1, 0, 0, 0, 1.6, 0, 0, 0]) # 7DOF (arm) + 2DOF (gripper)
    plant.GetJointByName("crane_x7_upper_arm_revolute_part_rotate_joint").set_angle(context, -1.2)
    plant.get_actuation_input_port().FixValue(context, np.zeros(9)) # 7DOF (arm) + 2DOF (gripper)
    simulator = Simulator(plant, context)
    simulator.AdvanceTo(5.0)
    print(context)

# create_model(URDF_PATH)


def create_model_scene_graph(urdf_path):
    builder = DiagramBuilder()

    station = crane_x7_station.CraneX7Station(urdf_path)
    plant, model, scene_graph = station.create_crane_x7_plant_scene_graph(
        builder=builder, time_step=1.0e-03)
    plant.Finalize()

    meshcat = Meshcat()
    web_url = meshcat.web_url()
    # Javascript(f'window.open("{web_url}");')

    visualizer = MeshcatVisualizerCpp.AddToBuilder(builder, scene_graph, meshcat)
    diagram = builder.Build()
    context = diagram.CreateDefaultContext()
    diagram.Publish(context)

    plant_context = plant.GetMyMutableContextFromRoot(context)
    plant.SetPositions(plant_context, [-1.57, 0.1, 0, -1.2, 0, 1.6, 0, 0, 0]) # 7DOF (arm) + 2DOF (gripper)
    plant.get_actuation_input_port().FixValue(plant_context, np.zeros(9)) # 7DOF (arm) + 2DOF (gripper)

    # real time simulation on 
    simulator = Simulator(diagram, context)
    simulator.set_target_realtime_rate(1.0)
    simulator.AdvanceTo(1.0)

create_model_scene_graph(URDF_PATH)


# def create_diff_ik_controller(urdf_path):
#     builder = DiagramBuilder()

#     station = crane_x7_station.CraneX7Station(urdf_path)
#     plant, model, scene_graph = station.create_crane_x7_plant_scene_graph(
#         builder=builder, time_step=1.0e-03)
#     plant.Finalize()

#     meshcat = Meshcat()
#     web_url = meshcat.web_url()
#     # Javascript(f'window.open("{web_url}");')

#     visualizer = MeshcatVisualizerCpp.AddToBuilder(builder, scene_graph, meshcat)

#     controller = builder.AddSystem(station.create_diff_ik_controller())
#     # controller = builder.AddSystem(crane_x7_controller.DiffIKController(plant))
#     integrator = builder.AddSystem(Integrator(9))

#     builder.Connect(controller.get_output_port(), 
#                     integrator.get_input_port())
#     builder.Connect(integrator.get_output_port(),
#                     plant.get_actuation_input_port())
#     builder.Connect(plant.get_state_output_port(),
#                     controller.get_input_port())

#     diagram = builder.Build()
#     context = diagram.CreateDefaultContext()
#     diagram.Publish(context)

#     plant_context = plant.GetMyMutableContextFromRoot(context)

#     simulator = Simulator(diagram)
#     plant_context = plant.GetMyContextFromRoot(simulator.get_mutable_context())
#     plant.get_actuation_input_port().FixValue(plant_context, np.zeros((9, 1)))
#     integrator.set_integral_value(
#         integrator.GetMyContextFromRoot(simulator.get_mutable_context()), 
#             plant.get_state_output_port().Eval(plant_context)[0:9])
#     simulator.set_target_realtime_rate(1.0)
#     simulator.AdvanceTo(5.0)
#     print(plant_context)


def create_diff_ik_controller(urdf_path):
    builder = DiagramBuilder()

    station = crane_x7_station.CraneX7Station(urdf_path)
    plant, model, scene_graph = station.create_crane_x7_plant_scene_graph(
        builder=builder, time_step=1.0e-03)
    plant.Finalize()

    meshcat = Meshcat()
    web_url = meshcat.web_url()

    visualizer = MeshcatVisualizerCpp.AddToBuilder(builder, scene_graph, meshcat)

    controller = builder.AddSystem(station.create_diff_ik_controller())

    builder.Connect(controller.get_output_port(), 
                    plant.get_actuation_input_port())

#     builder.Connect(controller.get_output_port(), 
#                     integrator.get_input_port())
#     builder.Connect(integrator.get_output_port(),
#                     plant.get_actuation_input_port())
#     builder.Connect(plant.get_state_output_port(),
#                     controller.get_input_port())

    diagram = builder.Build()
    context = diagram.CreateDefaultContext()
    diagram.Publish(context)

    plant_context = plant.GetMyMutableContextFromRoot(context)

    simulator = Simulator(diagram)
    plant_context = plant.GetMyContextFromRoot(simulator.get_mutable_context())
    controller.SetPositions(plant_context, plant.get_state_output_port().Eval(plant_context)[0:9])
    simulator.set_target_realtime_rate(1.0)
    simulator.AdvanceTo(5.0)
    print(plant_context)

    teleop = builder.AddSystem(MeshcatPoseSliders(meshcat,
      min_range=MeshcatPoseSliders.MinRange(
          roll=0, x=-0.6, z=0.0),
      max_range=MeshcatPoseSliders.MaxRange(
          roll=2*np.pi, x=0.8, z=1.1),
      value=MeshcatPoseSliders.Value(pitch=0, yaw=0, y=0),
      visible=MeshcatPoseSliders.Visible(pitch=False, yaw=False, y=False) 
    ))

# create_diff_ik_controller(URDF_PATH)