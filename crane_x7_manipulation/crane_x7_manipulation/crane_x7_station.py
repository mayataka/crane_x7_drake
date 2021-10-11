from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.analysis import Simulator
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, MultibodyPlant
from pydrake.multibody.parsing import Parser
from pydrake.all import MeshcatVisualizerCpp # where is this class declared ?

import os


def create_crane_x7_plant(urdf_path, time_step=0.01, finalize=False):
    plant = MultibodyPlant(time_step=time_step)
    model = Parser(plant).AddModelFromFile(os.path.abspath(urdf_path))
    if finalize:
        plant.Finalize()
    return plant, model


def create_crane_x7_plant_scene_graph(builder, urdf_path, time_step=0.01, finalize=False):
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=time_step)
    model = Parser(plant, scene_graph).AddModelFromFile(os.path.abspath(urdf_path))
    if finalize:
        plant.Finalize()
    return plant, model, scene_graph