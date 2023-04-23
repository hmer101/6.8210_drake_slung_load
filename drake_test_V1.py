### LOCAL PYDRAKE EXAMPLE   ###

# from pydrake.common import FindResourceOrThrow
# from pydrake.multibody.parsing import Parser
# from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
# from pydrake.systems.analysis import Simulator
# from pydrake.systems.framework import DiagramBuilder

# builder = DiagramBuilder()
# plant, _ = AddMultibodyPlantSceneGraph(builder, 0.0)
# Parser(plant).AddModels(
#     FindResourceOrThrow("drake/examples/pendulum/Pendulum.urdf"))
# plant.Finalize()
# diagram = builder.Build()
# simulator = Simulator(diagram)


###  LOCAL MESHCAT EXAMPLE   ###

# python libraries
import os
import numpy as np
from IPython.display import HTML, display
# pydrake imports
from pydrake.all import (
    AddMultibodyPlantSceneGraph, 
    DiagramBuilder,
    Linearize, 
    LinearQuadraticRegulator, 
    LogVectorOutput,     
    MeshcatVisualizer, 
    ModelVisualizer, 
    Parser, 
    Simulator, 
    StartMeshcat,
    MultibodyPlant,
    Propeller,
    PropellerInfo,
    RigidTransform)

from underactuated import running_as_notebook, FindResource
from underactuated.scenarios import AddFloatingRpyJoint

# HOW THE x500 model SDF FILE WAS CHANGED
# 1. commented out the use_parent_model_frame for all joints
# 2. wrote out whole path i.e. /home/bilab/6.8210_project/sdf_models/models/x500/meshes/1345_prop_ccw.stl
# vs model://x500/meshes/1345_prop_ccw.stl
# 3. These two lines were used to find the current path 
# dir_path = os.path.dirname(os.path.realpath(__file__))
# print(dir_path)
# 4. Commented out Gazebo plugins because those tags are unsupported elements

# HOW THE DEFAULT.SDF FILE WAS CHANGED
# 1. Changed all <uri> tags under <include> to not just be the name of the file but /home/bilab/6.8210_project/sdf_models/models/x500 (for example, instead of just x500)
# 2. Commented out most of the first half of the world file (there can only be one model tag in the sdf file)

# Start the visualizer (run this cell only once, each instance consumes a port)
meshcat = StartMeshcat()

sdf_path = 'sdf_models/worlds/default.sdf'
# sdf_path = 'sdf_models/models/x500/model.sdf'
# sdf_path = 'sdf_models/models/tether/model.sdf'
# sdf_path = 'sdf_models/models/load_generic/model.sdf'

visualizer = ModelVisualizer(meshcat=meshcat)
visualizer.parser().AddModelFromFile(sdf_path)
visualizer.Run(loop_once=not running_as_notebook)


input("Press [Enter] to simulate...")
visualizer = ModelVisualizer(meshcat=meshcat)
visualizer.parser().AddModelFromFile(sdf_path)
visualizer.Run(loop_once=not running_as_notebook)
input("Press [Enter] to exit...")
meshcat.Delete()
meshcat.DeleteAddedControls()

