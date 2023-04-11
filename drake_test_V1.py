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

# Start the visualizer (run this cell only once, each instance consumes a port)
meshcat = StartMeshcat()

sdf_path = 'sdf_models/models/x500/model.sdf'
# sdf_path = 'sdf_models/worlds/default.sdf'

visualizer = ModelVisualizer(meshcat=meshcat)
visualizer.parser().AddModelFromFile(sdf_path)
visualizer.Run(loop_once=not running_as_notebook)

# HACK TO KEEP OPEN I guess this works
while True:
    pass

meshcat.Delete()
meshcat.DeleteAddedControls()

