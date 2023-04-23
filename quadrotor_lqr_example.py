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



###  3D QUADROTOR EXAMPLE   ###
import math
import time

import matplotlib.pyplot as plt
import mpld3
import numpy as np
import pydot
from IPython.display import HTML, SVG, Latex, display
from pydrake.all import (AddMultibodyPlantSceneGraph, ConstantVectorSource,
                         DiagramBuilder, FirstOrderTaylorApproximation,
                         LinearQuadraticRegulator, MatrixGain,
                         MeshcatVisualizer, 
                         ModelVisualizer, MultibodyPlant, Parser,
                         Saturation, SceneGraph, Simulator, StartMeshcat,
                         WrapToSystem)
from pydrake.common.containers import namedview
from pydrake.examples.acrobot import (AcrobotGeometry, AcrobotInput,
                                      AcrobotPlant, AcrobotState)
from pydrake.examples.quadrotor import (QuadrotorGeometry, QuadrotorPlant,
                                        StabilizingLQRController)
from pydrake.solvers.mathematicalprogram import MathematicalProgram, Solve

from underactuated import FindResource, running_as_notebook
from underactuated.meshcat_cpp_utils import MeshcatSliders
from underactuated.quadrotor2d import Quadrotor2D, Quadrotor2DVisualizer

if running_as_notebook:
    mpld3.enable_notebook()

# Start the visualizer (run this cell only once, each instance consumes a port)
meshcat = StartMeshcat()
visualizer = ModelVisualizer(meshcat=meshcat)



def quadrotor_example():

    builder = DiagramBuilder()

    plant = builder.AddSystem(QuadrotorPlant())

    controller = builder.AddSystem(StabilizingLQRController(plant, [0, 0, 1]))
    builder.Connect(controller.get_output_port(0), plant.get_input_port(0))
    builder.Connect(plant.get_output_port(0), controller.get_input_port(0))

    # Set up visualization in MeshCat
    scene_graph = builder.AddSystem(SceneGraph())
    QuadrotorGeometry.AddToBuilder(builder, plant.get_output_port(0), scene_graph)
    meshcat.Delete()
    meshcat.ResetRenderMode()
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    # end setup for visualization

    diagram = builder.Build()

    # Set up a simulator to run this diagram
    simulator = Simulator(diagram)
    simulator.set_target_realtime_rate(1.0 if running_as_notebook else 0.0)
    context = simulator.get_mutable_context()


    input("Press [Enter] to simulate...")
    # Simulate
    for i in range(5):
        context.SetTime(0.)
        context.SetContinuousState(0.5*np.random.randn(12,))
        simulator.Initialize()
        simulator.AdvanceTo(4.0 if running_as_notebook else 0.1)
        # time.sleep(0.1)

quadrotor_example()
input("Press [Enter] to exit...")


