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
import matplotlib
matplotlib.use('TkAgg') # Because I get error "Matplotlib is currently using agg, which is a non-GUI backend, so cannot show the figure.
import matplotlib.pyplot as plt
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
    RigidTransform,
    plot_system_graphviz)

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

# HOW THE DEFAULT.SDF FILE WAS CHANGED_notebook
# 1. Changed all <uri> tags under <include> to not just be the name of the file but /home/bilab/6.8210_project/sdf_models/models/x500 (for example, instead of just x500)
# 2. Commented out most of the first half of the world file (there can only be one model tag in the sdf file)

# Start the visualizer (run this cell only once, each instance consumes a port)
meshcat = StartMeshcat()

# sdf_path = 'sdf_models/worlds/default.sdf'
sdf_path = 'sdf_models/models/x500/model.sdf'
# sdf_path = 'sdf_models/models/tether/model.sdf'
# sdf_path = 'sdf_models/models/load_generic/model.sdf'

builder = DiagramBuilder()

plant = builder.AddSystem(MultibodyPlant(0.0))
parser = Parser(plant)
model_instance = parser.AddModelFromFile(sdf_path)

# By default multibody has a quaternion floating base, so we manually add a floating roll pitch yaw joint to match QuadrotorPlant
AddFloatingRpyJoint(plant, plant.GetFrameByName("base_link"), model_instance, use_ball_rpy=False)
plant.Finalize()

# Add propellers
body_index = plant.GetBodyByName("base_link").index()
# Default parameters from quadrotor_plant.cc:
L = 0.174  # Length for arms (m)
kF = 1.0  # Force input constant.
kM = 0.0245  # Moment input constant.

# Note: Rotors 0 and 1 rotate one way and rotors 2 and 3 rotate the other.
prop_info = [
    PropellerInfo(body_index, RigidTransform([L, -L, 0]), kF, kM), # rotor 0
    PropellerInfo(body_index, RigidTransform([-L, L, 0]), kF, kM), # rotor 1
    PropellerInfo(body_index, RigidTransform([L, L, 0]), kF, -kM), # rotor 2 cw
    PropellerInfo(body_index, RigidTransform([-L, -L, 0]), kF, -kM), # rotor 3 cw
]

propellers = builder.AddSystem(Propeller(prop_info))
builder.Connect(propellers.get_output_port(), plant.get_applied_spatial_force_input_port(),)
builder.Connect(plant.get_body_poses_output_port(), propellers.get_body_poses_input_port(),)
builder.ExportInput(propellers.get_command_input_port(), "u")

diagram = builder.Build()
diagram.set_name("diagram")

# Note: had to "sudo apt install graphviz" because I got [Errno 2] "dot" not found in path
# Added matplotlib.use('TkAgg') to block of imports above
# Ran sudo apt-get install python3-tk to get the plots to show
plt.figure(figsize=(20,10))
plot_system_graphviz(diagram)
plt.show()

print('hello')


# visualizer = ModelVisualizer(meshcat=meshcat)
# visualizer.parser().AddModelFromFile(sdf_path)
# visualizer.Run(loop_once=not running_as_notebook)

# # HACK TO KEEP OPEN I guess this works
# while True:
#     pass

meshcat.Delete()
meshcat.DeleteAddedControls()

