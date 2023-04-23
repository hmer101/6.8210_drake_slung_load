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
    Diagram,
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

from pydrake.examples import (
    StabilizingLQRController,
)

from underactuated import running_as_notebook, FindResource
from underactuated.scenarios import AddFloatingRpyJoint

NAME_DRONE = "drone"
NAME_PROPS = "propellers"
NAME_DIAGRAM = "quad_diagram"


# HOW THE x500 model SDF FILE WAS CHANGED
# 1. commented out the use_parent_model_frame for all joints
# 2. wrote out whole path i.e. /home/bilab/6.8210_project/sdf_models/models/x500/meshes/1345_prop_ccw.stl
# vs model://x500/meshes/1345_prop_ccw.stl
# 3. These two lines were used to find the current path 
# dir_path = os.path.dirname(os.path.realpath(__file__))
# print(dir_path)
# 4. Commented out Gazebo plugins because those tags are unsupported elements
# 5. Set all rotor "joints" from "revolute" to "fixed". Propellers will be modeled with propeller class, unlike in Gazebo. 
# If joints are kept as revolute, the number of states will be 20 (x,y,z,r,p,yaw,and velocities (12) plus position, velocity of rotors x4 (8))

# HOW THE DEFAULT.SDF FILE WAS CHANGED_notebook
# 1. Changed all <uri> tags under <include> to not just be the name of the file but /home/bilab/6.8210_project/sdf_models/models/x500 (for example, instead of just x500)
# 2. Commented out most of the first half of the world file (there can only be one model tag in the sdf file)


def MakeMultibodyQuadrotor(sdf_path):
    #sdf_path = 'sdf_models/worlds/default.sdf'
    #sdf_path = 'sdf_models/models/x500/model.sdf'
    # sdf_path = 'sdf_models/models/tether/model.sdf'
    # sdf_path = 'sdf_models/models/load_generic/model.sdf'

    builder = DiagramBuilder()

    plant = builder.AddSystem(MultibodyPlant(0.0))
    plant.set_name(NAME_DRONE)

    parser = Parser(plant)
    #ConfigureParser(parser)
    model_instance = parser.AddModelFromFile(sdf_path)

    # By default multibody has a quaternion floating base, so we manually add a floating roll pitch yaw joint to match QuadrotorPlant
    #  We set `use_ball_rpy` to false because the BallRpyJoint uses angular velocities instead of ṙ, ṗ, ẏ.
    AddFloatingRpyJoint(plant, plant.GetFrameByName("base_link"), model_instance, use_ball_rpy=False)
    # AddFloatingRpyJoint(plant, plant.GetFrameByName("base_link"),
    #                     model_instance, use_ball_rpy=False)
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
    propellers.set_name(NAME_PROPS)

    builder.Connect(propellers.get_output_port(), plant.get_applied_spatial_force_input_port(),)
    builder.Connect(plant.get_body_poses_output_port(), propellers.get_body_poses_input_port(),)
    builder.ExportInput(propellers.get_command_input_port(), "u")

    # diagram = builder.Build()
    # diagram.set_name("quad_diagram")

    return builder, plant


def MakeQuadrotorController(diagram_plant_act): #, drone_sys, propeller_sys):
    def QuadrotorLQR(diagram_plant_act): #, drone_sys, propeller_sys):    # TODO: MAKE Q & R correct sizes
        drone_sys = diagram_plant_act.GetSubsystemByName(NAME_DRONE)
        prop_sys = diagram_plant_act.GetSubsystemByName(NAME_PROPS)
        
        # Create contexts
        diagram_context = diagram_plant_act.CreateDefaultContext()
        #context.SetContinuousState(np.zeros([13, 1])) 
        #plant.get_input_port(0).FixValue(context, plant.mass * plant.gravity / 2. * np.array([1, 1]))
        
        drone_context = drone_sys.GetMyContextFromRoot(
            diagram_context)
        prop_context = prop_sys.GetMyContextFromRoot(
            diagram_context)

        ## Set plant at linearization point
        # States
        drone_cs = drone_context.get_continuous_state()
        nq = drone_cs.num_q() # 7 with quaternion, 6 with RPY floating base
        nv = drone_cs.num_v() # 6 
        nz = drone_cs.num_z() # 0

        props_cs = prop_context.get_continuous_state()
        nq_p = props_cs.num_q() # 0
        nv_p = props_cs.num_v() # 0
        nz_p = props_cs.num_z() # 0

        drone_context.SetContinuousState([0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) # CORRECT ORDER?? change 2nd to 0
        #prop_context.SetContinuousState([0]) # Don't need to set linearization??

        num_props = prop_sys.num_propellers()

        print(diagram_context)

        # Inputs
        #diagram_plant_act.
        drone_mass = drone_sys.CalcTotalMass(drone_context)
        g = drone_sys.gravity_field().kDefaultStrength
        diagram_plant_act.get_input_port().FixValue(diagram_context, drone_mass * g / 4. * np.array([1, 1, 1, 1])) # TODO: U0 Different for when carrying load probably

        ## Other parameters
        #n_act = diagram_plant_act.num_actuators()
        #test_act_port = diagram_plant_act.get_actuation_input_port() #.FixValue(context, [])

        Q = np.diag([10, 10, 10, 10, 10, 10, 1, 1, 1, 1, 1, 1])
        R = np.diag([0.1, 0.1, 0.1, 0.1]) #np.array([[0.1, 0.05], [0.05, 0.1]])
 

        # Perhaps try LMPC: https://drake.mit.edu/doxygen_cxx/classdrake_1_1systems_1_1controllers_1_1_linear_model_predictive_controller.html
        # or finiteHorizonLQR: https://drake.mit.edu/doxygen_cxx/structdrake_1_1systems_1_1controllers_1_1_finite_horizon_linear_quadratic_regulator_options.html

        return LinearQuadraticRegulator(diagram_plant_act, diagram_context, Q, R) #, input_port_index=diagram_plant_act.get_actuation_input_port().get_index()) #FiniteHorizonLinearQuadraticRegulator(plant, context, Q, R)


    lqr_controller = QuadrotorLQR(diagram_plant_act)
    controller = builder.AddSystem(lqr_controller) # OUTPUT PROBABLY INCORRECT #StabilizingLQRController(plant, [0, 0, 1]))
    builder.Connect(controller.get_output_port(0), plant.get_input_port(0)) # Are these the correct ports??
    builder.Connect(plant.get_output_port(0), controller.get_input_port(0))

    # # Set up visualization in MeshCat
    # scene_graph = builder.AddSystem(SceneGraph())
    # QuadrotorGeometry.AddToBuilder(builder, plant.get_output_port(0), scene_graph)
    # meshcat.Delete()
    # meshcat.ResetRenderMode()
    # meshcat.SetProperty('/Background','visible',False)
    # MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    # # end setup for visualization

    # diagram = builder.Build()

    return builder, controller


def build_diagram(builder, name):
    diagram = builder.Build()
    diagram.set_name(name)

    return diagram

def show_diagram(diagram):
    # Show diagram
    # Note: had to "sudo apt install graphviz" because I got [Errno 2] "dot" not found in path
    # Added matplotlib.use('TkAgg') to block of imports above
    # Ran sudo apt-get install python3-tk to get the plots to show
    plt.figure(figsize=(20,10))
    plot_system_graphviz(diagram)
    plt.show()

def meshcat_visualize(): #builder
    # Set up visualization in MeshCat - for non-SDF quadrotor??
    # scene_graph = builder.AddSystem(SceneGraph())
    # QuadrotorGeometry.AddToBuilder(builder, plant.get_output_port(0), scene_graph)
    # meshcat.Delete()
    # meshcat.ResetRenderMode()
    # meshcat.SetProperty('/Background','visible',False)
    # MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    # # end setup for visualization

    # diagram = builder.Build()

    # Visualize with Meshcat
    visualizer = ModelVisualizer(meshcat=meshcat)
    visualizer.parser().AddModelFromFile(sdf_path)
    visualizer.Run(loop_once=not running_as_notebook)

def simulate_diagram(diagram):
    # Set up a simulator to run this diagram
    simulator = Simulator(diagram)
    simulator.set_target_realtime_rate(1.0 if running_as_notebook else 0.0)
    context = simulator.get_mutable_context()

    # Simulate
    for i in range(5):
        context.SetTime(0.)
        context.SetContinuousState(0.5*np.random.randn(12,)) #13?
        simulator.Initialize()
        simulator.AdvanceTo(4.0 if running_as_notebook else 0.1)


# Start the visualizer (run this cell only once, each instance consumes a port)
meshcat = StartMeshcat()

# Make Quadrotor
sdf_path = 'sdf_models/models/x500/model.sdf'
#sdf_path = 'sdf_models/worlds/default.sdf'
builder, plant = MakeMultibodyQuadrotor(sdf_path)

# Make controller
#builder, controller = MakeQuadrotorController(builder, plant)


# # LQR Controller 
# # Note: All joints in sdf file changed from "revolute" to "fixed". Number of continuous states should be 12
# context = plant.CreateDefaultContext()
# #context.SetContinuousState(np.zeros([12, 1]))
# context.SetContinuousState(np.zeros([13, 1]))
# #print(plant.gravity)


# Build and show diagram
diagram = build_diagram(builder, NAME_DIAGRAM)
#show_diagram(diagram)

# Make controller
builder, controller = MakeQuadrotorController(diagram) #, drone_sys, propeller_sys) #MakeQuadrotorController(builder, plant)


input('Press any key to end \n')

input("Press [Enter] to simulate...")
visualizer = ModelVisualizer(meshcat=meshcat)
visualizer.parser().AddModelFromFile(sdf_path)
visualizer.Run(loop_once=not running_as_notebook)
input("Press [Enter] to exit...")
meshcat.Delete()
meshcat.DeleteAddedControls()

