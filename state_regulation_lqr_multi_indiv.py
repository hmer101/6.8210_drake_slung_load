import numpy as np
import utils

from pydrake.all import(
    AddMultibodyPlantSceneGraph, 
    DiagramBuilder,
    LinearQuadraticRegulator,    
    MeshcatVisualizer, 
    Parser, 
    StartMeshcat,
    Propeller,
    PropellerInfo,
    RigidTransform)

from underactuated.scenarios import AddFloatingRpyJoint

NAME_SWARM = "swarm"
NAME_PROPS = "propellers"
NAME_DIAGRAM_QUAD = "quads_diagram"
NAME_DIAGRAM_WITH_CONTROLLER = "quads_with_controller"

GROUP_PREFIX = "swarm::"
MODEL_PREFIX_DRONE = "x500_"
NUM_DRONES = 3
FIRST_DRONE_NUM = 1


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



# Function to create an isolated subsystem for a specific drone
def create_isolated_drone_subsystem(sdf_path_drones, meshcat):
    builder = DiagramBuilder()
    isolated_plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
    
    parser = Parser(isolated_plant)
    drone_instance = parser.AddModelFromFile(sdf_path_drones)
    isolated_plant.Finalize()
    
    # Add Propellers, export inputs/outputs, and other configurations for the isolated drone
    # Default parameters from quadrotor_plant.cc:
    L = 0.174  # Length for arms (m)
    kF = 1.0  # Force input constant.
    kM = 0.0245  # Moment input constant.

    body_index = isolated_plant.GetBodyByName("base_link", drone_instance).index()

    # Note: Rotors 0 and 1 rotate one way and rotors 2 and 3 rotate the other.
    prop_info = [
        PropellerInfo(body_index, RigidTransform([L, -L, 0]), kF, kM), # rotor 0
        PropellerInfo(body_index, RigidTransform([-L, L, 0]), kF, kM), # rotor 1
        PropellerInfo(body_index, RigidTransform([L, L, 0]), kF, -kM), # rotor 2 cw
        PropellerInfo(body_index, RigidTransform([-L, -L, 0]), kF, -kM), # rotor 3 cw
    ]

    ## Connect diagram
    propellers = builder.AddSystem(Propeller(prop_info))
    propellers.set_name(NAME_PROPS) # + "_" + str(i))

    builder.Connect(propellers.get_output_port(), isolated_plant.get_applied_spatial_force_input_port()) #force_concat_sys.get_input_port(i-1)) # Connect propeller outputs to force concatinator
    builder.Connect(isolated_plant.get_body_poses_output_port(), propellers.get_body_poses_input_port(),)

    builder.ExportInput(propellers.get_command_input_port(), "test_u")
    builder.ExportOutput(isolated_plant.get_state_output_port(drone_instance), "test_state")
    
    meshcat.Delete()
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    
    return builder.Build()



# Make a multibody quadrotor with propellers diagram
# Return this diagram that includes the corresponding scene graph for visualizing simulation
def MakeMultibodyPlant(sdf_path_drones, sdf_path_load, sdf_path_tether, meshcat):
    ## Add quadrotor
    builder = DiagramBuilder()

    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0) #plant = builder.AddSystem(MultibodyPlant(0.0))
    plant.set_name(NAME_SWARM)

    #parser = Parser(plant)
    #model_instance = parser.AddModelFromFile(sdf_path)
    #model_instance = parser.AddModels(sdf_path)

    drone_diagrams = []
    for i in range(FIRST_DRONE_NUM,NUM_DRONES+1):
        next_drone_diagram = create_isolated_drone_subsystem(sdf_path_drones, meshcat)
        drone_diagrams.append(next_drone_diagram)
        #builder.AddNamedSystem(next_drone, MODEL_PREFIX_DRONE + str(i))
        next_drone = builder.AddSystem(next_drone_diagram)
        next_drone.set_name(MODEL_PREFIX_DRONE + str(i))


    diagram = builder.Build()
    diagram.set_name(NAME_DIAGRAM_QUAD)

    #utils.show_diagram(diagram)

    
    #load_instance = parser.AddModelFromFile(sdf_path_load)
    # drone_instances = []
    # tether_instances = []
    # for i in range(FIRST_DRONE_NUM,NUM_DRONES+1):
    #     parser = Parser(plant)
    #     drone_name = MODEL_PREFIX_DRONE + str(i)
    #     drone_instance = parser.AddModelFromFile(sdf_path_drones, model_name = drone_name)

    #     drone_instances.append(drone_instance)

    #     AddFloatingRpyJoint(plant, plant.GetFrameByName("base_link", drone_instance), drone_instance, use_ball_rpy=False)
    #     #drone_instances[i-1].set_name(MODEL_PREFIX_DRONE + str(i))
        #.set_name(NAME_DIAGRAM_QUAD)

        #tether_instances.append(parser.AddModelFromFile(sdf_path_tether))


    ## Setup variables for co-ordinate changing and propeller connection 
    # Default parameters from quadrotor_plant.cc:
    L = 0.174  # Length for arms (m)
    kF = 1.0  # Force input constant.
    kM = 0.0245  # Moment input constant.

    # ## Change frame for all drones in the group -> Use quaternions without this.
    #for i in range(FIRST_DRONE_NUM,NUM_DRONES+1):
    #     next_drone_name = GROUP_PREFIX + MODEL_PREFIX_DRONE + str(i)
    #     next_drone_instance = plant.GetModelInstanceByName(next_drone_name)
    #     #print(next_drone_name)

    #     # By default multibody has a quaternion floating base, so we manually add a floating roll pitch yaw joint to match QuadrotorPlant
    #     # We set `use_ball_rpy` to false because the BallRpyJoint uses angular velocities instead of ṙ, ṗ, ẏ.
    #     AddFloatingRpyJoint(plant, plant.GetFrameByName("base_link", next_drone_instance), next_drone_instance, use_ball_rpy=False)
    #     #plant.Finalize()

    #plant.Finalize()

    # # Instantiate the SpatialForceConcatinator to collect all propeller forces to feed into the MultiBodyPlant
    # spatial_force_concat = utils.SpatialForceConcatinator_[None](NUM_DRONES)
    # force_concat_sys = builder.AddSystem(spatial_force_concat)

    # ## Add propellers
    # for i in range(FIRST_DRONE_NUM,NUM_DRONES+1):
    #     next_drone_name = GROUP_PREFIX + MODEL_PREFIX_DRONE + str(i)
    #     next_drone_instance = plant.GetModelInstanceByName(next_drone_name)

    #     body_index = plant.GetBodyByName("base_link", next_drone_instance).index()

    #     # Note: Rotors 0 and 1 rotate one way and rotors 2 and 3 rotate the other.
    #     prop_info = [
    #         PropellerInfo(body_index, RigidTransform([L, -L, 0]), kF, kM), # rotor 0
    #         PropellerInfo(body_index, RigidTransform([-L, L, 0]), kF, kM), # rotor 1
    #         PropellerInfo(body_index, RigidTransform([L, L, 0]), kF, -kM), # rotor 2 cw
    #         PropellerInfo(body_index, RigidTransform([-L, -L, 0]), kF, -kM), # rotor 3 cw
    #     ]

    #     ## Connect diagram
    #     propellers = builder.AddSystem(Propeller(prop_info))
    #     propellers.set_name(NAME_PROPS + "_" + str(i))

    #     builder.Connect(propellers.get_output_port(0), force_concat_sys.get_input_port(i-1)) # Connect propeller outputs to force concatinator
    #     builder.Connect(plant.get_body_poses_output_port(), propellers.get_body_poses_input_port(),)
    #     builder.ExportInput(propellers.get_command_input_port(), next_drone_name + "_u")
    #     builder.ExportOutput(plant.get_state_output_port(next_drone_instance), next_drone_name + "_state")

    # # Connect the propeller forces concatinator to the multibody plant
    # builder.Connect(force_concat_sys.get_output_port(0),
    #                 plant.get_applied_spatial_force_input_port())

    # # Add meshcat visualizer
    # meshcat.Delete()
    # MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)

    ## Build diagram
    diagram = builder.Build()
    diagram.set_name(NAME_DIAGRAM_QUAD)

    # Show diagram
    utils.show_diagram(diagram)

    return diagram


# Make an LQR controller for state regulation of the plant given in the input diagram
def MakeQuadrotorController(diagram_plant):
    def QuadrotorLQR(diagram_plant):
        ## Setup
        # Get full drone sys
        drone_sys = diagram_plant.GetSubsystemByName(NAME_SWARM)

        # # Get first drone instance
        # first_drone_name = GROUP_PREFIX + MODEL_PREFIX_DRONE + FIRST_DRONE_NUM
        # first_drone_instance = plant.GetModelInstanceByName(first_drone_name)
        
        # Create contexts
        diagram_context = diagram_plant.CreateDefaultContext()        
        drone_context = drone_sys.GetMyContextFromRoot(diagram_context)
        #prop_context = prop_sys.GetMyContextFromRoot(diagram_context)

        ## Set plant at linearization point
        # States (Note states for tethers, load and reference link are also required)
        drone_1 = [0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        drone_2 = [2.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        drone_3 = [0.0, 2.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        drone_context.SetContinuousState(drone_1 + drone_2 + drone_3)

        # Inputs
        # single_drone_mass = drone_sys.CalcTotalMass(drone_context, first_drone_instance)
        # g = drone_sys.gravity_field().kDefaultStrength
        # diagram_plant.get_input_port().FixValue(diagram_context, single_drone_mass * g / 4. * np.ones([1, 1, 1, 1])) # TODO: U0 Different for when carrying load probably

        g = drone_sys.gravity_field().kDefaultStrength

        for i in range(FIRST_DRONE_NUM,NUM_DRONES+1):
            # Get drone instance
            drone_name = GROUP_PREFIX + MODEL_PREFIX_DRONE + str(i)
            drone_instance = drone_sys.GetModelInstanceByName(drone_name)

            # Set input port linearization
            drone_mass = drone_sys.CalcTotalMass(drone_context, [drone_instance])
            diagram_plant.get_input_port(i-1).FixValue(diagram_context, drone_mass * g / 4. * np.array([1, 1, 1, 1])) # TODO: U0 Different for when carrying load probably


        ## Other parameters
        Q = np.diag([10, 10, 10, 10, 10, 10, 1, 1, 1, 1, 1, 1])
        R = np.diag([0.1, 0.1, 0.1, 0.1])
 
        # Perhaps try LMPC: https://drake.mit.edu/doxygen_cxx/classdrake_1_1systems_1_1controllers_1_1_linear_model_predictive_controller.html
        # or finiteHorizonLQR: https://drake.mit.edu/doxygen_cxx/structdrake_1_1systems_1_1controllers_1_1_finite_horizon_linear_quadratic_regulator_options.html

 
        # TODO: HEREREEEEEE - LQR NOT WORKING. Potentially do separate LQR for different drones??? 
        # If can't split the drone plant easily, may have to do joints in Drake rather than SDF

        return LinearQuadraticRegulator(diagram_plant, diagram_context, Q, R)

    lqr_controller = QuadrotorLQR(diagram_plant)

    ## Build diagram with plant and controller
    builder = DiagramBuilder()

    # Add systems
    plant = builder.AddSystem(diagram_plant)
    plant.set_name("Drone with props")

    controller = builder.AddSystem(lqr_controller)
    controller.set_name("x500_0 controller")

    # Connect diagram
    builder.Connect(controller.get_output_port(0), plant.get_input_port(0))
    builder.Connect(plant.get_output_port(0), controller.get_input_port(0))

    # Build diagram
    diagram = utils.build_diagram(builder, "with controller test")

    return diagram 


def main():
    # Start the visualizer (run this cell only once, each instance consumes a port)
    meshcat = StartMeshcat()

    # Make Quadrotor
    sdf_path_drones = 'sdf_models/models/x500/model.sdf'
    sdf_path_load = 'sdf_models/models/load_generic/model.sdf'
    sdf_path_tether = 'sdf_models/models/tether/model.sdf'
    #sdf_path = 'sdf_models/worlds/default.sdf'
    #sdf_path = 'sdf_models/worlds/default_commented.sdf'
    diagram_plant = MakeMultibodyPlant(sdf_path_drones, sdf_path_load, sdf_path_tether, meshcat)

    # Show diagram
    #utils.show_diagram(diagram_quad)

    # Make controller
    diagram_full = MakeQuadrotorController(diagram_plant)

    # Show diagram
    utils.show_diagram(diagram_full)

    # Simulate
    state_init = 0.5*np.random.randn(12,)
    utils.simulate_diagram(diagram_full, state_init, meshcat, realtime_rate=0.75)


if __name__ == "__main__":
    main()