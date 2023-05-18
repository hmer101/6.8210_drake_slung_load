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
    Linearize,
    Demultiplexer,
    LogVectorOutput,
    Multiplexer,
    ExternallyAppliedSpatialForceMultiplexer,
    RigidTransform)

from underactuated.scenarios import AddFloatingRpyJoint

NAME_SWARM = "swarm"
NAME_PROPS = "propellers"
NAME_DIAGRAM_QUAD = "quads_diagram"
NAME_DIAGRAM_WITH_CONTROLLER = "quads_with_controller"

GROUP_PREFIX = "swarm::"
MODEL_PREFIX_DRONE = "x500_"
NUM_DRONES = 3 #1
FIRST_DRONE_NUM = 1
PROPS_PER_DRONE = 4


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

# Make a multibody quadrotor with propellers diagram
# Return this diagram that includes the corresponding scene graph for visualizing simulation
def MakeMultibodyQuadrotor(sdf_path, meshcat):
    ## Add quadrotor
    builder = DiagramBuilder()

    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0) #time_step=1e-4) #plant = builder.AddSystem(MultibodyPlant(0.0))
    plant.set_name(NAME_SWARM)

    parser = Parser(plant)
    #model_instance = parser.AddModelFromFile(sdf_path)
    model_instance = parser.AddModels(sdf_path)

    ## Setup variables for co-ordinate changing and propeller connection 
    # Default parameters from quadrotor_plant.cc:
    L = 0.174  # Length for arms (m)
    kF = 1.0  # Force input constant.
    kM = 0.0245  # Moment input constant.

    ## Change frame for all drones in the group -> Use quaternions without this.
    for i in range(FIRST_DRONE_NUM,NUM_DRONES+1):
        next_drone_name = GROUP_PREFIX + MODEL_PREFIX_DRONE + str(i)
        next_drone_instance = plant.GetModelInstanceByName(next_drone_name)
        #print(next_drone_name)

        # By default multibody has a quaternion floating base, so we manually add a floating roll pitch yaw joint to match QuadrotorPlant
        # We set `use_ball_rpy` to false because the BallRpyJoint uses angular velocities instead of ṙ, ṗ, ẏ.
        AddFloatingRpyJoint(plant, plant.GetFrameByName("base_link", next_drone_instance), next_drone_instance, use_ball_rpy=False)
        #plant.Finalize()

    plant.Finalize()

    ## Instantiate demux block to split a single input into multiple inputs. Gives the single input required for using LinearQuadraticRegular function
    # input_dim = NUM_DRONES*PROPS_PER_DRONE 
    # splits_input = PROPS_PER_DRONE*np.ones(NUM_DRONES)
    # splits_input = [int(x) for x in splits_input.tolist()]
    
    # input_demux = builder.AddSystem(Demultiplexer(splits_input)) #Demultiplexer(input_dim, splits))

    # first_drone_name = GROUP_PREFIX + MODEL_PREFIX_DRONE + str(1)
    # first_drone_instance = plant.GetModelInstanceByName(first_drone_name)
    
    # combines_output = plant.num_multibody_states(first_drone_instance)*np.ones(NUM_DRONES) #plant.num_multibody_states())
    # combines_output = [int(x) for x in combines_output.tolist()]

    # output_mux = builder.AddSystem(Multiplexer(combines_output))

    ## Instantiate the SpatialForceConcatinator to collect all propeller forces to feed into the MultiBodyPlant
    ## spatial_force_concat = utils.SpatialForceConcatinator_[None](NUM_DRONES)
    # spatial_force_concat = ExternallyAppliedSpatialForceMultiplexer(NUM_DRONES)
    # force_concat_sys = builder.AddSystem(spatial_force_concat)

    ## Add propellers
    prop_info = []
    for i in range(FIRST_DRONE_NUM,NUM_DRONES+1):
        next_drone_name = GROUP_PREFIX + MODEL_PREFIX_DRONE + str(i)
        next_drone_instance = plant.GetModelInstanceByName(next_drone_name)

        body_index = plant.GetBodyByName("base_link", next_drone_instance).index()

        # Note: Rotors 0 and 1 rotate one way and rotors 2 and 3 rotate the other.
        prop_info += [
            PropellerInfo(body_index, RigidTransform([L, -L, 0]), kF, kM), # rotor 0
            PropellerInfo(body_index, RigidTransform([-L, L, 0]), kF, kM), # rotor 1
            PropellerInfo(body_index, RigidTransform([L, L, 0]), kF, -kM), # rotor 2 cw
            PropellerInfo(body_index, RigidTransform([-L, -L, 0]), kF, -kM), # rotor 3 cw
        ]

        ## Connect diagram
        # propellers = builder.AddSystem(Propeller(prop_info))
        # propellers.set_name(NAME_PROPS + "_" + str(i))

        # builder.Connect(propellers.get_output_port(), plant.get_applied_spatial_force_input_port(),)
        # builder.Connect(plant.get_body_poses_output_port(), propellers.get_body_poses_input_port(),)
        # builder.ExportInput(propellers.get_command_input_port(), "u")
        # builder.ExportOutput(plant.get_state_output_port(model_instance), "x500_0_x")

        # builder.Connect(input_demux.get_output_port(i-1), propellers.get_command_input_port())
        # builder.Connect(propellers.get_output_port(0), force_concat_sys.get_input_port(i-1)) # Connect propeller outputs to force concatinator
        # builder.Connect(plant.get_body_poses_output_port(), propellers.get_body_poses_input_port())

        # builder.Connect(plant.get_state_output_port(next_drone_instance), output_mux.get_input_port(i-1))

        #builder.ExportInput(propellers.get_command_input_port(), next_drone_name + "_u")
        #builder.ExportOutput(plant.get_state_output_port(next_drone_instance), next_drone_name + "_state")

    propellers = builder.AddNamedSystem("propellers", Propeller(prop_info))

    builder.Connect(propellers.get_output_port(), plant.get_applied_spatial_force_input_port())
    builder.Connect(plant.get_body_poses_output_port(), propellers.get_body_poses_input_port())

    # # Export single mux input and output
    # builder.ExportInput(input_demux.get_input_port(0), "u_mux")
    # builder.ExportOutput(output_mux.get_output_port(), "state_mux")

    # # Connect the propeller forces concatinator to the multibody plant
    # builder.Connect(force_concat_sys.get_output_port(0),
    #                 plant.get_applied_spatial_force_input_port())

    builder.ExportInput(propellers.get_command_input_port(), "u")
    builder.ExportOutput(plant.get_state_output_port(), "q")

    # Add meshcat visualizer
    meshcat.Delete()
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)

    # Logger block to measured data
    logger = LogVectorOutput(plant.get_state_output_port(), builder)
    logger.set_name("logger")

    ## Build diagram
    diagram = builder.Build()
    diagram.set_name(NAME_DIAGRAM_QUAD)

    return diagram, logger


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
        # state_init = np.asarray([2.0, 0.0, 0.0, 0.0, 0.0, 0.0]+
        #                         [-2.0, 2.0, 0.0, 0.0, 0.0, 0.0]+
        #                         [-2.0, -2.0, 0.0, 0.0, 0.0, 0.0]+
        #                         [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]+
        #                         [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]+
        #                         [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        drone_1 = [2.0, -3.0, 1.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        drone_2 = [3, 2, 3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        drone_3 = [-2.5, 0, 2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        #drone_context.SetContinuousState(drone_1)
        x_0 = drone_1[0:6] + drone_2[0:6] + drone_3[0:6] + drone_1[6:12] + drone_2[6:12] + drone_3[6:12]
        drone_context.SetContinuousState(x_0)

        # print(f'num_total: {drone_context.num_total_states()}')
        # print(f'num_cont: {drone_context.num_continuous_states()}')
        # print(f'num_disc: {drone_context.get_discrete_state().size()}') #num_discrete_state_groups

        # Inputs
        input_dim = NUM_DRONES*PROPS_PER_DRONE 
        drone_name = GROUP_PREFIX + MODEL_PREFIX_DRONE + str(1)
        first_drone_instance = drone_sys.GetModelInstanceByName(drone_name)
        single_drone_mass = drone_sys.CalcTotalMass(drone_context, [first_drone_instance])
        g = drone_sys.gravity_field().kDefaultStrength 
        u_0 = single_drone_mass * g / 4. * np.array([1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1])

        diagram_plant.get_input_port().FixValue(diagram_context, u_0) #.FixValue(diagram_context, single_drone_mass * g / 4. * np.ones(input_dim)) # TODO: U0 Different for when carrying load probably

        #g = drone_sys.gravity_field().kDefaultStrength
        # for i in range(FIRST_DRONE_NUM,NUM_DRONES+1):
        #     # Get drone instance
        #     drone_name = GROUP_PREFIX + MODEL_PREFIX_DRONE + str(i)
        #     drone_instance = drone_sys.GetModelInstanceByName(drone_name)

        #     # Set input port linearization
        #     drone_mass = drone_sys.CalcTotalMass(drone_context, [drone_instance])
        #     diagram_plant.get_input_port(i-1).FixValue(diagram_context, drone_mass * g / 4. * np.array([1, 1, 1, 1])) # TODO: U0 Different for when carrying load probably
    

        Q_comb = np.diag([10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1])
        
        #np.diag(Q_diag[0:6] + Q_diag[0:6] + Q_diag[0:6] + Q_diag[6:12] + Q_diag[6:12] + Q_diag[6:12])
        #Q_alt = np.diag([10, 10, 10, 10, 10, 10, 1, 1, 1, 1, 1, 1, 10, 10, 10, 10, 10, 10, 1, 1, 1, 1, 1, 1, 10, 10, 10, 10, 10, 10, 1, 1, 1, 1, 1, 1])

        R_diag = [0.1, 0.1, 0.1, 0.1]
        R_comb = np.diag(R_diag + R_diag + R_diag)

        return LinearQuadraticRegulator(diagram_plant, diagram_context, Q_comb, R_comb)

    lqr_controller = QuadrotorLQR(diagram_plant)

    ## Build diagram with plant and controller
    builder = DiagramBuilder()

    # Add systems
    plant = builder.AddSystem(diagram_plant)
    plant.set_name("Swarm plant")

    controller = builder.AddSystem(lqr_controller)
    controller.set_name("Swarm controller")

    # Connect diagram
    builder.Connect(controller.get_output_port(0), plant.get_input_port(0))
    builder.Connect(plant.get_output_port(0), controller.get_input_port(0))

    # Build diagram
    diagram = utils.build_diagram(builder, "Swarm with controller")

    return diagram 


def main():
    # Start the visualizer (run this cell only once, each instance consumes a port)
    meshcat = StartMeshcat()

    # Make Quadrotor
    #sdf_path = 'sdf_models/models/x500/model.sdf'
    #sdf_path = 'sdf_models/worlds/default.sdf'
    sdf_path = 'sdf_models/worlds/default_3drones_Ryan.sdf'
    diagram_quad, log = MakeMultibodyQuadrotor(sdf_path, meshcat)

    # Show diagram
    #utils.show_diagram(diagram_quad)

    # Make controller
    diagram_full = MakeQuadrotorController(diagram_quad)

    # Show diagram
    # utils.show_diagram(diagram_full)

    # Simulate
    state_init = np.asarray([2.0, 0.0, 0.0, 0.0, 0.0, 0.0]+
                        [-2.0, 2.0, 0.0, 0.0, 0.0, 0.0]+
                        [-2.0, -2.0, 0.0, 0.0, 0.0, 0.0]+
                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]+
                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]+
                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    state_index_for_plotting = [0, 1, 2, 6, 7, 8, 12, 13, 14]
    
    utils.simulate_diagram(diagram_full, state_init, meshcat, logger=log, state_indices=state_index_for_plotting, realtime_rate=0.75)


if __name__ == "__main__":
    main()