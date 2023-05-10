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
NUM_DRONES = 1 #3 
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

    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0) #plant = builder.AddSystem(MultibodyPlant(0.0))
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
    # for i in range(FIRST_DRONE_NUM,NUM_DRONES+1):
    #     next_drone_name = GROUP_PREFIX + MODEL_PREFIX_DRONE + str(i)
    #     next_drone_instance = plant.GetModelInstanceByName(next_drone_name)
    #     #print(next_drone_name)

    #     # By default multibody has a quaternion floating base, so we manually add a floating roll pitch yaw joint to match QuadrotorPlant
    #     # We set `use_ball_rpy` to false because the BallRpyJoint uses angular velocities instead of ṙ, ṗ, ẏ.
    #     #AddFloatingRpyJoint(plant, plant.GetFrameByName("base_link", next_drone_instance), next_drone_instance, use_ball_rpy=False)
    #     #plant.Finalize()

    # Change frame for load
    load_name = GROUP_PREFIX + "load"
    load_instance = plant.GetModelInstanceByName(load_name)
    AddFloatingRpyJoint(plant, plant.GetFrameByName("base_link", load_instance), load_instance, use_ball_rpy=False)

    plant.Finalize()

    # Instantiate demux block to split a single input into multiple inputs. Gives the single input required for using LinearQuadraticRegular function
    # input_dim = NUM_DRONES*PROPS_PER_DRONE 
    splits_input = PROPS_PER_DRONE*np.ones(NUM_DRONES)
    splits_input = [int(x) for x in splits_input.tolist()]
    #print(splits_input)
    
    input_demux = builder.AddSystem(Demultiplexer(splits_input)) #Demultiplexer(input_dim, splits))

    first_drone_name = GROUP_PREFIX + MODEL_PREFIX_DRONE + str(1)
    first_drone_instance = plant.GetModelInstanceByName(first_drone_name)
    
    combines_output = plant.num_multibody_states(first_drone_instance)*np.ones(NUM_DRONES) #plant.num_multibody_states())
    combines_output = [int(x) for x in combines_output.tolist()]
    #print(combines_output)

    output_mux = builder.AddSystem(Multiplexer(combines_output))

    # Instantiate the SpatialForceConcatinator to collect all propeller forces to feed into the MultiBodyPlant
    #spatial_force_concat = utils.SpatialForceConcatinator_[None](NUM_DRONES)
    spatial_force_concat = ExternallyAppliedSpatialForceMultiplexer(NUM_DRONES)
    force_concat_sys = builder.AddSystem(spatial_force_concat)


    ## Add propellers
    for i in range(FIRST_DRONE_NUM,NUM_DRONES+1):
        load_name = GROUP_PREFIX + MODEL_PREFIX_DRONE + str(i)
        load_instance = plant.GetModelInstanceByName(load_name)

        body_index = plant.GetBodyByName("base_link", load_instance).index()

        # Note: Rotors 0 and 1 rotate one way and rotors 2 and 3 rotate the other.
        prop_info = [
            PropellerInfo(body_index, RigidTransform([L, -L, 0]), kF, kM), # rotor 0
            PropellerInfo(body_index, RigidTransform([-L, L, 0]), kF, kM), # rotor 1
            PropellerInfo(body_index, RigidTransform([L, L, 0]), kF, -kM), # rotor 2 cw
            PropellerInfo(body_index, RigidTransform([-L, -L, 0]), kF, -kM), # rotor 3 cw
        ]

        ## Connect diagram
        propellers = builder.AddSystem(Propeller(prop_info))
        propellers.set_name(NAME_PROPS + "_" + str(i))
        builder.Connect(input_demux.get_output_port(i-1), propellers.get_command_input_port())
        builder.Connect(propellers.get_output_port(0), force_concat_sys.get_input_port(i-1)) # Connect propeller outputs to force concatinator
        builder.Connect(plant.get_body_poses_output_port(), propellers.get_body_poses_input_port())

        builder.Connect(plant.get_state_output_port(load_instance), output_mux.get_input_port(i-1))

        #builder.ExportInput(propellers.get_command_input_port(), next_drone_name + "_u")
        #builder.ExportOutput(plant.get_state_output_port(next_drone_instance), next_drone_name + "_state")

    # Export single mux input and output
    builder.ExportInput(input_demux.get_input_port(0), "u_mux")
    builder.ExportOutput(output_mux.get_output_port(), "state_mux")

    # Connect the propeller forces concatinator to the multibody plant
    builder.Connect(force_concat_sys.get_output_port(0),
                    plant.get_applied_spatial_force_input_port())

    # Add meshcat visualizer
    meshcat.Delete()
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)

    ## Build diagram
    diagram = builder.Build()
    diagram.set_name(NAME_DIAGRAM_QUAD)

    return diagram


# Make an LQR controller for state regulation of the plant given in the input diagram
def MakeQuadrotorController(diagram_plant):
    def QuadrotorLQR(diagram_plant):
        ## Setup
        # Get full drone sys
        swarm_sys = diagram_plant.GetSubsystemByName(NAME_SWARM)

        # # Get first drone instance
        # first_drone_name = GROUP_PREFIX + MODEL_PREFIX_DRONE + FIRST_DRONE_NUM
        # first_drone_instance = plant.GetModelInstanceByName(first_drone_name)
        
        # Create contexts
        diagram_context = diagram_plant.CreateDefaultContext()        
        swarm_context = swarm_sys.GetMyContextFromRoot(diagram_context)
        #prop_context = prop_sys.GetMyContextFromRoot(diagram_context)

        ## Set plant at linearization point
        # States (Note states for tethers, load and reference link are also required)
        drone_1 = [0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        drone_2 = [2.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        drone_3 = [0.0, 2.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        load = [0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        state_0 = [0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 
                       0.0, -np.pi/2, 0.0, 
                       0.0, np.pi/2, 0.0, 
                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        print(f'num_total: {swarm_context.num_total_states()}')
        print(f'num_cont: {swarm_context.num_continuous_states()}')

        swarm_context.SetContinuousState(state_0)
        #swarm_context.SetContinuousState(drone_1[0:6] + drone_2[0:6] + drone_3[0:6] + drone_1[6:12] + drone_2[6:12] + drone_3[6:12])


        # Inputs
        input_dim = NUM_DRONES*PROPS_PER_DRONE 
        drone_name = GROUP_PREFIX + MODEL_PREFIX_DRONE + str(1)
        load_name = GROUP_PREFIX + "load"
        
        first_drone_instance = swarm_sys.GetModelInstanceByName(drone_name)
        load_instance = swarm_sys.GetModelInstanceByName(load_name)

        single_drone_mass = swarm_sys.CalcTotalMass(swarm_context, [first_drone_instance])
        load_mass = swarm_sys.CalcTotalMass(swarm_context, [load_instance])
        g = swarm_sys.gravity_field().kDefaultStrength

        # print(f'Drone mass: {single_drone_mass}')
        # print(f'Load mass: {load_mass}')

        diagram_plant.get_input_port().FixValue(diagram_context, (single_drone_mass+load_mass) * g / 4. * np.array([1, 1, 1, 1]))
        #diagram_plant.get_input_port().FixValue(diagram_context, single_drone_mass * g / 4. * np.array([1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1])) #.FixValue(diagram_context, single_drone_mass * g / 4. * np.ones(input_dim)) # TODO: U0 Different for when carrying load probably


        #g = drone_sys.gravity_field().kDefaultStrength
        # for i in range(FIRST_DRONE_NUM,NUM_DRONES+1):
        #     # Get drone instance
        #     drone_name = GROUP_PREFIX + MODEL_PREFIX_DRONE + str(i)
        #     drone_instance = drone_sys.GetModelInstanceByName(drone_name)

        #     # Set input port linearization
        #     drone_mass = drone_sys.CalcTotalMass(drone_context, [drone_instance])
        #     diagram_plant.get_input_port(i-1).FixValue(diagram_context, drone_mass * g / 4. * np.array([1, 1, 1, 1])) # TODO: U0 Different for when carrying load probably


        # LOAD: load_x, load_y, load_z, load_YAW, load_PITCH, load_ROLL, 
        # LOAD-side joint: roll_around_tether_axis (+ve out), ang_around_y_axis, ang_around_z_axis, 
        # DRONE-side joint: ang_around_x, ang_y, ang_z 
        # ^^ Velocities for the above?? Hopefully same order

        ## Other parameters
        Q_diag = [10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1] #[10, 10, 10, 10, 10, 10, 1, 1, 1, 1, 1, 1]
        Q_comb = np.diag(Q_diag)
        #Q_comb = np.diag(Q_diag[0:6] + Q_diag[0:6] + Q_diag[0:6] + Q_diag[6:12] + Q_diag[6:12] + Q_diag[6:12])
        #Q_alt = np.diag([10, 10, 10, 10, 10, 10, 1, 1, 1, 1, 1, 1, 10, 10, 10, 10, 10, 10, 1, 1, 1, 1, 1, 1, 10, 10, 10, 10, 10, 10, 1, 1, 1, 1, 1, 1])

        R_diag = [0.1, 0.1, 0.1, 0.1]
        R_comb = np.diag(R_diag)
        #R_comb = np.diag(R_diag + R_diag + R_diag)

        #print(R_diag + R_diag + R_diag)
        #R_alt = np.diag([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
        #A, B, C, D = Linearize(diagram_plant, diagram_context)
        #print(diagram_plant.num_inputs())
 
        # Perhaps try LMPC: https://drake.mit.edu/doxygen_cxx/classdrake_1_1systems_1_1controllers_1_1_linear_model_predictive_controller.html
        # or finiteHorizonLQR: https://drake.mit.edu/doxygen_cxx/structdrake_1_1systems_1_1controllers_1_1_finite_horizon_linear_quadratic_regulator_options.html

 
        # TODO: Look at R_comb size -> appears to only be doing LQR for single input!!! Have to do for one drone at a time???

        # HOW TO SPLIT controller K into individual drones??
        # What do with tether and load states for linearising??

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
    sdf_path = 'sdf_models/worlds/default_commented.sdf'
    diagram_quad = MakeMultibodyQuadrotor(sdf_path, meshcat)

    # Show diagram
    #utils.show_diagram(diagram_quad)

    # Make controller
    diagram_full = MakeQuadrotorController(diagram_quad)

    # Show diagram
    #utils.show_diagram(diagram_full)

    # Simulate
    #state_init = 0.5*np.random.randn(12,)

    drone_1 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    # drone_2 = [2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    # drone_3 = [0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    load = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # LOAD: load_x, load_y, load_z, load_YAW, load_PITCH, load_ROLL, 
    # LOAD-side joint: roll_around_tether_axis (+ve out), ang_around_y_axis, ang_around_z_axis, 
    # DRONE-side joint: ang_around_x, ang_y, ang_z 
    # ^^ Velocities for the above?? Hopefully same order
    state_init_test = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                       0.0, -np.pi/2, 0.0, 
                       0.0, np.pi/2, 0.0, 
                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    #state_init = drone_1
    #state_init = drone_1[0:6] + drone_2[0:6] + drone_3[0:6] + drone_1[6:12] + drone_2[6:12] + drone_3[6:12]
     
    state_init = state_init_test #load + drone_1

    utils.simulate_diagram(diagram_quad, state_init, meshcat, realtime_rate=0.75)
    #utils.simulate_diagram(diagram_full, None, meshcat, realtime_rate=0.75)


if __name__ == "__main__":
    main()