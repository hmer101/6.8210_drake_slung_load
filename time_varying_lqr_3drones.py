import numpy as np
import utils

from matplotlib import pyplot as plt

from pydrake.all import(
    AddMultibodyPlantSceneGraph, 
    DiagramBuilder,
    LinearQuadraticRegulator,    
    MeshcatVisualizer, 
    Parser, 
    StartMeshcat,
    Propeller,
    PropellerInfo,
    RigidTransform,
    Linearize,
    DirectCollocation,
    PiecewisePolynomial,
    Solve,
    Demultiplexer,
    Multiplexer,
    ExternallyAppliedSpatialForceMultiplexer,
    Simulator,
    LogVectorOutput,
    ModelVisualizer,
    MathematicalProgram,
    FiniteHorizonLinearQuadraticRegulator,
    FiniteHorizonLinearQuadraticRegulatorOptions,
    MakeFiniteHorizonLinearQuadraticRegulator)

from underactuated import running_as_notebook
from underactuated.scenarios import AddFloatingRpyJoint

NAME_SWARM = "swarm"
NAME_PROPS = "propellers"
NAME_DIAGRAM_QUAD = "quads_diagram"
NAME_DIAGRAM_WITH_CONTROLLER = "quads_with_controller"

GROUP_PREFIX = "swarm::"
MODEL_PREFIX_DRONE = "x500_"
NUM_DRONES = 3
FIRST_DRONE_NUM = 1
PROPS_PER_DRONE = 4


# HOW THE x500 model SDF FILE WAS CHANGED
# 1. commented out the use_parent_model_frame for all joints
# 2. wrote out whole path i.e. /home/bilab/6.8210_project/sdf_models/models/x500/meshes/1345_prop_ccw.stl
# vs model://x500/meshes/1345_prop_ccw    dircol.AddDurationBounds(1.0, 10.0).stl
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
    model_instance = parser.AddModels(sdf_path)

    for i in range(FIRST_DRONE_NUM,NUM_DRONES+1):
        next_drone_name = GROUP_PREFIX + MODEL_PREFIX_DRONE + str(i)
        next_drone_instance = plant.GetModelInstanceByName(next_drone_name)
        print(next_drone_name)

        # By default multibody has a quaternion floating base, so we manually add a floating roll pitch yaw joint to match QuadrotorPlant
        # We set `use_ball_rpy` to false because the BallRpyJoint uses angular velocities instead of ṙ, ṗ, ẏ.
        AddFloatingRpyJoint(plant, plant.GetFrameByName("base_link", next_drone_instance), next_drone_instance, use_ball_rpy=False)
    
    # load_instance = plant.GetModelInstanceByName("swarm::load")
    # AddFloatingRpyJoint(plant, plant.GetFrameByName("base_link", load_instance), load_instance, use_ball_rpy=False)

    # The current model instances are 'DefaultModelInstance', 'WorldModelInstance', 
    # 'swarm', 'swarm::load', 'swarm::tether_1', 'swarm::tether_2', 'swarm::tether_3', 'swarm::x500_1', 'swarm::x500_2', 'swarm::x500_3'

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

    ## Setup variables for co-ordinate changing and propeller connection 
    # Default parameters from quadrotor_plant.cc:
    L = 0.174  # Length for arms (m)
    kF = 1.0  # Force input constant.
    kM = 0.0245  # Moment input constant.

    ## Add propellers
    for i in range(FIRST_DRONE_NUM,NUM_DRONES+1):
        next_drone_name = GROUP_PREFIX + MODEL_PREFIX_DRONE + str(i)
        next_drone_instance = plant.GetModelInstanceByName(next_drone_name)

        body_index = plant.GetBodyByName("base_link", next_drone_instance).index()

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

        builder.Connect(plant.get_state_output_port(next_drone_instance), output_mux.get_input_port(i-1))

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

    # Logger block to measured data
    logger = LogVectorOutput(output_mux.get_output_port(), builder)
    logger.set_name("logger")

    ## Build diagram
    diagram = builder.Build()
    diagram.set_name(NAME_DIAGRAM_QUAD)

    return diagram, logger

# Make a finite_horizon LQR controller for state regulation of the plant given in the input diagram
def MakeQuadrotorController(diagram_plant, x_traj, u_traj):
    def QuadrotorFiniteHorizonLQR(diagram_plant, options):
        # Create contexts
        diagram_context = diagram_plant.CreateDefaultContext()  

        # Q and R matrices
        Q = np.diag([0, 0, 0, 0.1, 0.1, 0.1, 
                    0, 0, 0, 0.1, 0.1, 0.1,  
                    0, 0, 0, 0.1, 0.1, 0.1,
                    0, 0, 0, 10, 10, 10,
                    0, 0, 0, 10, 10, 10,
                    0, 0, 0, 10, 10, 10,])
        R = np.diag([1, 1, 1, 1,
                    1, 1, 1, 1,
                    1, 1, 1, 1])
 
        return MakeFiniteHorizonLinearQuadraticRegulator(
            diagram_plant, 
            diagram_context,
            t0=options.u0.start_time(),
            tf=options.u0.end_time(),
            Q=Q,
            R=R,
            options=options
        )
    
    def QuadrotorLQR(diagram_plant):
        ## Setup
        drone_sys = diagram_plant.GetSubsystemByName(NAME_DRONE)
        prop_sys = diagram_plant.GetSubsystemByName(NAME_PROPS)
        
        # Create contexts
        diagram_context = diagram_plant.CreateDefaultContext()        
        drone_context = drone_sys.GetMyContextFromRoot(diagram_context)
        #prop_context = prop_sys.GetMyContextFromRoot(diagram_context)

        ## Set plant at linearization point
        # States
        drone_context.SetContinuousState([2.0, 2.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        # Inputs
        drone_mass = drone_sys.CalcTotalMass(drone_context)
        g = drone_sys.gravity_field().kDefaultStrength
        diagram_plant.get_input_port().FixValue(diagram_context, drone_mass * g / 4. * np.array([1, 1, 1, 1])) # TODO: U0 Different for when carrying load probably

        # Linearize and get A and B matrices for LQR controller
        input_i = diagram_plant.get_input_port().get_index()
        output_i = diagram_plant.get_output_port().get_index()
        drone_lin = Linearize(diagram_plant, diagram_context, input_port_index=input_i, output_port_index=output_i)
        
        A = drone_lin.A()
        B = drone_lin.B()

        return LinearQuadraticRegulator(A, B, Q, R)

    # Get Qf from infinite horizon LQR controller
    # (K, S) = QuadrotorLQR(diagram_plant)
    
    # Set options
    options = FiniteHorizonLinearQuadraticRegulatorOptions()
    options.x0 = x_traj
    options.u0 = u_traj
    # options.Qf = 2*S

    lqr_finite_horizon_controller = QuadrotorFiniteHorizonLQR(diagram_plant, options)

    ## Build diagram with plant and controller
    builder = DiagramBuilder()

    # Add systems
    plant = builder.AddSystem(diagram_plant)
    plant.set_name("Drone with props")

    controller = builder.AddSystem(lqr_finite_horizon_controller)
    controller.set_name("x500_0 controller")

    # Connect diagram
    builder.Connect(controller.get_output_port(0), plant.get_input_port(0))
    builder.Connect(plant.get_output_port(0), controller.get_input_port(0))

    # Build diagram
    diagram = utils.build_diagram(builder, "with controller test")

    return diagram 


# Generates trajectories using direct collocation
# Returns trajectory objects
def GenerateDirColTrajectory(diagram_plant):
    diagram_context = diagram_plant.CreateDefaultContext() 

    dircol = DirectCollocation(
        diagram_plant,
        diagram_context,
        num_time_samples=21,
        minimum_timestep=0.05,
        maximum_timestep=0.2
    )

    # Create constraints on trajectory here    # state_init = np.zeros(210,)
    # utils.simulate_diagram(diagram_full, state_init, meshcat, realtime_rate=0.75)
    prog = dircol.prog()

    dircol.AddEqualTimeIntervalsConstraints()

    lift_force_limit = 10.0
    u = dircol.input()
    for k in range(np.size(u)):
        dircol.AddConstraintToAllKnotPoints(-lift_force_limit <= u[k])
        dircol.AddConstraintToAllKnotPoints(u[k] <= lift_force_limit)

    initial_state = np.asarray([2.0, 0.0, 0.0, 0.0, 0.0, 0.0]+
                            [-2.0, 2.0, 0.0, 0.0, 0.0, 0.0]+
                            [-2.0, -2.0, 0.0, 0.0, 0.0, 0.0]+
                            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]+
                            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]+
                            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    prog.AddBoundingBoxConstraint(initial_state, initial_state, dircol.initial_state())

    final_state = np.asarray([2.0, 0.0, 2.0, 0.0, 0.0, 0.0]+
                            [-2.0, 2.0, 2.0, 0.0, 0.0, 0.0]+
                            [-2.0, -2.0, 2.0, 0.0, 0.0, 0.0]+
                            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]+
                            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]+
                            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    prog.AddBoundingBoxConstraint(final_state, final_state, dircol.final_state())

    # Cost functions on control effort and time duration
    R = 10
    dircol.AddRunningCost(R*(u[0]**2 + u[1]**2 + u[2]**2 + u[3]**2 + u[4]**2 + u[5]**2 + u[6]**2 + u[7]**2 + u[8]**2 + u[9]**2 + u[10]**2 + u[11]**2))

    dircol.AddFinalCost(dircol.time()) 

    # Define initial trajectory
    initial_trajectory = PiecewisePolynomial.FirstOrderHold([0.0, 4.0], np.column_stack((initial_state, final_state)))
    dircol.SetInitialTrajectory(PiecewisePolynomial(), initial_trajectory)

    # Solve for trajectory
    result = Solve(prog)
    assert result.is_success()

    # Extract trajectory information
    x_traj = dircol.ReconstructStateTrajectory(result)
    u_traj = dircol.ReconstructInputTrajectory(result)

    # Uncomment block below to visualize
    times = np.linspace(u_traj.start_time(), u_traj.end_time(), 100)
    u_values = u_traj.vector_values(times)
    x_values = x_traj.vector_values(times)
    xyz_values1 = x_values[0:3, :]
    xyz_values2 = x_values[6:9, :]
    xyz_values3 = x_values[12:15, :]
    rpy_values = x_values[3:6, :]
    # vxyz_values = x_values[6:9, :]
    # vrpy_values = x_values[9:12, :]

    # fig, ax = plt.subplots(3, 1)
    # ax[0].plot(times, np.transpose(u_values)) #  label=["x", "y", "z"])
    # ax[0].set_ylabel("Position (m)")
    # ax[0].legend()

    # ax[1].plot(times, np.transpose(xyz_values2), label=["x", "y", "z"])
    # ax[1].set_ylabel("Position (m)")
    # ax[1].legend()

    # ax[2].plot(times, np.transpose(xyz_values3), label=["x", "y", "z"])
    # ax[2].set_ylabel("Position (m)")
    # ax[2].legend()

    # ax[2].set_xlabel("Time(s)")
    # plt.show()

    return x_traj, u_traj


def main():
    # Start the visualizer (run this cell only once, each instance consumes a port)
    meshcat = StartMeshcat()
    # Make Quadrotor
    # sdf_path = 'sdf_models/models/x500/model.sdf'
    sdf_path = 'sdf_models/worlds/default_kintreetest.sdf'
    
    print("Creating multibody system")
    diagram_quad, logger = MakeMultibodyQuadrotor(sdf_path, meshcat)

    # diagram_context = diagram_quad.CreateDefaultContext()
    # swarm_sys = diagram_quad.GetSubsystemByName(NAME_SWARM)
    # swarm_context = swarm_sys.GetMyContextFromRoot(diagram_context)

    # print(swarm_context.num_total_states())
    # print(swarm_context.num_continuous_states())

    # Generate example state and input trajectories
    print("Generating trajectory")
    x_trajectory, u_trajectory = GenerateDirColTrajectory(diagram_quad)     
    # print("done")

    # Make controller
    print("Creating controller")
    diagram_full = MakeQuadrotorController(diagram_quad, x_trajectory, u_trajectory)

    # # Show diagram
    # utils.show_diagram(diagram_full)

    # Simulate
    # state_init = np.zeros(36,)
    state_init = np.asarray([2.0, 0.0, 0.0, 0.0, 0.0, 0.0]+
                            [-2.0, 2.0, 0.0, 0.0, 0.0, 0.0]+
                            [-2.0, -2.0, 0.0, 0.0, 0.0, 0.0]+
                            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]+
                            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]+
                            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    utils.simulate_diagram(diagram_full, state_init, meshcat, realtime_rate=0.75)

    # simulator = Simulator(diagram_full)
    # print("here0")
    # context = simulator.get_mutable_context()
    # context.SetTime(0.)
    # context.SetContinuousState(state_init)
    # simulator.AdvanceTo(100)
    # print("here1")

    # logger = diagram_full.log
    # logger.FindLog(context).data
    # print("here2")

    # print('1')
    # visualizer = ModelVisualizer(meshcat=meshcat)
    # print("2")
    # visualizer.parser().AddModels(sdf_path)
    # print("3")
    # visualizer.Run(loop_once=not running_as_notebook)

    # while True:
    #     pass

    # meshcat.Delete()
    # meshcat.DeleteAddedControls()

if __name__ == "__main__":
    main()      