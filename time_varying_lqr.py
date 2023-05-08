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
    MathematicalProgram,
    FiniteHorizonLinearQuadraticRegulator,
    FiniteHorizonLinearQuadraticRegulatorOptions,
    MakeFiniteHorizonLinearQuadraticRegulator)

from underactuated.scenarios import AddFloatingRpyJoint

NAME_DRONE = "drone"
NAME_PROPS = "propellers"
NAME_DIAGRAM_QUAD = "quad_diagram"
NAME_DIAGRAM_WITH_CONTROLLER = "quad_with_controller"


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
    plant.set_name(NAME_DRONE)

    parser = Parser(plant)
    model_instance = parser.AddModelFromFile(sdf_path)

    # By default multibody has a quaternion floating base, so we manually add a floating roll pitch yaw joint to match QuadrotorPlant
    #  We set `use_ball_rpy` to false because the BallRpyJoint uses angular velocities instead of ṙ, ṗ, ẏ.
    AddFloatingRpyJoint(plant, plant.GetFrameByName("base_link"), model_instance, use_ball_rpy=False)
    plant.Finalize()

    ## Add propellers
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

    ## Connect diagram
    propellers = builder.AddSystem(Propeller(prop_info))
    propellers.set_name(NAME_PROPS)

    builder.Connect(propellers.get_output_port(), plant.get_applied_spatial_force_input_port(),)
    builder.Connect(plant.get_body_poses_output_port(), propellers.get_body_poses_input_port(),)
    builder.ExportInput(propellers.get_command_input_port(), "u")
    builder.ExportOutput(plant.get_state_output_port(model_instance), "x500_0_x") # Not clarifying model_instance takes state from full plant

    meshcat.Delete()
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)

    ## Build diagramt
    diagram = builder.Build()
    diagram.set_name(NAME_DIAGRAM_QUAD)

    return diagram

# Make a finite_horizon LQR controller for state regulation of the plant given in the input diagram
def MakeQuadrotorController(diagram_plant, x_traj, u_traj):
    def QuadrotorFiniteHorizonLQR(diagram_plant, options):
        # Create contexts
        diagram_context = diagram_plant.CreateDefaultContext()  

        # Q and R matrices
        Q = np.diag([10, 10, 10, 10, 10, 10, 1, 1, 1, 1, 1, 1])
        R = np.diag([0.1, 0.1, 0.1, 0.1])
 
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

        ## Other parameters
        Q = np.diag([10, 10, 10, 10, 10, 10, 1, 1, 1, 1, 1, 1])
        R = np.diag([0.1, 0.1, 0.1, 0.1])
 
        # Perhaps try LMPC: https://drake.mit.edu/doxygen_cxx/classdrake_1_1systems_1_1controllers_1_1_linear_model_predictive_controller.html
        # or finiteHorizonLQR: https://drake.mit.edu/doxygen_cxx/structdrake_1_1systems_1_1controllers_1_1_finite_horizon_linear_quadratic_regulator_options.html

        return LinearQuadraticRegulator(A, B, Q, R)

    # Get Qf from infinite horizon LQR controller
    (K, S) = QuadrotorLQR(diagram_plant)
    
    # Set options
    options = FiniteHorizonLinearQuadraticRegulatorOptions()
    options.x0 = x_traj
    options.u0 = u_traj
    options.Qf = 2*S

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

    # Create constraints on trajectory here
    prog = dircol.prog()

    dircol.AddEqualTimeIntervalsConstraints()

    lift_force_limit = 10.0
    u = dircol.input()
    for k in range(np.size(u)):
        dircol.AddConstraintToAllKnotPoints(-lift_force_limit <= u[k])
        dircol.AddConstraintToAllKnotPoints(u[k] <= lift_force_limit)

    initial_state = np.zeros(12,) # 0.5*np.random.randn(12,)
    prog.AddBoundingBoxConstraint(initial_state, initial_state, dircol.initial_state())

    final_state = np.asarray([2.0, 2.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    prog.AddBoundingBoxConstraint(final_state, final_state, dircol.final_state())

    # Cost functions on control effort and time duration
    R = 10
    dircol.AddRunningCost(R*(u[0]**2 + u[1]**2 + u[2]**2 + u[3]**2))

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
    xyz_values = x_values[0:3, :]
    rpy_values = x_values[3:6, :]
    vxyz_values = x_values[6:9, :]
    vrpy_values = x_values[9:12, :]

    # fig, ax = plt.subplots(3, 1)
    # ax[0].plot(times, np.transpose(u_values), label=["Rotor 1", "Rotor 2", "Rotor 3", "Rotor 4"])
    # ax[0].set_ylabel("Lift Force (kN?)")
    # ax[0].legend()

    # ax[1].plot(times, np.transpose(xyz_values), label=["x", "y", "z"])
    # ax[1].set_ylabel("Position (m)")
    # ax[1].legend()

    # ax[2].plot(times, np.transpose(vxyz_values), label=["vx", "vy", "vz"])
    # ax[2].set_ylabel("Velocity (m/s)")
    # ax[2].legend()

    # ax[2].set_xlabel("Time(s)")
    # plt.show()

    return x_traj, u_traj


def main():
    # Start the visualizer (run this cell only once, each instance consumes a port)
    meshcat = StartMeshcat()
    # Make Quadrotor
    sdf_path = 'sdf_models/models/x500/model.sdf'
    #sdf_path = 'sdf_models/worlds/default.sdf'
    diagram_quad = MakeMultibodyQuadrotor(sdf_path, meshcat)

    # Generate example state and input trajectories
    x_trajectory, u_trajectory = GenerateDirColTrajectory(diagram_quad)
    
    # Make controller
    diagram_full = MakeQuadrotorController(diagram_quad, x_trajectory, u_trajectory)

    # Show diagram
    # utils.show_diagram(diagram_full)

    # Simulate
    state_init = np.zeros(12,)
    utils.simulate_diagram(diagram_full, state_init, meshcat, realtime_rate=0.75)


if __name__ == "__main__":
    main()      