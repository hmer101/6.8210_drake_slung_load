import numpy as np
import os
import sys
import inspect

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
pparentdir = os.path.dirname(parentdir)
sys.path.insert(0, pparentdir) 
import utils
# import SingleQuadrotorTrajectory

import diff_flatness.MultiRotorTrajectory
import pickle
import os

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


global global_initial_state
global_initial_state= np.asarray([2.0, 0.0, 0.0, 0.0, 0.0, 0.0]+
                        [-2.0, 2.0, 0.0, 0.0, 0.0, 0.0]+
                        [-2.0, -2.0, 0.0, 0.0, 0.0, 0.0]+
                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]+
                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]+
                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
global global_final_state
global_final_state = np.asarray([2.0, 0.0, 2.0, 0.0, 0.0, 0.0]+
                        [-2.0, 2.0, 2.0, 0.0, 0.0, 0.0]+
                        [-2.0, -2.0, 2.0, 0.0, 0.0, 0.0]+
                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]+
                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]+
                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])


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

    plant.Finalize()

    ## Setup variables for co-ordinate changing and propeller connection 
    # Default parameters from quadrotor_plant.cc:
    L = 0.174  # Length for arms (m)
    kF = 1.0  # Force input constant.
    kM = 0.0245  # Moment input constant.

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
    
    # Connect propellers to plant
    propellers = builder.AddNamedSystem("propellers", Propeller(prop_info))

    builder.Connect(propellers.get_output_port(), plant.get_applied_spatial_force_input_port())
    builder.Connect(plant.get_body_poses_output_port(), propellers.get_body_poses_input_port())

    # Export input
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

# Make a finite_horizon LQR controller for state regulation of the plant given in the input diagram
def MakeQuadrotorController(diagram_plant, x_traj, u_traj):

    def QuadrotorFiniteHorizonLQR(diagram_plant, options):
        # Create contexts
        diagram_context = diagram_plant.CreateDefaultContext()  

        # Q and R matrices
        Q = np.diag([10, 10, 10, 10, 10, 10, 
                    10, 10, 10, 10, 10, 10, 
                    10, 10, 10, 10, 10, 10,
                    1, 1, 1, 1, 1, 1,
                    1, 1, 1, 1, 1, 1,
                    1, 1, 1, 1, 1, 1])
        R = np.diag([0.1, 0.1, 0.1, 0.1,
                    0.1, 0.1, 0.1, 0.1,
                    0.1, 0.1, 0.1, 0.1])

        return MakeFiniteHorizonLinearQuadraticRegulator(
            diagram_plant, 
            diagram_context,
            t0=options.u0.start_time(),
            tf=options.u0.end_time(),
            Q=Q,
            R=R,
            options=options
        )
    
    def QuadrotorInfiniteHorizonLQR(diagram_plant):
        ## Setup
        drone_sys = diagram_plant.GetSubsystemByName(NAME_SWARM)
        
        # Create contexts
        diagram_context = diagram_plant.CreateDefaultContext()        
        drone_context = drone_sys.GetMyContextFromRoot(diagram_context)

        ## Set plant at linearization point
        # States
        global global_final_state
        drone_context.SetContinuousState(global_final_state) # TODO maybe lol get_mutable_continuous_state_vector

        # Inputs
        input_dim = NUM_DRONES*PROPS_PER_DRONE 
        drone_name = GROUP_PREFIX + MODEL_PREFIX_DRONE + str(1)
        first_drone_instance = drone_sys.GetModelInstanceByName(drone_name)
        single_drone_mass = drone_sys.CalcTotalMass(drone_context, [first_drone_instance])
        g = drone_sys.gravity_field().kDefaultStrength

        diagram_plant.get_input_port().FixValue(diagram_context, single_drone_mass * g / 4. * np.array([1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1])) # TODO: U0 Different for when carrying load probably

        # Linearize and get A and B matrices for LQR controller
        input_i = diagram_plant.get_input_port().get_index()
        output_i = diagram_plant.get_output_port().get_index()
        drone_lin = Linearize(diagram_plant, diagram_context, input_port_index=input_i, output_port_index=output_i)
        
        A = drone_lin.A()
        B = drone_lin.B()

        # Other parameters
        Q = np.diag([10, 10, 10, 10, 10, 10, 
                    10, 10, 10, 10, 10, 10, 
                    10, 10, 10, 10, 10, 10,
                    1, 1, 1, 1, 1, 1,
                    1, 1, 1, 1, 1, 1,
                    1, 1, 1, 1, 1, 1])
        R = np.diag([0.1, 0.1, 0.1, 0.1,
                    0.1, 0.1, 0.1, 0.1,
                    0.1, 0.1, 0.1, 0.1])

        return LinearQuadraticRegulator(A, B, Q, R)


    #################################################################################################################
    # Get Qf from infinite horizon LQR controller
    inf_horizon_lqr = True
    #################################################################################################################

    if inf_horizon_lqr:
        (K, S) = QuadrotorInfiniteHorizonLQR(diagram_plant)
        print(S)

    # Set options
    options = FiniteHorizonLinearQuadraticRegulatorOptions()
    options.x0 = x_traj
    options.u0 = u_traj
    if inf_horizon_lqr:
        options.Qf = 20*S

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
def GenerateDiffFlatnessTrajectory(diagram_plant):
    #################################################################################################################
    include_global_init = False
    include_global_final = False
    #################################################################################################################
    diagram_context = diagram_plant.CreateDefaultContext() 


    ##################
    # DIFF FLATNESS
    ##################
    timesteps = 51
    dt = .1
    tf = timesteps*dt

    zpp = diff_flatness.MultiRotorTrajectory.circle_example_n_rotors(n=3, degree=6, continuity_degree=4, 
            discretization_samples=timesteps, diff_solver_samples=7, tf=tf)
    x_L,x_i, r_L,rpy_i, x_dot_i,x_dot_L, omega_i, Omega_L, u_out, Tiqi, t_array = diff_flatness.MultiRotorTrajectory.solve_for_states_n_rotors(zpp, 
                                                                                    3, tf=tf, timesteps=timesteps)
    

    trajectories_3d=[x_L[:-2,:], x_i[:-2,0:3], x_i[:-2,3:6], x_i[:-2,6:9]]
    legend=["Load","Drone 0","Drone 1","Drone 2"]
    diff_flatness.MultiRotorTrajectory.plot_trajectories_3d(trajectories_3d, legend=legend)
    diff_flatness.MultiRotorTrajectory.animate_trajectories(trajectories_3d, legend=legend)


    ###########################################################
    # Translating to states for PiecewisePolynomialTrajectory #
    ###########################################################
    intermediate_states = []
    # offset = 5 # time to get to the beginning and end states
    int_offset = 0 # intermediate offset
    # final_offset = 10
    # if include_global_init:
    #     t_out = [0]
    #     int_offset = offset
    # else:
    #     int_offset = 0
    t_out = []
    for t in range(len(x_i)):
        # x_i[t] = [ x_0, y_0, z_0, x_1, y_1, ... y_n, z_n]
        # rpy_i[t] = [ r_0, p_0, yaw_0, r_1, p_1, ... p_n, yaw_n]
        # x_i[t][3*i:3*i+3] --> [x_i, y_i, z_i] at timestep number t


        state = []

        # To make trajectories (xyz0 xyz1 .. xyzn rpy0 rpy1 ... rpyn xyzdot1 .. xyzdotn rpydot0 ... rpydotn)
        # for i in range(3):
        #     state.extend(x_i[t][3*i:3*i+3])
        # for i in range(3):
        #     state.extend(rpy_i[t][3*i:3*i+3])
        # for i in range(3):
        #     state.extend(x_dot_i[t][3*i:3*i+3])
        # for i in range(3):
        #     state.extend(omega_i[t][3*i:3*i+3])


        # To make trajectories (xyz0 rpy0 xyz1 rpy1 .. xyzn rpyn xyzdot0 rpydot0 .. xyzdotn rpydotn)
        for i in range(3):
            state.extend(x_i[t][3*i:3*i+3])
            state.extend(rpy_i[t][3*i:3*i+3])
        for i in range(3):
            state.extend(x_dot_i[t][3*i:3*i+3])
            state.extend(omega_i[t][3*i:3*i+3])


        # To make trajectories (xyz0 rpy0 xyzdot0 rpydot0 xyz1 rpy1 xyzdot1 rpydot1 ... xyzn rpyn xyzdotn rpydotn)
        # for i in range(3):
        #     state.extend(x_i[t][3*i:3*i+3])
        #     state.extend(rpy_i[t][3*i:3*i+3])
        #     state.extend(x_dot_i[t][3*i:3*i+3])
        #     state.extend(omega_i[t][3*i:3*i+3])
        

        intermediate_states.append(np.asarray(state))
        t_out.append(t_array[t]+int_offset)

    ###########################################################
    last_state = intermediate_states[-1]
    last_input = u_out[-1]
    last_state[18:] = np.zeros(18)
    intermediate_states[-1] = last_state
    # u_out.append(last_input)

    states_out = np.asarray(intermediate_states).T
    inputs_out = np.asarray(u_out).T
    t_out = np.asarray(t_out)

    # global global_final_state
    # global global_initial_state


    # if include_global_init:
    #     states_out = np.column_stack((global_initial_state, states_out))
    #     initial_state = global_initial_state
    # else:
    #     initial_state = intermediate_states[0]
    #     global_initial_state = initial_state


    # if include_global_final:
    #     newtf = t_out[len(t_out)-1]+final_offset
    #     t_out.append(newtf)
    #     # OLD_final_state = final_state
    #     # NEW_final_state = global_final_state
    #     final_state = global_final_state
    #     states_out = np.column_stack((states_out, global_final_state))
    # else:
    #     final_state = intermediate_states[-1]
    #     global_final_state = final_state

    # assert len(t_out) == len(states_out[0]), "time and state dimension mismatch"
    # prog.AddBoundingBoxConstraint(initial_state, initial_state, dircol.initial_state())
    # prog.AddBoundingBoxConstraint(final_state, final_state, dircol.final_state())
    x_trajectory = PiecewisePolynomial.FirstOrderHold(t_out, states_out)
    u_trajectory = PiecewisePolynomial.FirstOrderHold(t_out[0:-1], inputs_out)
    # dircol.SetInitialTrajectory(PiecewisePolynomial(), initial_trajectory)



    global global_initial_state
    global global_final_state
    print(f"states_out.shape: {states_out.shape}")
    print(f"states_out[:,-1]: {states_out[:,-1]}")
    print(f"states_out[:,0]: {states_out[:,0]}")
    print(f"states_out[0,:]: {states_out[0:]}")
    global_initial_state = states_out[:,0]
    global_final_state = states_out[:,-1]


    # ###################
    # # SOLVER CACHEING #
    # ###################
    # cache_file = "result_time_varying_lqr_3drones_cache.npy"
    # # Solve for trajectory
    # if (os.path.exists(cache_file)):
    #     print(f" loading initial guess from file {cache_file}")
    #     with open(cache_file, 'rb') as f:
    #         data = pickle.load(f)
    #     # prog.SetInitialGuessForAllVariables(data)
    # else:
    #     print(f"Solving trajectory optimization problem...")
    # result = Solve(prog)
    # assert result.is_success()
    # pickle.dump( result.GetSolution(), open( cache_file, "wb" ) )


    # Extract trajectory information
    # x_traj = initial_trajectory.ReconstructStateTrajectory(result)
    # u_traj = initial_trajectory.ReconstructInputTrajectory(result)
    return x_trajectory, u_trajectory, t_out





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

    initial_state = np.asarray([2.0, 0.0, 0.0, 0.0, 0.0, 0.0]+
                            [-2.0, 2.0, 0.0, 0.0, 0.0, 0.0]+
                            [-2.0, -2.0, 0.0, 0.0, 0.0, 0.0]+
                            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]+
                            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]+
                            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    prog.AddBoundingBoxConstraint(initial_state, initial_state, dircol.initial_state())

    final_state = np.asarray([2.0, 2.0, 3.0, 0.0, 0.0, 0.0]+
                            [-1.0, 1.0, 2.0, 0.0, 0.0, 0.0]+
                            [-2.5, -2.5, 1.5, 0.0, 0.0, 0.0]+
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



    cache_file = "result_time_varying_lqr_circle_3drones.npy"
    # Solve for trajectory
    if (os.path.exists(cache_file)):
        print(f" loading initial guess from file {cache_file}")
        with open(cache_file, 'rb') as f:
            data = pickle.load(f)
        prog.SetInitialGuessForAllVariables(data)
    result = Solve(prog)
    assert result.is_success()
    pickle.dump( result.GetSolution(), open( cache_file, "wb" ) )


    # Extract trajectory information
    x_traj = dircol.ReconstructStateTrajectory(result)
    u_traj = dircol.ReconstructInputTrajectory(result)


    return x_traj, u_traj





def main():
    # Start the visualizer (run this cell only once, each instance consumes a port)
    meshcat = StartMeshcat()
    # Make Quadrotor
    # sdf_path = 'sdf_models/models/x500/model.sdf'
    #sdf_path = 'sdf_models/worlds/default_kintreetest.sdf'
    sdf_path = 'sdf_models/worlds/default_actual.sdf'
    sdf_path = 'sdf_models/worlds/default_3drones_Kevin.sdf'

    print("Creating multibody system")
    diagram_quad, logger = MakeMultibodyQuadrotor(sdf_path, meshcat)

    # Generate example state and input trajectories
    x_trajectory, u_trajectory, t_out =  GenerateDiffFlatnessTrajectory(diagram_quad)
    # x_trajectory, u_trajectory, t_out =  GenerateDirColTrajectory(diagram_quad)  
        
        # pickle.dump( result.GetSolution(), open( cache_file, "wb" ) )   
        # pickle.dump( x_trajectory, open( "x_traj_time_varying_lqr_circle_3drones.npy", "wb" ) )
        # pickle.dump( u_trajectory, open( "u_traj_time_varying_lqr_circle_3drones.npy", "wb" ) )
    # print("done")

    # Make controller
    print("Creating controller")
    diagram_full = MakeQuadrotorController(diagram_quad, x_trajectory, u_trajectory)

    # # Show diagram
    utils.show_diagram(diagram_full)

    # Simulate
    # state_init = np.zeros(36,)
    global global_initial_state
    global global_final_state
    state_init = global_initial_state

    print(f"Simulation should last {t_out[-1]} seconds")
    utils.simulate_diagram(diagram_full, state_init, meshcat, realtime_rate=1.0, sleep_time=0.1)

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


