import numpy as np
import utils
import math

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
MODEL_PREFIX_TETHER = "tether_"
NUM_DRONES = 3 
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

    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-4) #time_step=1e-4) #plant = builder.AddSystem(MultibodyPlant(0.0))
    plant.set_name(NAME_SWARM)

    parser = Parser(plant)
    #model_instance = parser.AddModelFromFile(sdf_path)
    model_instance = parser.AddModels(sdf_path)

    ## Setup variables for co-ordinate changing and propeller connection 
    # Default parameters from quadrotor_plant.cc:
    L = 0.174  # Length for arms (m)
    kF = 1.0  # Force input constant.
    kM = 0.0245  # Moment input constant.

    # Change frame for load
    load_name = GROUP_PREFIX + "load"
    load_instance = plant.GetModelInstanceByName(load_name)
    AddFloatingRpyJoint(plant, plant.GetFrameByName("base_link", load_instance), load_instance, use_ball_rpy=False)

    plant.Finalize()

    ## Add propellers
    prop_info = []
    for i in range(FIRST_DRONE_NUM,NUM_DRONES+1):
        next_drone_name = GROUP_PREFIX + MODEL_PREFIX_DRONE + str(i)
        next_drone_instance = plant.GetModelInstanceByName(next_drone_name)

        body_index = plant.GetBodyByName("base_link", next_drone_instance).index()

        # Note: Rotors 0 and 1 rotate one way and rotors 2 and 3 rotate the other.
        prop_info += [
            # PropellerInfo(body_index, RigidTransform([L, -L, 0]), kF, kM),
            PropellerInfo(body_index, RigidTransform([L, -L, 0.06]), kF, kM), # rotor 0
            PropellerInfo(body_index, RigidTransform([-L, L, 0.06]), kF, kM), # rotor 1
            PropellerInfo(body_index, RigidTransform([L, L, 0.06]), kF, -kM), # rotor 2 cw
            PropellerInfo(body_index, RigidTransform([-L, -L, 0.06]), kF, -kM), # rotor 3 cw
        ]


    propellers = builder.AddNamedSystem("propellers", Propeller(prop_info))

    builder.Connect(propellers.get_output_port(), plant.get_applied_spatial_force_input_port())
    builder.Connect(plant.get_body_poses_output_port(), propellers.get_body_poses_input_port())


    builder.ExportInput(propellers.get_command_input_port(), "u")
    builder.ExportOutput(plant.get_state_output_port(), "q")

    # Add meshcat visualizer
    meshcat.Delete()
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)

    ## Build diagram
    diagram = builder.Build()
    diagram.set_name(NAME_DIAGRAM_QUAD)

    return diagram

# Make an LQR controller for state regulation of the plant given in the input diagram
def MakeQuadrotorController(diagram_plant, meshcat):
    def QuadrotorLQR(diagram_plant, meshcat):
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
        # drone_1 = [0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # drone_2 = [2.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # drone_3 = [0.0, 2.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # load = [0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # state_0 = [0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 
        #                0.0, -np.pi/2, 0.0, 
        #                0.0, np.pi/2, 0.0, 
        #                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        
        #swarm_context.SetContinuousState(drone_1[0:6] + drone_2[0:6] + drone_3[0:6] + drone_1[6:12] + drone_2[6:12] + drone_3[6:12])


        ## Set plant at linearization point
        input_dim = NUM_DRONES*PROPS_PER_DRONE 
        drone_name = GROUP_PREFIX + MODEL_PREFIX_DRONE + str(1)
        load_name = GROUP_PREFIX + "load"
        cable_name = GROUP_PREFIX + MODEL_PREFIX_TETHER + str(1)
        
        first_drone_instance = swarm_sys.GetModelInstanceByName(drone_name)
        load_instance = swarm_sys.GetModelInstanceByName(load_name)
        first_cable_instance = swarm_sys.GetModelInstanceByName(cable_name)

        single_drone_mass = swarm_sys.CalcTotalMass(swarm_context, [first_drone_instance])
        load_mass = swarm_sys.CalcTotalMass(swarm_context, [load_instance])
        single_cable_mass = swarm_sys.CalcTotalMass(swarm_context, [first_cable_instance])

        g = swarm_sys.gravity_field().kDefaultStrength

        # print(f'Drone mass: {single_drone_mass}')
        # print(f'Load mass: {load_mass}')

        # States (Note states for tethers, load and reference link are also required)
        #get_stable_state(num_drones, num_links_per_cable, m_drone, m_load, m_cable, drone_elevation_ang,      load_pose: np.ndarray)
        x0, u0 = utils.get_stable_state(False, NUM_DRONES, 1, single_drone_mass, load_mass, single_cable_mass, math.pi/4, np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))

        # print(f'num_total: {swarm_context.num_total_states()}')
        # print(f'num_cont: {swarm_context.num_continuous_states()}')

        #swarm_context.SetContinuousState(x0)
        swarm_context.SetDiscreteState(x0)

        # Inputs
        diagram_plant.get_input_port().FixValue(diagram_context, u0) #(single_drone_mass+load_mass) * g / 4. * np.array([1, 1, 1, 1]))
        #diagram_plant.get_input_port().FixValue(diagram_context, single_drone_mass * g / 4. * np.array([1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1])) #.FixValue(diagram_context, single_drone_mass * g / 4. * np.ones(input_dim)) # TODO: U0 Different for when carrying load probably

        #utils.simulate_diagram(diagram_plant, x0, meshcat, realtime_rate=0.75)

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
        #Q_diag =  np.diag(([10.0] * (6 + 6*NUM_DRONES) + [1.0] * (6 + 6*NUM_DRONES))) #[]  [10.0] * 24 + [1.0] * 24) * n) #[10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1] #[10, 10, 10, 10, 10, 10, 1, 1, 1, 1, 1, 1]
        load_pos = [10.0] * 3
        load_orient = [1.0] * 3
        cable_load_drone = [1.0] * (6*NUM_DRONES)
        vel = [1.0] * (6 + 6*NUM_DRONES)
         
        
        Q_diag =  np.diag(load_pos + load_orient + cable_load_drone + vel)
        Q_comb = Q_diag #np.diag(Q_diag)
        #Q_comb = np.diag(Q_diag[0:6] + Q_diag[0:6] + Q_diag[0:6] + Q_diag[6:12] + Q_diag[6:12] + Q_diag[6:12])
        #Q_alt = np.diag([10, 10, 10, 10, 10, 10, 1, 1, 1, 1, 1, 1, 10, 10, 10, 10, 10, 10, 1, 1, 1, 1, 1, 1, 10, 10, 10, 10, 10, 10, 1, 1, 1, 1, 1, 1])

        R_diag = np.diag(([0.1] * (4*NUM_DRONES))) #[0.1, 0.1, 0.1, 0.1]
        R_comb = R_diag #np.diag(R_diag)
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

        # MAP VELOCITY TO Q_DOT ERROR HERE!!!!
        return LinearQuadraticRegulator(diagram_plant, diagram_context, Q_comb, R_comb)

    lqr_controller = QuadrotorLQR(diagram_plant, meshcat)

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
    sdf_path = 'sdf_models/worlds/default.sdf'
    #sdf_path = 'sdf_models/worlds/default_same.sdf'
    #sdf_path = 'sdf_models/worlds/default_commented.sdf'
    diagram_quad = MakeMultibodyQuadrotor(sdf_path, meshcat)

    # Show diagram
    #utils.show_diagram(diagram_quad)

    # Make controller
    diagram_full = MakeQuadrotorController(diagram_quad, meshcat)

    # Show diagram
    utils.show_diagram(diagram_full)

    # Simulate
    #state_init = 0.5*np.random.randn(12,)

    # drone_1 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    # # drone_2 = [2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    # # drone_3 = [0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    # load = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # LOAD: load_x, load_y, load_z, load_YAW, load_PITCH, load_ROLL, 
    # LOAD-side joint: roll_around_tether_axis (+ve out), ang_around_y_axis, ang_around_z_axis, 
    # DRONE-side joint: ang_around_x, ang_y, ang_z 
    # ^^ Velocities for the above?? Hopefully same order
    # state_init_test = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
    #                    0.0, -np.pi/2, 0.0, 
    #                    0.0, np.pi/2, 0.0, 
    #                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    
    #x0, u0 = utils.get_stable_state(3, 1, 2.0, 0.5, 0.1, math.pi/4, np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
    x0, u0 = utils.get_stable_state(True, NUM_DRONES, 1, 0, 0, 0, 0, np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))


    #state_init = drone_1
    #state_init = drone_1[0:6] + drone_2[0:6] + drone_3[0:6] + drone_1[6:12] + drone_2[6:12] + drone_3[6:12]
     
    state_init = x0 #state_init_test #load + drone_1

    #utils.simulate_diagram(diagram_quad, state_init, meshcat, realtime_rate=0.75)
    utils.simulate_diagram(diagram_full, state_init, meshcat, realtime_rate=0.75)


if __name__ == "__main__":
    main()