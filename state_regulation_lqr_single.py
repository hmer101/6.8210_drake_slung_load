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
    RigidTransform)

from underactuated.scenarios import AddFloatingRpyJoint

NAME_DRONE = "drone"
NAME_PROPS = "propellers"
NAME_DIAGRAM_QUAD = "quad_diagram"
NAME_DIAGRAM_WITH_CONTROLLER = "quad_with_controller"


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
        #PropellerInfo(body_index, RigidTransform([0, 0, 0]), kF, 0),

        # PropellerInfo(body_index, RigidTransform([L, -L, 0]), kF, 0), # rotor 0
        # PropellerInfo(body_index, RigidTransform([-L, L, 0]), kF, 0), # rotor 1
        # PropellerInfo(body_index, RigidTransform([L, L, 0]), kF, 0), # rotor 2 cw
        # PropellerInfo(body_index, RigidTransform([-L, -L, 0]), kF, 0), # rotor 3 cw

        PropellerInfo(body_index, RigidTransform([L, -L, 0.06]), kF, kM), # rotor 0
        PropellerInfo(body_index, RigidTransform([-L, L, 0.06]), kF, kM), # rotor 1
        PropellerInfo(body_index, RigidTransform([L, L, 0.06]), kF, -kM), # rotor 2 cw
        PropellerInfo(body_index, RigidTransform([-L, -L, 0.06]), kF, -kM), # rotor 3 cw
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

    ## Build diagram
    diagram = builder.Build()
    diagram.set_name(NAME_DIAGRAM_QUAD)

    return diagram


# Make an LQR controller for state regulation of the plant given in the input diagram
def MakeQuadrotorController(diagram_plant):
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
        # For RPY: SetContinuousState([X, Y, Z, YAW, PITCH?, ROLL?, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        drone_context.SetContinuousState([0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        # For quaternion: #x = [qw, qx, qy, qz, x, y, z, ???? from here ???? vx, vy, vz, wx, wy, wz]. Must satisfy: qw^2 + qx^2 + qy^2 + qz^2 = 1
        #drone_context.SetContinuousState([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        #drone_context.SetContinuousState([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        # Inputs
        drone_mass = drone_sys.CalcTotalMass(drone_context)
        g = drone_sys.gravity_field().kDefaultStrength
        #diagram_plant.get_input_port().FixValue(diagram_context, drone_mass * g / 4. * np.array([1]))
        diagram_plant.get_input_port().FixValue(diagram_context, drone_mass * g / 4. * np.array([1, 1, 1, 1])) # TODO: U0 Different for when carrying load probably

        # # Linearize and get A and B matrices for LQR controller
        # input_i = diagram_plant.get_input_port().get_index()
        # output_i = diagram_plant.get_output_port().get_index()
        # drone_lin = Linearize(diagram_plant, diagram_context, input_port_index=input_i, output_port_index=output_i)
        
        # A = drone_lin.A()
        # B = drone_lin.B()

        # print(A)
        # print(np.any(A))
        # print(np.shape(A))
        # print(B)
        # print(np.shape(B))

        ## Other parameters
        Q = np.diag([10, 10, 10, 10, 10, 10, 1, 1, 1, 1, 1, 1])
        
        #Q = np.diag([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]) #np.diag([10, 10, 10, 10, 10, 10, 10, 1, 1, 1, 1, 1, 1])
        R = np.diag([0.1, 0.1, 0.1, 0.1]) #np.diag([0.1, 0.1, 0.1, 0.1])
 
        # Perhaps try LMPC: https://drake.mit.edu/doxygen_cxx/classdrake_1_1systems_1_1controllers_1_1_linear_model_predictive_controller.html
        # or finiteHorizonLQR: https://drake.mit.edu/doxygen_cxx/structdrake_1_1systems_1_1controllers_1_1_finite_horizon_linear_quadratic_regulator_options.html

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
    diagram = utils.build_diagram(builder, "Drone with controller")

    return diagram 


def main():
    # Start the visualizer (run this cell only once, each instance consumes a port)
    meshcat = StartMeshcat()

    # Make Quadrotor
    sdf_path = 'sdf_models/models/x500/model.sdf'
    #sdf_path = 'sdf_models/worlds/default.sdf'
    diagram_quad = MakeMultibodyQuadrotor(sdf_path, meshcat)

    # Make controller
    diagram_full = MakeQuadrotorController(diagram_quad)

    # Show diagram
    #utils.show_diagram(diagram_full)

    # Simulate 
    state_init = np.array([0.0, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    #state_init = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) #0.5*np.random.randn(13,) #np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) #np.array([0.0, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) #0.5*np.random.randn(12,)
    utils.simulate_diagram(diagram_full, state_init, meshcat, realtime_rate=0.75)


if __name__ == "__main__":
    main()