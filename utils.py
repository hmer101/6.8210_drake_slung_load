import time
import utils
import numpy as np
import math

import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('TkAgg') # Because I get error "Matplotlib is currently using agg, which is a non-GUI backend, so cannot show the figure.

from pydrake.all import(
    ModelVisualizer, 
    Simulator, 
    plot_system_graphviz,
    AbstractValue,
    ExternallyAppliedSpatialForce_,
    LeafSystem_,
    TemplateSystem)


# Build (and name) diagram drafted in a builder
def build_diagram(builder, name):
    diagram = builder.Build()
    diagram.set_name(name)

    return diagram

# Show a diagram using Graphviz
def show_diagram(diagram):
    # Show diagram
    # Note: had to "sudo apt install graphviz" because I got [Errno 2] "dot" not found in path
    # Added matplotlib.use('TkAgg') to block of imports above
    # Ran sudo apt-get install python3-tk to get the plots to show
    plt.figure(figsize=(20,10))
    plot_system_graphviz(diagram)
    plt.show()

# Visualize an SDF model found at the given path, using meshcat
def meshcat_visualize(sdf_path, meshcat):
    visualizer = ModelVisualizer(meshcat=meshcat)
    visualizer.parser().AddModelFromFile(sdf_path)
    visualizer.Run(loop_once=False) #not running_as_notebook

# Simulate a built diagram with meshcat, starting at a given initial state
def simulate_diagram(diagram, state_init, meshcat, realtime_rate=1.0, max_advance_time=0.1, sleep_time=0.04):
    # Set up a simulator to run diagram
    simulator = Simulator(diagram)
    simulator.set_target_realtime_rate(realtime_rate)
    context = simulator.get_mutable_context()

    context.SetTime(0.)

    # Set initial state if one is provided
    if state_init is None:
        print("Using default initial state")
    else:
        #context.SetContinuousState(state_init)
        context.SetDiscreteState(state_init)

    simulator.Initialize()

    print("Press 'Stop Simulation' in MeshCat to continue.")
    meshcat.AddButton('Stop Simulation')

    #utils.show_diagram(diagram)

    # Run simulation
    input("Press [Enter] to start simulation...")
    while meshcat.GetButtonClicks('Stop Simulation') < 1:
        #print("Before advance")


        simulator.AdvanceTo(simulator.get_context().get_time() + max_advance_time)


        #print("After advance")
        time.sleep(sleep_time)
        #print("After sleep")


# Get a stable state to linearize around for the n-drone case
# load_pos, load_orient - 6x1 np array that defines the load's pose to linearize around
# 
def get_stable_state(state_init: bool, num_drones, num_links_per_cable, m_drone, m_load, m_cable, drone_elevation_ang, load_pose: np.ndarray):
    g = 9.81 #9.80665 
    
    # Check that inputs are of the correct type and expected size
    assert isinstance(load_pose, np.ndarray), "load_pose should be a numpy array of size 6"
    assert load_pose.shape == (6,)

    ### q
    ## Load
    x0 = load_pose

    
    ## Cable joint angles
    load_z_ang_step = 2*math.pi/num_drones
    load_y_ang = 0.0
    F = 0.0
    
    if not state_init:
        load_y_ang = drone_elevation_ang

        T_load_z = (m_load*g)/num_drones
        #T_load_x = T_load_z/(math.tan(load_y_ang))

        T_drone_z = T_load_z + m_cable*g
        T_drone_x = (2*T_load_z + m_cable*g)/(2*math.tan(load_y_ang))
        

    for drone_num in range(num_drones):
        #x0 =  np.concatenate(x0, np.array([0.0, 0.0, 0.0]), axis=0)

        # Assume joints added in order of links they are on 
        ## Load side
        #x0 =  np.concatenate((x0, np.array([0.0, 0.0, 0.0])), axis=0)
        x0 =  np.concatenate((x0, np.array([0.0, -load_y_ang, drone_num*load_z_ang_step])), axis=0)

        ## Drone side 
        if not state_init:
            #F_z = T_load_z + (m_drone+m_cable)*g # TODO: Might not be correct way to add m_cable 
            #F_x = T_load_x
            #F_scalar = math.sqrt(F_x**2 + F_z**2)

            drone_phi_ang =  math.atan(T_drone_x/(T_drone_z+(m_drone*g)))
            drone_y_ang = drone_phi_ang + load_y_ang
            F = T_drone_x/math.sin(drone_phi_ang)

            #x0 =  np.concatenate((x0, np.array([0.0, 0.0, 0.0])), axis=0)
            x0 =  np.concatenate((x0, np.array([0.0, drone_y_ang, 0.0])), axis=0) #load_y_ang+math.atan(F_x/F_z)
        else:
            x0 =  np.concatenate((x0, np.array([0.0, 0.0, 0.0])), axis=0)


    ### q_dot
    ## Load
    x0 =  np.concatenate((x0, np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])), axis=0)

    ## Cable joints
    for joint_num in range(num_drones*(num_links_per_cable+1)):
        x0 =  np.concatenate((x0, np.array([0.0, 0.0, 0.0])), axis=0)


    u0 = F / 4. * np.ones(num_drones * 4) #np.array([1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1])


    return x0, u0
    
    # LOAD: load_x, load_y, load_z, load_YAW, load_PITCH, load_ROLL, 
    # LOAD-side joint: roll_around_tether_axis (+ve out), ang_around_y_axis, ang_around_z_axis, 
    # DRONE-side joint: ang_around_x, ang_y, ang_z 
    # ^^ Velocities for the above?? Hopefully same order\



# Class to create a force mux LeafSystem that combines spatial forces applied to a multibody system
# From: https://stackoverflow.com/questions/72120901/applying-propeller-and-wing-forces-to-a-multibodyplant-in-drake/72121171#72121171
@TemplateSystem.define("SpatialForceConcatinator_")
def SpatialForceConcatinator_(T):
    class Impl(LeafSystem_[T]):
        def _construct(self, N_inputs, converter = None):
            LeafSystem_[T].__init__(self, converter)
            self.N_inputs = N_inputs
            self.Input_ports = [self.DeclareAbstractInputPort(f"Spatial_Force_{i}",
                                AbstractValue.Make([ExternallyAppliedSpatialForce_[T]()]))
                                for i in range(N_inputs)]
        
            self.DeclareAbstractOutputPort("Spatial_Forces",
                                           lambda: AbstractValue.Make(                                             
                                           [ExternallyAppliedSpatialForce_[T]()
                                              for i in range(N_inputs)]),
                                           self.Concatenate)

        def Concatenate(self, context, output):
            out = []
            for port in self.Input_ports:
                out += port.Eval(context)
            output.set_value(out)
        
        def _construct_copy(self, other, converter=None,):
            Impl._construct(self, other.N_inputs, converter=converter)
    
    return Impl

