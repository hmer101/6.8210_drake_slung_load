import time
import utils

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
        context.SetContinuousState(state_init)
        #context.SetDiscreteState(state_init)

    simulator.Initialize()

    print("Press 'Stop Simulation' in MeshCat to continue.")
    meshcat.AddButton('Stop Simulation')

    utils.show_diagram(diagram)

    # Run simulation
    input("Press [Enter] to start simulation...")
    while meshcat.GetButtonClicks('Stop Simulation') < 1:
        #print("Before advance")


        simulator.AdvanceTo(simulator.get_context().get_time() + max_advance_time)


        #print("After advance")
        time.sleep(sleep_time)
        #print("After sleep")


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