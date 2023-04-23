import time

import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('TkAgg') # Because I get error "Matplotlib is currently using agg, which is a non-GUI backend, so cannot show the figure.

from pydrake.all import(
    ModelVisualizer, 
    Simulator, 
    plot_system_graphviz)


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
    context.SetContinuousState(state_init)
    simulator.Initialize()

    print("Press 'Stop Simulation' in MeshCat to continue.")
    meshcat.AddButton('Stop Simulation')

    # Run simulation
    input("Press [Enter] to start simulation...")
    while meshcat.GetButtonClicks('Stop Simulation') < 1:
        simulator.AdvanceTo(simulator.get_context().get_time() + max_advance_time)
        time.sleep(sleep_time)