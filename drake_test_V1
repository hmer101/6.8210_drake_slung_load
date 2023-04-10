### LOCAL PYDRAKE EXAMPLE   ###

# from pydrake.common import FindResourceOrThrow
# from pydrake.multibody.parsing import Parser
# from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
# from pydrake.systems.analysis import Simulator
# from pydrake.systems.framework import DiagramBuilder

# builder = DiagramBuilder()
# plant, _ = AddMultibodyPlantSceneGraph(builder, 0.0)
# Parser(plant).AddModels(
#     FindResourceOrThrow("drake/examples/pendulum/Pendulum.urdf"))
# plant.Finalize()
# diagram = builder.Build()
# simulator = Simulator(diagram)


###  LOCAL MESHCAT EXAMPLE   ###

# python libraries
import numpy as np
from IPython.display import HTML, display
# pydrake imports
from pydrake.all import (AddMultibodyPlantSceneGraph, DiagramBuilder,
                         Linearize, LinearQuadraticRegulator, LogVectorOutput,
                         MeshcatVisualizer, ModelVisualizer, Parser, Simulator, StartMeshcat)

from underactuated import running_as_notebook

# Start the visualizer (run this cell only once, each instance consumes a port)
meshcat = StartMeshcat()

# DO NOT MODIFY
base_urdf = """
  <link name="base">

    <inertial>
      <origin xyz="0 0 0" />
      <mass value="1" />
    </inertial>

    <visual>
      <origin xyz="0 0 0" />
      <geometry>
        <box size=".5 .2 .2" />
      </geometry>
      <material>
        <color rgba="0 1 0 1" />
      </material>
    </visual>

    <visual>
      <origin xyz=".15 0 -.15" rpy="0 0 0" />
      <geometry>
        <sphere radius=".05" />
      </geometry>
      <material>
        <color rgba="0 0 0 1" />
      </material>
    </visual>
    
    <visual>
      <origin xyz="-.15 0 -.15" rpy="0 0 0" />
      <geometry>
        <sphere radius=".05" />
      </geometry>
      <material>
        <color rgba="0 0 0 1" />
      </material>
    </visual>
  </link>
"""

pendulum_link = """ 
  <link name="pendulum0">
    
  <!-- This is how you can write comments in urdfs -->
  <!-- TODO: Write the inertial component below -->
    <inertial>
        <origin xyz="0 0 -1.0" />
        <mass value="1" />
    </inertial>


  <!-- TODO: Write the visual component for the sphere (radius=0.05, mass=1.) below -->
    <visual>
      <origin xyz="0 0 -1.0" rpy="0 0 0" />
      <geometry>
        <sphere radius=".05" />
      </geometry>
      <material>
        <color rgba="0 0 0 1" />
      </material>
    </visual>

  <!-- TODO: Write the visual component for the cylindrical rod (radius=0.01, length=1.) below -->
    <visual>
      <origin xyz="0 0 -0.5" rpy="0 0 0" />
      <geometry>
        <cylinder radius=".01" length="1."/>
        <!-- <cylinder length="1." /> -->
      </geometry>
      <material>
        <color rgba="0 0 0 1" />
      </material>
    </visual>


  </link>
"""

# DO NOT MODIFY
base_joint = """
  <joint name="x" type="prismatic">
    <parent link="world" />
    <child link="base" />
    <axis xyz="1 0 0" />
  </joint>
"""

pendulum_joint = """

<!-- TODO: write the parent, child, axis and origin for the pendulum joint named "theta0" with type "continuous". -->
    <joint name="theta0" type="continuous">
        <parent link="base" />
        <child link="pendulum0" />

        <axis xyz="0 -1 0" />
        <origin xyz="0 0 0" rpy="0 0 0"/>
    </joint>


"""

# DO NOT MODIFY
transmission = """
  <transmission type="SimpleTransmission" name="base_force">
    <actuator name="force" />
    <joint name="x" />
  </transmission>
"""

single_pendulum_urdf = f"""
<?xml version="1.0"?><robot name="OurCartPole">
{base_urdf}
{pendulum_link}
{base_joint}
{pendulum_joint}
{transmission}
  </robot>
</xml>
"""
print(single_pendulum_urdf)
 
visualizer = ModelVisualizer(meshcat=meshcat)
visualizer.parser().AddModelsFromString(single_pendulum_urdf, 'urdf')
visualizer.Run(loop_once=not running_as_notebook)

# HACK TO KEEP OPEN
while True:
    pass

meshcat.Delete()
meshcat.DeleteAddedControls()

