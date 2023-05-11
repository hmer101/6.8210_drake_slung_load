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
    LeafSystem,
    ExternallyAppliedSpatialForceMultiplexer,
    RigidTransform)

from underactuated.scenarios import AddFloatingRpyJoint

class CustomLQRController(LeafSystem):
    def __init__(self, K_raw, x_0, u_0):
        super().__init__()  # Don't forget to initialize the base class.
        self._x_port = self.DeclareVectorInputPort(name="x", size=24)
        self.DeclareVectorOutputPort(name="u", size=8, calc=self.CalcOutput)
        
        self.K = K_raw
        self.x_0 = x_0
        self.u_0 = u_0

    def CalcOutput(self, context, output):
        # Evaluate the input ports to obtain the 2x1 vectors.
        x = self._x_port.Eval(context)

        self.K[0:4, 6:12] = 0
        self.K[0:4, 18:24] = 0
        self.K[4:8, 0:6] = 0
        self.K[4:8, 12:18] = 0

        u = self.u_0 - np.dot(self.K, (x - self.x_0))
        
        # Write the sum into the output vector.
        output.SetFromVector(u)
