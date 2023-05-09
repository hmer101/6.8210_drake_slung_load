
# Imports
import math

import matplotlib.pyplot as plt
import mpld3
import numpy as np
from IPython.display import HTML, display
from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    ControllabilityMatrix,
    DiagramBuilder,
    Linearize,
    LinearQuadraticRegulator,
    MeshcatVisualizer,
    Parser,
    Saturation,
    SceneGraph,
    Simulator,
    StartMeshcat,
    WrapToSystem,
)
from pydrake.examples import (
    AcrobotGeometry,
    AcrobotInput,
    AcrobotPlant,
    AcrobotState,
    QuadrotorGeometry,
    QuadrotorPlant,
    StabilizingLQRController,
)
from pydrake.solvers import MathematicalProgram, Solve

from underactuated import ConfigureParser, running_as_notebook
from underactuated.meshcat_utils import MeshcatSliders
from underactuated.quadrotor2d import Quadrotor2D, Quadrotor2DVisualizer

if running_as_notebook:
    mpld3.enable_notebook()




tcount = 100
g= 9.81


def solve_for_states(z):

    ###############################
    #  Information and constants  #
    ###############################
    # Z vector drone: [ xi yi zi rolli pitchi yawi ]

    # Z vector : [ xi yi zi yawi ]

    # How to pull from z:
    # x = zpp.eval(t) 
    # xddot = zpp.eval(t, 2)
    # Tiqi = [0,0,0] # quadrotor tension vector (with direction)

    ###############################
    #   Constants & Conversions   #
    ###############################
    m_i = 1 # quadrotor mass (update later)
    J_i = np.eye(3) # quadrotor inertia (update later)
    e3 = np.array([0,0,1]) # z vector
    thrust_ratio = 1
    moment_ratio = 1
    # used to convert u1 u2 u3 u4 to Tx Ty and Tz
    # u2m = moment_ratio * np.array([[0,1,0,-1],[1,0 -1,0],[-1,1,-1,1]] )
    # u2m = np.array([[1, 2, 3], [4, 5, 6],[1, 2, 3]])
    u2m = moment_ratio * np.array([[0, 1, 0, -1], [1, 0, -1, 0],[-1, 1, -1, 1]])
 
    ###############################
    #           Unknowns          #
    ###############################
    prog    = MathematicalProgram()
    u       = prog.NewContinuousVariables(tcount, 4, name="u_i")     # desired output of rotor i
    rpy_i    = prog.NewContinuousVariables(tcount, 3, name="RPi")   # roll, pitch of rotor i
    Omega_i = prog.NewContinuousVariables(tcount, 3, name="RPdoti")   # roll, pitch of rotor i
    Omegadot_i= prog.NewContinuousVariables(tcount, 3, name="RPddoti")   # roll, pitch of rotor i
    # Tiqi  = prog.NewContinuousVariables(tcount, 3, name="Tiqi") # Tension x, y, z of rotor i
    fi = []

    for t in range(tcount):

        # x,y,z accelerations of drone i
        x     = zpp.eval(t)[0:3]
        xdot  = zpp.eval(t,1)[0:3]
        xddot = zpp.eval(t,2)[0:3]
        
        Ri = rotation_matrix(rpy_i[t])


        #########################
        # Kinematic Constraints #
        #########################
        # constraining yaw and its derivatives
        prog.AddConstraint(rpy_i[t][2]      == zpp.eval(t)[3])
        prog.AddConstraint(Omega_i[t][2]    == zpp.eval(t,1)[3])
        prog.AddConstraint(Omegadot_i[t][2] == zpp.eval(t,2)[3])

        #########################
        #    Sum of forces      #
        #########################
        # m*xddot, m*yddot, m*zddot
        lhs_f = m_i*xddot 
        # quadrotor force - gravity + tension force
        #      fi Ri e3  -  mi g e3
        fi = sum(u[t]) * thrust_ratio
        rhs_f = fi * Ri.dot(e3) - m_i*g*e3 #+ Tiqi[t]
        rhs_f = rhs_f.tolist()
        lhs_f = lhs_f.tolist()
        for j in range(len(lhs_f)):
            prog.AddConstraint(lhs_f[j] == rhs_f[j])

        #########################
        #    Sum of moments     #
        #########################
        lhs_m = J_i.dot(Omegadot_i[t]) + np.cross(Omega_i[t],J_i.dot(Omega_i[t]))
        M_i = u2m.dot(np.array(u[t]))
        rhs_m = M_i
        rhs_m = rhs_m.tolist()
        lhs_m = lhs_m.tolist()
        for j in range(len(lhs_m)):
            prog.AddConstraint(lhs_m[j] == rhs_m[j])


    result = Solve(prog)
    good = result.is_success()
    if good:


        u_out = result.GetSolution(u)
        rpy_i_out = result.GetSolution(rpy_i)
        omega_i_out = result.GetSolution(Omega_i)

        return u_out, rpy_i_out, omega_i_out #, Tiqi_out

ui, rpy_i, omega_i_out = solve_for_states(z)

print (ui)
print(rpy_i)
print(omega_i_out)


x = zpp.eval(t)[0:3]
xdot  = zpp.eval(t,1)[0:3]

for t in range(tcount):
    xyz    = zpp.eval(t)[0:3]
    xyzdot = zpp.eval(t,1)[0:3]
    rpy    = rpy_i[t]
    rpydot    = omega_i_out[t]
    u = ui
    print(f"---------- t = {t} ----------")
    print (f"x={xyz[0]}\t y={xyz[1]}\t z={xyz[2]}")
    print (f"xd={xyzdot[0]}\t yd={xyzdot[0]}\t zd={xyzdot[0]}")
    print (f"r={rpy[0]}\t p={rpy[1]}\t y={rpy[2]}")
    print (f"rd={rpydot[0]}\t pd={rpydot[1]}\t yd={rpydot[2]}")
    print (f"u= {u[0]}")