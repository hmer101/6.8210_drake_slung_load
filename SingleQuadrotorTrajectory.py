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

# from underactuated import ConfigureParser, running_as_notebook
# from underactuated.meshcat_utils import MeshcatSliders
from underactuated.quadrotor2d import Quadrotor2D, Quadrotor2DVisualizer

# if running_as_notebook:
# mpld3.enable_notebook()




########################################################################################################
# Diff_flatness_example
########################################################################################################

def circle_constraints(zpp, tf, amplitude=1, n=10):
    dof = 4
    amp = amplitude

    # Init condition
    zpp.add_constraint(t=0, derivative_order=2, lb=np.zeros(dof))

    # Intermediate conditions
    for ti in range(n):
        ratio = (ti)/n
        x = ratio * 2*np.pi
        lb = [amp*np.cos(x)+amp, amp*np.sin(x)+amp, -np.inf, -np.inf]
        ub = [amp*np.cos(x)+amp, amp*np.sin(x)+amp, np.inf, np.inf]
        zpp.add_constraint(t=tf*ratio, derivative_order=0, lb=lb, ub=ub)

    # Final conditions
    zpp.add_constraint(t=tf, derivative_order=2, lb=np.zeros(dof))



def circle_example():
    dof = 4
    tf = 20
    degree=6
    continuity_degree=degree-1
    diff_solver_samples = 7 # for solving the piecewise linear trajectory
    discretization_samples = 100 # for backworking to get the remaining state variables and u

    zpp = PPTrajectory(
        sample_times=np.linspace(0, tf, diff_solver_samples),
        num_vars=dof,
        degree=degree,
        continuity_degree=continuity_degree,)

    circle_constraints(zpp, tf)
    did_succeed = zpp.generate()
    t = np.linspace(0, tf, discretization_samples)
    z = np.zeros((dof, len(t)))

    for i in range(len(t)):
        z[:, i] = zpp.eval(t[i])

    # Z is shape (DOF, discretization_samples)
    if did_succeed:
        return zpp
    else:
        print("Error with trajectory generation")
        zpp.show_traj(z)
        return None




########################################################################################################
# PPTrajectory
########################################################################################################



# TODO(russt): Use drake.trajectories.PiecewisePolynomialTrajectory
#  instead (currently missing python bindings for the required constructor),
#  or port this class to C++.

# using radians
def rotation_matrix(angles):
    roll, pitch, yaw = angles
    # Create rotation matrices for each axis
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(roll), -np.sin(roll)],
                    [0, np.sin(roll), np.cos(roll)]])
    R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                    [0, 1, 0],
                    [-np.sin(pitch), 0, np.cos(pitch)]])
    R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                    [np.sin(yaw), np.cos(yaw), 0],
                    [0, 0, 1]])

    # Combine the rotation matrices
    R = R_z @ R_y @ R_x

    return R


########################################################################################################
# From example, not written by me
########################################################################################################
class PPTrajectory:
    def __init__(self, sample_times, num_vars, degree, continuity_degree):
        self.sample_times = sample_times
        self.n = num_vars
        self.degree = degree

        self.prog = MathematicalProgram()
        self.coeffs = []
        for i in range(len(sample_times)):
            self.coeffs.append(
                self.prog.NewContinuousVariables(num_vars, self.degree + 1, "C")
            )
        self.result = None

        # Add continuity constraints
        for s in range(len(sample_times) - 1):
            trel = sample_times[s + 1] - sample_times[s]
            coeffs = self.coeffs[s]
            for var in range(self.n):
                for deg in range(continuity_degree + 1):
                    # Don't use eval here, because I want left and right
                    # values of the same time
                    left_val = 0
                    for d in range(deg, self.degree + 1):
                        left_val += (
                            coeffs[var, d]
                            * np.power(trel, d - deg)
                            * math.factorial(d)
                            / math.factorial(d - deg)
                        )
                    right_val = self.coeffs[s + 1][var, deg] * math.factorial(
                        deg
                    )
                    self.prog.AddLinearConstraint(left_val == right_val)

        # Add cost to minimize highest order terms
        for s in range(len(sample_times) - 1):
            self.prog.AddQuadraticCost(
                np.eye(num_vars),
                np.zeros((num_vars, 1)),
                self.coeffs[s][:, -1],
            )

    def eval(self, t, derivative_order=0):
        if derivative_order > self.degree:
            return 0

        s = 0
        while s < len(self.sample_times) - 1 and t >= self.sample_times[s + 1]:
            s += 1
        trel = t - self.sample_times[s]

        if self.result is None:
            coeffs = self.coeffs[s]
        else:
            coeffs = self.result.GetSolution(self.coeffs[s])

        deg = derivative_order
        val = 0 * coeffs[:, 0]
        for var in range(self.n):
            for d in range(deg, self.degree + 1):
                val[var] += (
                    coeffs[var, d]
                    * np.power(trel, d - deg)
                    * math.factorial(d)
                    / math.factorial(d - deg)
                )

        return val

    def add_constraint(self, t, derivative_order, lb, ub=None):
        """Adds a constraint of the form d^deg lb <= x(t) / dt^deg <= ub."""
        if ub is None:
            ub = lb

        assert derivative_order <= self.degree
        val = self.eval(t, derivative_order)
        self.prog.AddLinearConstraint(val, lb, ub)

    def generate(self):
        self.result = Solve(self.prog)
        return self.result.is_success()

def show_traj(z):
    ax.plot(z[0, :], z[1, :])

    for t in np.linspace(0, tf, 10):
        x = zpp.eval(t)
        xddot = zpp.eval(t, 2)
        
        # Use x, xdot, xddot to solve for other constraints
        theta = np.arctan2(-xddot[0], (xddot[1] + 9.81))


        v = Quadrotor2DVisualizer(ax=ax)
        context = v.CreateDefaultContext()
        v.get_input_port(0).FixValue(context, [x[0], x[1], theta, 0, 0, 0])
        v.draw(context)

    # show_objects(ax)
    ax.set_xlim([-1, 7])
    ax.set_ylim([-1, 5])
    ax.set_title("")

if False:  # May be useful for debugging
    t = np.linspace(0, tf, 100)
    z = np.zeros((2, len(t)))
    knots = np.zeros((2, len(zpp.sample_times)))
    fig, ax = plt.subplots(zpp.degree + 1, 1)
    for deg in range(zpp.degree + 1):
        for i in range(len(t)):
            z[:, i] = zpp.eval(t[i], deg)
        for i in range(len(zpp.sample_times)):
            knots[:, i] = zpp.eval(zpp.sample_times[i], deg)
        ax[deg].plot(t, z.transpose())
        ax[deg].plot(zpp.sample_times, knots.transpose(), ".")
        ax[deg].set_xlabel("t (sec)")
        ax[deg].set_ylabel("z deriv " + str(deg))










########################################################################################################
# Quadcopter Solver
########################################################################################################
def solve_for_states(zpp):

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
    tcount = 100
    g= 9.81
    u       = prog.NewContinuousVariables(tcount, 4, name="u_i")     # desired output of rotor i
    rpy_i    = prog.NewContinuousVariables(tcount, 3, name="RPi")   # roll, pitch of rotor i
    Omega_i = prog.NewContinuousVariables(tcount, 3, name="RPdoti")   # roll, pitch of rotor i
    Omegadot_i= prog.NewContinuousVariables(tcount, 3, name="RPddoti")   # roll, pitch of rotor i
    # Tiqi  = prog.NewContinuousVariables(tcount, 3, name="Tiqi") # Tension x, y, z of rotor i
    x_i = [zpp.eval(t)[0:3] for t in range(tcount)]
    xdot_i = [zpp.eval(t,1)[0:3] for t in range(tcount)]
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
        return x_i, xdot_i, rpy_i_out, omega_i_out, u_out #, Tiqi_out














def print_traj(x_i, xdot_i, rpy_i, omega_i, ui):
    for t in range(len(x_i)):
        xyz    = zpp.eval(t)[0:3]
        xyzdot = zpp.eval(t,1)[0:3]
        rpy    = rpy_i[t]
        rpydot    = omega_i[t]
        u = ui
        print(f"---------- t = {t} ----------")
        print (f"x={xyz[0]}\t y={xyz[1]}\t z={xyz[2]}")
        print (f"xd={xyzdot[0]}\t yd={xyzdot[0]}\t zd={xyzdot[0]}")
        print (f"r={rpy[0]}\t p={rpy[1]}\t y={rpy[2]}")
        print (f"rd={rpydot[0]}\t pd={rpydot[1]}\t yd={rpydot[2]}")
        print (f"u= {u[0]}")

if True:
    zpp = circle_example()
    x_i, xdot_i, rpy_i, omega_i, ui = solve_for_states(zpp)
    print_traj(x_i, xdot_i, rpy_i, omega_i, ui)