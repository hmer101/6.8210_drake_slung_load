# Imports
import math
import matplotlib.pyplot as plt
import mpld3
import numpy as np
import os
import pickle
from IPython.display import HTML, display
from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    # ControllabilityMatrix,
    DiagramBuilder,
    Linearize,
    # LinearQuadraticRegulator,
    # MeshcatVisualizer,
    Parser,
    Saturation,
    SceneGraph,
    Simulator,
    StartMeshcat,
    WrapToSystem,
)
# from pydrake.examples import (
#     # AcrobotGeometry,
#     # AcrobotInput,
#     # AcrobotPlant,
#     # AcrobotState,
#     # QuadrotorGeometry,
#     # QuadrotorPlant,
#     # StabilizingLQRController,
# )
from pydrake.solvers import MathematicalProgram, Solve, SnoptSolver

# from underactuated import ConfigureParser, running_as_notebook
# from underactuated.meshcat_utils import MeshcatSliders
# from underactuated.quadrotor2d import Quadrotor2D, Quadrotor2DVisualizer

# if running_as_notebook:
#     mpld3.enable_notebook()

from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation


def animate_trajectories(trajectories, legend=None, interval=50):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    if not(legend):
        legend = [f'Trajectory {i}' for i in range(len(trajectories))]
    lines = [ax.plot([], [], [], label=legend[i])[0] for i in range(len(trajectories))]
    start_points = [ax.scatter([], [], [], color='g') for _ in range(len(trajectories))]
    end_points = [ax.scatter([], [], [], color='r') for _ in range(len(trajectories))]

    def init():
        ax.set_xlim([min(min(traj[0]) for traj in trajectories), max(max(traj[0]) for traj in trajectories)])
        ax.set_ylim([min(min(traj[1]) for traj in trajectories), max(max(traj[1]) for traj in trajectories)])
        ax.set_zlim([min(min(traj[2]) for traj in trajectories), max(max(traj[2]) for traj in trajectories)])
        return lines + start_points + end_points

    def update(i):
        for j, line in enumerate(lines):
            line.set_data(trajectories[j][0][:i], trajectories[j][1][:i])
            line.set_3d_properties(trajectories[j][2][:i])
            start_points[j]._offsets3d = (trajectories[j][0][:1], trajectories[j][1][:1], trajectories[j][2][:1])
            end_points[j]._offsets3d = (trajectories[j][0][i-1:i], trajectories[j][1][i-1:i], trajectories[j][2][i-1:i])
        return lines + start_points + end_points

    ani = FuncAnimation(fig, update, frames=np.arange(1, len(trajectories[0][0])), init_func=init, blit=True, interval=interval)
    plt.legend()
    plt.show()



def plot_trajectories(arr, x=-1, legend=['x', 'y', 'z'], title=None, xlabel="Time (s)", ylabel=None, xscale=.1):
    if (x==-1):
        for x in range(len(arr[0])):
            arr_list = [arr[t][x] for t in range(len(arr))]
            plt.plot([i * xscale for i in range(len(arr_list))], arr_list[:], label=legend[x])
    else: 
        arr_list = [arr[t][x] for t in range(len(arr))]
        plt.plot([i * xscale for i in range(len(arr_list))], arr_list[:], label=legend[x])
    if title:
        plt.title(title)
    if xlabel:
        plt.xlabel(xlabel)
    if ylabel:
        plt.ylabel(ylabel)
    plt.legend()
    plt.grid()
    plt.show()

def plot_flat_trajectory(x_values, y_values, title=None, xlabel=None, ylabel=None):
    plt.plot(x_values, y_values)
    plt.scatter(x_values[0], y_values[0], color='g', label='Start')
    plt.scatter(x_values[-1], y_values[-1], color='r', label='End')
    if title:
        plt.title(title)
    if xlabel:
        plt.xlabel(xlabel)
    if ylabel:
        plt.ylabel(ylabel)
    plt.legend()
    plt.grid()
    plt.show()


def plot_trajectories_3d(trajectories_3d, legend=None):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    if not legend:
        legend = [f'Trajectory {i+1}' for i in range(len(trajectories_3d))]
    for i, trajectory in enumerate(trajectories_3d):
        x_values, y_values, z_values = trajectory.T
        ax.plot(x_values, y_values, z_values, label=legend[i])
        ax.scatter(x_values[0], y_values[0], z_values[0], color='g')
        ax.scatter(x_values[-1], y_values[-1], z_values[-1], color='r')
    ax.set_xlim([min(min(traj[:, 0]) for traj in trajectories_3d), max(max(traj[:, 0]) for traj in trajectories_3d)])
    ax.set_ylim([min(min(traj[:, 1]) for traj in trajectories_3d), max(max(traj[:, 1]) for traj in trajectories_3d)])
    ax.set_zlim([min(min(traj[:, 2]) for traj in trajectories_3d), max(max(traj[:, 2]) for traj in trajectories_3d)])
    plt.legend()
    plt.show()



def animate_trajectories(trajectories_3d, legend=None):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    if not legend:
        legend = [f'Trajectory {i+1}' for i in range(len(trajectories_3d))]
    lines = [ax.plot([], [], [], label=legend[i])[0] for i in range(len(trajectories_3d))]
    start_points = [ax.scatter([], [], [], color='g') for _ in range(len(trajectories_3d))]
    # end_points = [ax.scatter([], [], [], color='r') for _ in range(len(trajectories_3d))]

    def init():
        ax.set_xlim([min(min(traj[:, 0]) for traj in trajectories_3d), max(max(traj[:, 0]) for traj in trajectories_3d)])
        ax.set_ylim([min(min(traj[:, 1]) for traj in trajectories_3d), max(max(traj[:, 1]) for traj in trajectories_3d)])
        ax.set_zlim([min(min(traj[:, 2]) for traj in trajectories_3d), max(max(traj[:, 2]) for traj in trajectories_3d)])
        return lines + start_points #+ end_points

    def update(i):
        for j, line in enumerate(lines):
            x_values, y_values, z_values = trajectories_3d[j].T
            line.set_data(x_values[:i], y_values[:i])
            line.set_3d_properties(z_values[:i])
            start_points[j]._offsets3d = (x_values[:1], y_values[:1], z_values[:1])
            # end_points[j]._offsets3d = (x_values[i-1:i], y_values[i-1:i], z_values[i-1:i])
        return lines + start_points #+ end_points

    ani = FuncAnimation(fig, update, frames=np.arange(1, len(trajectories_3d[0])), init_func=init, blit=True)
    plt.legend()
    plt.show()



def plot_multiple_flat_trajectories(trajectories, title=None, xlabel=None, ylabel=None, legend=None):
    for i, trajectory in enumerate(trajectories):
        x_values, y_values = trajectory
        if legend:
            plt.plot(x_values, y_values, label=legend[i])
        else:
            plt.plot(x_values, y_values)
        plt.scatter(x_values[0], y_values[0], color='g')
        plt.scatter(x_values[-1], y_values[-1], color='r')
    if title:
        plt.title(title)
    if xlabel:
        plt.xlabel(xlabel)
    if ylabel:
        plt.ylabel(ylabel)
    if legend:
        plt.legend()
    plt.grid()
    plt.show()


# for 4 quadrotors with a load
def circle_example_n_rotors(n, degree=6, continuity_degree=5, discretization_samples=10, diff_solver_samples=7, tf=20):
    dof_tot = 5*n+6 # total DOF
    dof_act = 4*n   # deg of act
    dof_ua  = dof_tot - dof_act # degree underactuated

    # diff_solver_samples for solving the piecewise linear trajectory
    # discretization_samples for backworking to get the remaining state variables and u

    zpp = PPTrajectory(
        sample_times=np.linspace(0, tf, diff_solver_samples), # evenly spaced out time samples
        num_vars=dof_act, # controlled degrees of actuation
        degree=degree, # degree of the polynomial
        continuity_degree=continuity_degree,) # continuity degree of the polynomial

    lam_min = .0001*9.8/3
    lam_max = .0001*9.8/3

    circle_constraints_n_rotors(zpp, tf, n, lam_min, lam_max, amplitude=1, diff_solver_samples=diff_solver_samples)
    did_succeed = zpp.generate()
    t = np.linspace(0, tf, discretization_samples)
    z = np.zeros((dof_act, len(t)))

    for i in range(len(t)):
        z[:, i] = zpp.eval(t[i])

    # Z is shape (DOF, discretization_samples)
    if did_succeed:
        # print(f"Deg={degree} cdeg={continuity_degree} ok")
        return zpp
    else:
        # zpp.show_traj(z)
        print(f"Deg={degree} cdeg={continuity_degree} failed")
        # assert did_succeed, "Error with trajectory generation"
        return None


def circle_constraints_n_rotors(zpp, tf, n, lam_min, lam_max, amplitude=1, diff_solver_samples=10):
    dof = 4*n
    ##########################################################################################################
    # zpp --> [xload, yload, zload, rload, pload, yawLoad, lambda_1, ..., lambda_(3n-6), psi_1, ..., psi_n]  #
    ##########################################################################################################
    amp = amplitude # amplitude of the cirlce in XY
    altitute_l = 0 # load altitude
    hdg_l = 0       # load heading
    pitch_l = 0     # load pitch
    yaw_l = 0       # load yaw    
    lambda_min = [lam_min]*(3*n-6) # lambda, whatever it ends up being in the solver
    lambda_max = [lam_max]*(3*n-6)
    psi_min = [0] * n # heading of each drone
    psi_max = [0] * n
    assert (len(psi_min) == n), "error, psi_min length incorrect"
    # Init condition
    # zpp.add_constraint(t=0, derivative_order=0, lb=np.zeros(dof))
    # zpp.add_constraint(t=0, derivative_order=1, lb=np.zeros(dof))


    # Intermediate conditions
    for ti in range(diff_solver_samples):
        ratio = (ti)/diff_solver_samples
        x = ratio * 2*np.pi
        tol = 0.01
        # #      X lb and ub,       Y lb and ub,      Z,        hdg
        lb = [amp*np.cos(x)+amp-tol,   amp*np.sin(x)+amp-tol,        altitute_l,    hdg_l, pitch_l, yaw_l] + lambda_min + psi_min
        ub = [amp*np.cos(x)+amp+tol,   amp*np.sin(x)+amp+tol,        altitute_l,    hdg_l, pitch_l, yaw_l] + lambda_max + psi_max # Y --> amp*np.sin(x)+amp

        # Trivial example for now
        # lb = [0,      0-tol,        altitute_l,    hdg_l, pitch_l, yaw_l] + lambda_min + psi_min
        # ub = [0,      0+tol,        altitute_l,    hdg_l, pitch_l, yaw_l] + lambda_max + psi_max # Y --> amp*np.sin(x)+amp

        assert len(lb) == dof, f"Length of lb should equal DOF=n={n}, got: {len(lb)}"
        assert len(ub) == dof, f"Length of ub should equal DOF=n={n}, got: {len(ub)}"
        zpp.add_constraint(t=tf*ratio, derivative_order=0, lb=lb, ub=ub)

    # Final conditions (stationary)
    # zpp.add_constraint(t=tf, derivative_order=0, lb=np.zeros(dof))
    # zpp.add_constraint(t=tf, derivative_order=1, lb=np.zeros(dof))










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


# From example, not written by me
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


        cache_file = "ppt_trajectory_solution.npy"
        # Solve for trajectory
        if (os.path.exists(cache_file)):
            # print(f" loading initial guess from file {cache_file}")
            with open(cache_file, 'rb') as f:
                data = pickle.load(f)
            # self.prog.SetInitialGuessForAllVariables(data)
        result = Solve(self.prog)
        assert result.is_success()
        pickle.dump( result.GetSolution(), open( cache_file, "wb" ) )
        print(f" saving initial guess to file {cache_file}")
        
        self.result = Solve(self.prog)
        return self.result.is_success()





##############################################################################################################################
# zpp --> [xload, yload, zload, rload, pload, yawLoad, lambda_0, ..., lambda_j, ...,lambda_(3n-5), psi_0, ...,  psi_j, ...   #
#            0      1      2      3      4       5       6+0     ...,  6+j,     ...,  3n-1           3n   ...,  3n+j , ...   #
##############################################################################################################################


def solve_for_states_n_rotors(zpp, n, tf, timesteps):
    print(f"solving states for {n} quadrotors using {timesteps} timesteps with tf={tf}")

    enable_delta_u_cost = False
    enable_delta_x_cost = False
    enable_delta_r_cost = False
    seed_solver = True
    major_tol = 1e-2
    minor_tol = 1e-2


    ###############################
    #  Information and constants  #
    ###############################
    # Z vector drone: [ xi yi zi rolli pitchi yawi ]
    # Z vector : [ xi yi zi yawi ]
    # How to pull from z:
    # x = zpp.eval(t) 
    # xddot = zpp.eval(t, 2)
    # Tiqi = [0,0,0] # tension vector (acting on quadrotor)
    ###############################
    #   Constants & Conversions   #
    ###############################
    tcount = timesteps
    dt = tf/tcount
    g= 9.81
    e3 = np.array([0,0,1]) # z vector


    ###############################
    # Load & Drone Dynamics       #
    ###############################

    # distance from anchor point to drone
    rope_len = 0.174  # Length for arms (m)
    # vectors from the load's COM to the anchor point
    anchor_points = 0.5 * np.array([ np.array([ 0,  1,  1]),
                                    np.array([-1, -1,  1]),
                                    np.array([-1,  1,  1])])
    
    # 3D vector distances from drone's COM to each rotor's COM (from sdf_models/x500/model.sdf)
    rotor_distances = np.array([np.array([-0.174, -0.174, 0.06]),np.array([-0.174, 0.174, 0.06]),np.array([0.174, 0.174, 0.06]),np.array([0.174, -0.174, 0.06])])
    
    # each individual rotor (4 per drone, currently assumed to be identical, but can be changed as needed)
    m_rotor = 0.016076923076923075
    J_rotors = [3.8464910483993325e-07, 2.6115851691700804e-05, 2.649858234714004e-05]
    # base of the drone
    m_base = 2.0
    J_base = [0.02166666666666667, 0.02166666666666667, 0.04000000000000001]
    # total mass of the drone
    m = [m_rotor*4+m_base]*n
    # inertia of the drone's body using parallel axis theorem
    J_drone = [J_base[0]+m_rotor*rotor_distances[0][0]**2, J_base[1]+m_rotor*rotor_distances[0][1]**2, J_base[2]+m_rotor*rotor_distances[0][2]**2]
    J = []
    for i in range(n):
        J.append(np.diag(J_drone)) # quadrotor inertia (copied from model.sdf)
    
    # load mass & inertia (semi-arbitrary for now)
    m_L = 1.0 # load mass
    J_L = np.diag(J_drone)*m_L/m[0]

    # Rotor positions & directions
    #
    #                         ^  y
    #  ccw  1    2  cw        |
    #         \ /             |_______> x
    #         / \
    #   ccw  0   3  cw
    #
    # Thruster info
    thrust_ratio = 1.0
    moment_ratio = 0.0245
    umax = 10 # maximum commanded thrust
    # used to convert u1 u2 u3 u4 to Tx Ty and Tz
    # assumes vertical rotors with directions ccw, ccw, cw, cw
    u2m = np.array([[rotor_distances[0][0], rotor_distances[1][0], rotor_distances[2][0], rotor_distances[3][0]], 
                    [rotor_distances[0][1], rotor_distances[1][1], rotor_distances[2][1], rotor_distances[3][1]],
                    [moment_ratio, moment_ratio, -moment_ratio, -moment_ratio]])


    ###############################
    #           Unknowns          #
    ###############################
    prog    = MathematicalProgram()
    #  [u1_n=0  u2_n=0  u3_n=0  u4_n=0  u1_n=1 ... u1_n=n   u1_n=n    u3_n=n   u4_n=n]
    #    0        1        2       3     3+1.       4n+0     4n+1      4n+2     4n+3
    u       = prog.NewContinuousVariables(tcount, 4*n, name="u_i")        # desired output of rotor i
    #  [x0 y0 z0 x1 y1 z1 ... xn      yn      zn]
    #   0. 1. 2.             3n+0    3n+1    3n+2
    X_i     = prog.NewContinuousVariables(tcount, 3*n, name="X_i")      # x of rotor i (X vector, x, y, and z)
    Xdot_i  = prog.NewContinuousVariables(tcount, 3*n, name="Xdot_i")   # x dot of rotor i
    Xddot_i = prog.NewContinuousVariables(tcount, 3*n, name="Xddot_i")  # x ddot of rotor i
    #  [r0   p0   yaw0  r1 ...   rn    pn    yawn]
    #    0   1    2            3n+0   3n+1  3n+2
    rpy_i     = prog.NewContinuousVariables(tcount, 3*n, name="RPYi")        # roll, pitch, yaw of rotor i
    Omega_i   = prog.NewContinuousVariables(tcount, 3*n, name="RPdoti")     # roll, pitch, yaw dot of rotor i
    Omegadot_i= prog.NewContinuousVariables(tcount, 3*n, name="RPddoti")    # roll, pitch, yaw ddot of rotor i
    # [Tx_0    Ty_0    Tz_0   Tx_1 ... Tx_n   Ty_n   Tz_n]
    #   0        1       2      3  ... 3n+0   3n+1   3n+2
    Tiqi  = prog.NewContinuousVariables(tcount, 3*n, name="Tiqi")           # Tension x, y, z of rotor i
    # [lambda_0     lambda_1    ...    lambda_j    ]
    # [   0           1                  j         ]
    lambda_m = prog.NewContinuousVariables(tcount, 3*n-6, name="lambda_m")  # lambda

    ############################################################
    # Z, Zdot, Zddot. Knowns for differentially flat variables #
    ############################################################
    no_d  = [zpp.eval(t*dt)    for t in range(tcount)]
    one_d = [zpp.eval(t*dt,1)  for t in range(tcount)]
    two_d = [zpp.eval(t*dt,2)  for t in range(tcount)]

    # List of each timestep. 't' is used as an index for refrencing the actual times in the list below
    t_array = [t*dt for t in range(tcount)]
 
    # NOT computing the final u
    for t in range(tcount-1):
        # x,y,z, roll, pitch, yaw positions, velocities, and accelerations of load
        XL     = no_d[t][0:3]
        XdotL  = one_d[t][0:3]
        XddotL = two_d[t][0:3]
        RL         = no_d[t][3:6]
        OmegaL     = one_d[t][3:6]
        OmegadotL  = two_d[t][3:6]

        # Converts load oriention to a rotation matrix
        r_L = rotation_matrix(RL)

        assert type(XL) == type(np.array([])), "Error, XL not a numpy array"

        # Drones cannot angle more than 90*max_angle_pct degrees (enforces more realistic solutions)
        max_angle_pct = 0.4
        max_angle = max_angle_pct*(np.pi/4)
        prog.AddLinearConstraint(rpy_i[t], [-max_angle]*3*n, [max_angle]*3*n)
        prog.AddLinearConstraint(rpy_i[t], [-max_angle]*3*n, [max_angle]*3*n)


        prog.AddLinearConstraint(u[t], [umax/10]*(4*n), [umax]*(4*n))

        for i in range(n):
            # Rotation matrix of quadrotor i wrt earth
            r_i = rotation_matrix(rpy_i[t][0+i:3+i])
            assert r_i.shape == (3,3), "Error, rotation matrix not 3x3 (r_i={r_i})"


            # Tension must pull rotor down (negative z component)
            # prog.AddLinearConstraint(Tiqi[t][3*i:3*i+3], [-100000,-100000,-100000], [100000,100000,5])
            # prog.AddLinearConstraint(Tiqi[t][3*i+2:3*i+3], [-100000], [0])
            # prog.AddQuadraticCost((Tiqi[t][3*i+2]-m_L*g/n)**2)


            #########################
            # Kinematic Constraints #
            #########################
            
            # rope must remain taut (magnitude = rope_len), and
            # Tension MUST be parallel to the rope. Proof for constraint as written below:
            # rope_length_vector/||rope_length_vector||  == Tiqi/||Tiqi||
            # rope_length_vector/rope_len                == Tiqi/||Tiqi||
            # rope_length_vector                         == Tiqi/||Tiqi|| * rope_len
            rope_length_vector = X_i[t][3*i:3*i+3] - (XL + anchor_points[i])
            qi = Tiqi[t][3*i:3*i+3]/np.linalg.norm(Tiqi[t][3*i:3*i+3])
            lhs_dist = (qi*rope_len).tolist()
            rhs_dist = rope_length_vector.tolist()
            for j in range(3):
                prog.AddConstraint(lhs_dist[j] == rhs_dist[j])


            ###############################################
            # Drone Velocity and acceleration constraints #
            ###############################################

            # Linear
            if (t>0):
                # Linear position cost
                if (enable_delta_x_cost):
                    vel = (X_i[t-1] - X_i[t+1])/dt #+ Xddot_i[t]*dt
                    prog.AddQuadraticCost((vel - Xdot_i[t]).dot(vel - Xdot_i[t]))
                # Angular position cost
                if (enable_delta_r_cost):
                    ave_a = (rpy_i[t-1] + rpy_i[t+1])*0.5
                    prog.AddQuadraticCost((ave_a - rpy_i[t]).dot(ave_a - rpy_i[t]))
                # Input discontinuity cost
                if (enable_delta_u_cost):
                    ave_u = (u[t-1] + u[t+1])*0.5
                    prog.AddQuadraticCost((ave_u - u[t]).dot(ave_u - u[t]) *0.1)
                prog.AddQuadraticCost((m_L*g/(4*n) - u[t]).dot(m_L*g/(4*n) - u[t]))

            # Load yaw and its derivatives
            prog.AddConstraint(rpy_i[t][3*i+2]           == zpp.eval(t)  [3*n+i])
            prog.AddConstraint(Omega_i[t][3*i+2]         == zpp.eval(t,1)[3*n+i])
            prog.AddConstraint(Omegadot_i[t][3*i+2]      == zpp.eval(t,2)[3*n+i])


            #####################################
            #    Sum of forces on the drones    #
            #####################################
            # m*xddot, m*yddot, m*zddot of drone i
            lhs_f = m_L*Xddot_i[t][3*i:3*i+3]
            # quadrotor force - gravity + tension force
            #      fi Ri e3  -  mi g e3
            fi = sum(u[t][4*i:4*i+4]) * thrust_ratio
            rhs_f = fi * r_i.dot(e3) - m[i]*g*e3 + r_L.dot(Tiqi[t][3*i:3*i+3])
            assert len(lhs_f) == 3, "Error, lhs_f != 3"
            assert len(rhs_f) == 3, "Error, rhs_f != 3"
            rhs_f = rhs_f.tolist()
            lhs_f = lhs_f.tolist()
            for j in range(len(lhs_f)):
                prog.AddConstraint(lhs_f[j] == rhs_f[j])

            #####################################
            #    Sum of moments on the drones   #
            #####################################
            lhs_m = J[i].dot(Omegadot_i[t][3*i:3*i+3]) + np.cross(Omega_i[t][3*i:3*i+3],J[i].dot(Omega_i[t][3*i:3*i+3]))
            Moments = u2m.dot(np.array(u[t][4*i:4*i+4]))
            rhs_m = Moments
            rhs_m = rhs_m.tolist()
            lhs_m = lhs_m.tolist()
            assert len(lhs_m) == 3, "Error, lhs_m != 3"
            assert len(rhs_m) == 3, "Error, rhs_m != 3"
            for j in range(len(lhs_m)):
                prog.AddConstraint(lhs_m[j] == rhs_m[j])

            # LOOPED THROUGH EACH QUADROTOR

        #####################################
        #    Sum of forces on the load      #
        #####################################
        # m*xddot, m*yddot, m*zddot
        lhs_f = m_L*XddotL
        tension = 0
        for i in range(n):
            tension += r_L.dot(Tiqi[t][3*i:3*i+3]) #re-orient tension 
        rhs_f = -tension - m_L*g*e3 #change direction of the tension, add gravity
        rhs_f = rhs_f.tolist()
        lhs_f = lhs_f.tolist()
        assert len(lhs_f) == 3, "Error, load lhs_f != 3"
        assert len(rhs_f) == 3, "Error, load rhs_f != 3"
        for j in range(len(lhs_f)):
            prog.AddConstraint(lhs_f[j] == rhs_f[j])

        #####################################
        #    Sum of moments on the load     #
        #####################################
        lhs_m = J_L.dot(OmegadotL) + np.cross(OmegaL,J_L.dot(OmegaL))
        M_i = -1*sum([np.cross(anchor_points[i],Tiqi[t][3*i:3*i+3]) for i in range(n)])
        rhs_m = M_i#[0]
        rhs_m = rhs_m.tolist()
        lhs_m = lhs_m.tolist()
        for j in range(len(lhs_m)):
            prog.AddConstraint(lhs_m[j] == rhs_m[j])
        assert len(lhs_m) == 3, "Error, load lhs_m != 3"
        assert len(rhs_m) == 3, "Error, load rhs_m != 3"


        #####################################
        #              Lambda               #
        #####################################
        # T12
        # T13
        # T32
        # Need to hard-code lambdas in for different numbers of quadcopters
            # [Tx_0    Ty_0    Tz_0   Tx_1 ... Tx_n   Ty_n   Tz_n]
            #   0        1       2      3  ... 3n+0   3n+1   3n+2
        # if (n == 3):
            # prog.AddConstraint(lambda_m[t][i] == Tiqi[t][3*0+0]) # x of copter 0
            # prog.AddConstraint(lambda_m[t][i] == Tiqi[t][3*0+1]) # y of copter 0
            # prog.AddConstraint(lambda_m[t][i] == Tiqi[t][3*1+0]) # x of copter 1


    # END OF LOOP THRU EACH TIMESTEP

    # solve mathematical program with initial guess
    ##############################################################################################################################
    # zpp --> [xload, yload, zload, rload, pload, yawLoad, lambda_0, ..., lambda_j, ...,lambda_(3n-5), psi_0, ...,  psi_j, ...   #
    #            0      1      2      3      4       5       6+0     ...,  6+j,     ...,  3n-1           3n   ...,  3n+j , ...   #
    ##############################################################################################################################
    
    dist_offset = np.matrix([[0,  0,  rope_len],
                        [0,  0,  rope_len],
                        [0,  0,  rope_len]])


    cache_file = f"n_rotor_states_{tcount}steps_{dt}dt.npy"
    # Solve for trajectory
    if (os.path.exists(cache_file)):
        if (seed_solver):
            print(f"\tloading initial guess from file {cache_file}")
            with open(cache_file, 'rb') as f:
                data = pickle.load(f)
                prog.SetInitialGuessForAllVariables(data)
        else:
            print(f"\tNOT loading initial guess from cache file {cache_file}")
    else:
        print(f"No initial cached guess for {tcount} steps, {dt} dt")

    solver = SnoptSolver()
    prog.SetSolverOption(solver.solver_id(), "Feasibility tolerance", major_tol)
    prog.SetSolverOption(solver.solver_id(), "Major feasibility tolerance", major_tol)
    prog.SetSolverOption(solver.solver_id(), "Major optimality tolerance", major_tol)
    prog.SetSolverOption(solver.solver_id(), "Minor feasibility tolerance", minor_tol)
    prog.SetSolverOption(solver.solver_id(), "Minor optimality tolerance", minor_tol)
    result = solver.Solve(prog)

    assert result.is_success(), "Error, solver failed to find a solution"
    pickle.dump( result.GetSolution(), open( cache_file, "wb" ) )
    
    print(f"\tsaving initial guess to file {cache_file}")
    good = result.is_success()

    axis = 0
    x_L_out     =  np.stack( [no_d[t][0:3] for t in range(tcount)], axis=axis)
    x_dot_L_out =  np.stack([one_d[t][0:3] for t in range(tcount)], axis=axis)
    R_L_out     =  np.stack( [no_d[t][3:6] for t in range(tcount)], axis=axis)
    Omega_L_out =  np.stack([one_d[t][3:6] for t in range(tcount)], axis=axis)
    x_i_out = result.GetSolution(X_i)
    x_dot_i_out = result.GetSolution(Xdot_i)
    u_out = result.GetSolution(u)
    rpy_i_out = result.GetSolution(rpy_i)
    omega_i_out = result.GetSolution(Omega_i)
    Tiqi_out = result.GetSolution(Tiqi)
    lambda_out = result.GetSolution(lambda_m)
    t_array = np.array(t_array)

    all_outputs = [x_L_out,x_i_out, R_L_out,rpy_i_out,    x_dot_i_out,x_dot_L_out, omega_i_out,Omega_L_out,   u_out, Tiqi_out]
    last_shape = x_L_out.shape
    for output in all_outputs:
        assert last_shape[0] == output.shape[0], "Error, shapes are different! last_shape={last_shape} output_shape={output.shape}"
    u_out = u_out[:-1]

    if not(good):
        print("DYNAMICS SOLVER FAILED")
    return x_L_out,x_i_out, R_L_out,rpy_i_out,    x_dot_i_out,x_dot_L_out, omega_i_out,Omega_L_out,   u_out, Tiqi_out, t_array





if __name__ == "__main__":

    timesteps = 51
    dt = .1
    tf = timesteps*dt
    zpp = circle_example_n_rotors(n=3, degree=6, continuity_degree=4, 
            discretization_samples=timesteps, diff_solver_samples=7, tf=tf)
    x_L,x_i, r_L,rpy_i, x_dot_i,x_dot_L, omega_i, Omega_L, u_out, Tiqi, t_array = solve_for_states_n_rotors(zpp, 
                                                                                    3, tf=tf, timesteps=timesteps)
    
    # no_d = [zpp.eval(t*dt)    for t in range(timesteps)]
    # x_L_out     =  np.stack( [no_d[t][0:3] for t in range(timesteps)], axis=0)
    
    # print("XL")
    # plot_trajectories(x_L, title="Load Position", ylabel="Position (m)")
    # print("RL")
    # plot_trajectories(r_L, title="Load Orientation", legend=["Roll","Pitch","Yaw"], ylabel="Angle (rad)")
    # print("xi")
    # plot_trajectories(x_i, title="Drone Positions", legend=["x0","y0","z0","x1","y1","z1","x2","y2","z2",], ylabel="Position (m)")


    trajectories_3d=[x_L[:-2,:], x_i[:-2,0:3], x_i[:-2,3:6], x_i[:-2,6:9]]
    legend=["Load","Drone 0","Drone 1","Drone 2"]
    plot_trajectories_3d(trajectories_3d, legend=legend)
    animate_trajectories(trajectories_3d, legend=legend)

    # trajectories_2d=[[x_L[:-2,0], x_L[:-2,1]], [x_i[:-2,0], x_i[:-2,1]], [x_i[:-2,3], x_i[:-2,4]], [x_i[:-2,6], x_i[:-2,7]]]
    # plot_multiple_flat_trajectories(trajectories_2d, title="Trajectories", xlabel="X (m)", ylabel="Y (m)", legend=["Load","Drone 0","Drone 1","Drone 2"])

    
    plot_trajectories(rpy_i[:,0:3], legend=["Roll","Pitch","Yaw"], title="Drone 0 Orientation", ylabel="Angle (rad)")
    print("rpy_1")
    plot_trajectories(rpy_i[:,3:6], legend=["Roll","Pitch","Yaw"], title="Drone 1 Orientation", ylabel="Angle (rad)")
    print("rpy_2")
    plot_trajectories(rpy_i[:,6:9], legend=["Roll","Pitch","Yaw"], title="Drone 2 Orientation", ylabel="Angle (rad)")
    print("TiQi 1")
    plot_trajectories(Tiqi[:-2,0:3], title="Drone 0 Tension Vector", ylabel="Tension (N)")
    print("TiQi 2")
    plot_trajectories(Tiqi[:-2,3:6], title="Drone 1 Tension Vector", ylabel="Tension (N)")
    print("TiQi 3")
    plot_trajectories(Tiqi[:-2,6:9], title="Drone 2 Tension Vector", ylabel="Tension (N)")
    print("u 1")
    plot_trajectories(u_out[:,0:4], legend=["Rotor 1","Rotor 2","Rotor 3","Rotor 4"], title="Drone 0 Rotor Forces", ylabel="Force (N)")
    print("u 2")
    plot_trajectories(u_out[:,4:8], legend=["Rotor 1","Rotor 2","Rotor 3","Rotor 4"], title="Drone 1 Rotor Forces", ylabel="Force (N)")
    print("u 3")
    plot_trajectories(u_out[:,8:12], legend=["Rotor 1","Rotor 2","Rotor 3","Rotor 4"], title="Drone 2 Rotor Forces", ylabel="Force (N)")





    # plot_flat_trajectory(x_L[:-2,0], x_L[:-2,1], title="Load Trajectory", xlabel="X (m)", ylabel="Y (m)")
    # plot_flat_trajectory(x_i[:-2,0], x_i[:-2,1], title="Drone 0 Trajectory", xlabel="X (m)", ylabel="Y (m)")
    # plot_flat_trajectory(x_i[:-2,3], x_i[:-2,4], title="Drone 1 Trajectory", xlabel="X (m)", ylabel="Y (m)")
    # plot_flat_trajectory(x_i[:-2,6], x_i[:-2,7], title="Drone 2 Trajectory", xlabel="X (m)", ylabel="Y (m)")
    # print("x0")
    # plot_trajectories(x_i[:,0:3], title="Drone 0 Position", ylabel="Position (m)")
    # print("x1")
    # plot_trajectories(x_i[:,3:6], title="Drone 1 Position", ylabel="Position (m)")
    # print("x2")
    # plot_trajectories(x_i[:,6:9], title="Drone 2 Position", ylabel="Position (m)")
    # print("rpy_0")
    
