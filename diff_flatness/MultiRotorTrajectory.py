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
from pydrake.solvers import MathematicalProgram, Solve

# from underactuated import ConfigureParser, running_as_notebook
# from underactuated.meshcat_utils import MeshcatSliders
# from underactuated.quadrotor2d import Quadrotor2D, Quadrotor2DVisualizer

# if running_as_notebook:
#     mpld3.enable_notebook()




def plot_trajectories(arr, x=-1):
    if (x==-1):
        for x in range(len(arr[0])):
            arr_list = [arr[t][x] for t in range(len(arr))]
            plt.plot(arr_list[:])
    else: 
        arr_list = [arr[t][x] for t in range(len(arr))]
        plt.plot(arr_list[:])
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
    zpp.add_constraint(t=0, derivative_order=1, lb=np.zeros(dof))


    # Intermediate conditions
    for ti in range(diff_solver_samples):
        ratio = (ti)/diff_solver_samples
        x = ratio * 2*np.pi
        tol = 0.1
        # #      X lb and ub,       Y lb and ub,      Z,        hdg
        lb = [amp*np.cos(x)+amp-tol,   amp*np.sin(x)+amp-tol,        altitute_l,    hdg_l, pitch_l, yaw_l] + lambda_min + psi_min
        ub = [amp*np.cos(x)+amp+tol,   amp*np.sin(x)+amp+tol,        altitute_l,    hdg_l, pitch_l, yaw_l] + lambda_max + psi_max # Y --> amp*np.sin(x)+amp

        # Trivial example for now
        # lb = [0,      0,        altitute_l,    hdg_l, pitch_l, yaw_l] + lambda_min + psi_min
        # ub = [0,      0,        altitute_l,    hdg_l, pitch_l, yaw_l] + lambda_max + psi_max # Y --> amp*np.sin(x)+amp

        assert len(lb) == dof, f"Length of lb should equal DOF=n={n}, got: {len(lb)}"
        assert len(ub) == dof, f"Length of ub should equal DOF=n={n}, got: {len(ub)}"
        zpp.add_constraint(t=tf*ratio, derivative_order=0, lb=lb, ub=ub)

    # Final conditions (stationary)
    zpp.add_constraint(t=tf, derivative_order=1, lb=np.zeros(dof))










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
            print(f" loading initial guess from file {cache_file}")
            with open(cache_file, 'rb') as f:
                data = pickle.load(f)
            self.prog.SetInitialGuessForAllVariables(data)
        result = Solve(self.prog)
        assert result.is_success()
        pickle.dump( result.GetSolution(), open( cache_file, "wb" ) )
        print(f" saving initial guess to file {cache_file}")
        
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
















##############################################################################################################################
# zpp --> [xload, yload, zload, rload, pload, yawLoad, lambda_0, ..., lambda_j, ...,lambda_(3n-5), psi_0, ...,  psi_j, ...   #
#            0      1      2      3      4       5       6+0     ...,  6+j,     ...,  3n-1           3n   ...,  3n+j , ...   #
##############################################################################################################################


def solve_for_states_n_rotors(zpp, n, tf, timesteps):
    print(f"solving states for {n} quadrotors using {timesteps} timesteps with tf={tf}")

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
    tcount = timesteps
    dt = tf/tcount
    g= 9.81

    m = [0.016076923076923075]*n # quadrotor mass (copied from model.sdf)
    umax = 10/4
    J = []
    for i in range(n):
        J.append(np.diag([3.8464910483993325e-07, 2.6115851691700804e-05, 2.649858234714004e-05])) # quadrotor inertia (copied from model.sdf)
    m_L = .0001
    J_L = np.diag([3.8464910483993325e-07, 2.6115851691700804e-05, 2.649858234714004e-05])*m_L
    # distance from anchor point to drone
    rope_len = 0.174  # Length for arms (m)
    # vectors from the load's COM to the anchor point
    anchor_points = 0.5 * np.array([ np.array([ 0,  1,  1]),
                                np.array([-1, -1,  1]),
                                np.array([-1,  1,  1])])

    e3 = np.array([0,0,1]) # z vector
    thrust_ratio = 1.0
    moment_ratio = 0.0245
    # used to convert u1 u2 u3 u4 to Tx Ty and Tz
    u2m = moment_ratio * np.array([[0, 1, 0, -1], [1, 0, -1, 0],[-1, 1, -1, 1]])
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


    no_d  = [zpp.eval(t*dt)    for t in range(tcount)]
    one_d = [zpp.eval(t*dt,1)  for t in range(tcount)]
    two_d = [zpp.eval(t*dt,2)  for t in range(tcount)]
    t_array = [t*dt for t in range(tcount)]
 
    for t in range(tcount-1):
        # x,y,z, roll, pitch, yaw positions, velocities, and accelerations of load
        XL     = no_d[t][0:3]
        XdotL  = one_d[t][0:3]
        XddotL = two_d[t][0:3]
        RL         = no_d[t][3:6]
        OmegaL     = one_d[t][3:6]
        OmegadotL  = two_d[t][3:6]

        r_L = rotation_matrix(RL)

        assert type(XL) == type(np.array([])), "Error, XL not a numpy array"

        max_angle = 0.4*np.pi/4
        prog.AddLinearConstraint(rpy_i[t], [-max_angle]*3*n, [max_angle]*3*n)
        for i in range(n):
            # Rotation matrix of quadrotor i wrt earth
            r_i = rotation_matrix(rpy_i[t][0+i:3+i])
            assert r_i.shape == (3,3), "Error, rotation matrix not 3x3 (r_i={r_i})"


            #########################
            # Kinematic Constraints #
            #########################
            
            # Rope must remain taut
            vector = X_i[t][3*i:3*i+3] - (XL + anchor_points[i])
            distance = np.linalg.norm(vector)
            prog.AddConstraint(distance == rope_len)

            # Input constraints
            prog.AddLinearConstraint(u[t], [0]*len(u[t]), [umax] * len(u[t]))

            ###############################################
            # Deone Velocity and acceleration constraints #
            ###############################################

            # Linear
            vel = (X_i[t] - X_i[t+1])/dt
            accel = (Xdot_i[t] - Xdot_i[t+1])/dt
            lhs_v = vel.tolist()
            rhs_v = Xdot_i[t].tolist()
            lhs_a = accel.tolist()
            rhs_a = Xddot_i[t].tolist()
            for j in range(len(rhs_v)):
                prog.AddConstraint(lhs_v[j] == rhs_v[j])
                prog.AddConstraint(lhs_a[j] == rhs_a[j])
            
            # Angular
            avel = (rpy_i[t] - rpy_i[t+1])/dt
            aaccel = (Omega_i[t] - Omega_i[t+1])/dt
            lhs_av = avel.tolist()
            rhs_av = Omega_i[t].tolist()
            lhs_aa = aaccel.tolist()
            rhs_aa = Omegadot_i[t].tolist()
            for j in range(len(rhs_av)):
                prog.AddConstraint(lhs_av[j] == rhs_av[j])
                prog.AddConstraint(lhs_aa[j] == rhs_aa[j])
            
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
            tension += r_L.dot(Tiqi[t][3*i:3*i+3])
        rhs_f = -tension - m_L*g*e3
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

        deltaU = u[t+1] - u[t]
        u_jerk = 3
        prog.AddLinearConstraint(deltaU, [-u_jerk]*len(deltaU), [u_jerk]*len(deltaU))

        #####################################
        #              Lambda               #
        #####################################
        # T12
        # T13
        # T32
        # Need to hard-code lambdas in for different numbers of quadcopters
            # [Tx_0    Ty_0    Tz_0   Tx_1 ... Tx_n   Ty_n   Tz_n]
            #   0        1       2      3  ... 3n+0   3n+1   3n+2
        if (n == 3):
            prog.AddConstraint(lambda_m[t][i] == Tiqi[t][3*0+0]) # x of copter 0
            prog.AddConstraint(lambda_m[t][i] == Tiqi[t][3*0+1]) # y of copter 0
            prog.AddConstraint(lambda_m[t][i] == Tiqi[t][3*1+0]) # x of copter 1


    # END OF LOOP THRU EACH TIMESTEP

    # solve mathematical program with initial guess
    ##############################################################################################################################
    # zpp --> [xload, yload, zload, rload, pload, yawLoad, lambda_0, ..., lambda_j, ...,lambda_(3n-5), psi_0, ...,  psi_j, ...   #
    #            0      1      2      3      4       5       6+0     ...,  6+j,     ...,  3n-1           3n   ...,  3n+j , ...   #
    ##############################################################################################################################
    
    dist_offset = np.matrix([[0,  0,  rope_len],
                        [0,  0,  rope_len],
                        [0,  0,  rope_len]])




    cache_file = "n_rotor_states.npy"
    # Solve for trajectory
    if (os.path.exists(cache_file)):
        print(f" loading initial guess from file {cache_file}")
        with open(cache_file, 'rb') as f:
            data = pickle.load(f)
        prog.SetInitialGuessForAllVariables(data)
    result = Solve(prog)
    assert result.is_success()
    pickle.dump( result.GetSolution(), open( cache_file, "wb" ) )
    print(f" saving initial guess to file {cache_file}")
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


    if not(good):
        print("DYNAMICS SOLVER FAILED")
    return x_L_out,x_i_out, R_L_out,rpy_i_out,    x_dot_i_out,x_dot_L_out, omega_i_out,Omega_L_out,   u_out, Tiqi_out, t_array





if __name__ == "__main__":

    timesteps = 50
    dt = .1
    tf = timesteps*dt
    zpp = circle_example_n_rotors(n=3, degree=6, continuity_degree=4, 
            discretization_samples=timesteps, diff_solver_samples=7, tf=tf)
    x_L,x_i, r_L,rpy_i, x_dot_i,x_dot_L, omega_i, Omega_L, u_out, Tiqi, t_array = solve_for_states_n_rotors(zpp, 
                                                                                    3, tf=tf, timesteps=timesteps)
    
    print("XL")
    plot_trajectories(x_L)
    print("Xi")
    plot_trajectories(x_i)
    print("x0")
    plot_trajectories(x_i[:,0:3])
    print("x1")
    plot_trajectories(x_i[:,3:6])
    print("x2")
    plot_trajectories(x_i[:,6:9])
    print("rpy_i")
    plot_trajectories(rpy_i)
    print("rpy_0")
    plot_trajectories(rpy_i[:,0:3])
    print("TiQi")
    plot_trajectories(Tiqi)
    print("u")
    plot_trajectories(u_out)
    