
import numpy as np
from PPTrajectory import PPTrajectory



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

