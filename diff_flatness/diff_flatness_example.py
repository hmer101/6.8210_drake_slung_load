from PPTrajectory import PPTrajectory

tf = 5
dof = 5
num_vars=dof
degree=6
continuity_degree=5

zpp = PPTrajectory(
    sample_times=np.linspace(0, tf, 6),
    num_vars=num_vars,
    degree=degree,
    continuity_degree=continuity_degree,
)

init_conditions = np.zeros(dof)

final_conditions = np.ones(dof)*3

# Init conditions
zpp.add_constraint(t=0, derivative_order=0, lb=init_conditions)
zpp.add_constraint(t=0, derivative_order=1, lb=np.zeros(dof))
zpp.add_constraint(t=0, derivative_order=2, lb=np.zeros(dof))

# Intermediate conditions
# zpp.add_constraint(t=1, derivative_order=0, lb=[2, 1.5, 0])

# zpp.add_constraint(t=2, derivative_order=0, lb=[4, 1, 0])


# Final conditions
zpp.add_constraint(t=tf, derivative_order=0, lb=final_conditions)
zpp.add_constraint(t=tf, derivative_order=1, lb=np.zeros(dof))
zpp.add_constraint(t=tf, derivative_order=2, lb=np.zeros(dof))


zpp.generate()

t = np.linspace(0, tf, 100)
z = np.zeros((dof, len(t)))

for i in range(len(t)):
    z[:, i] = zpp.eval(t[i])



if False:  # Useful for debugging
    t = np.linspace(0, tf, 100)
    z = np.zeros((dof, len(t)))
    knots = np.zeros((dof, len(zpp.sample_times)))
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
    
else:

    fig, ax = plt.subplots()
    PPTrajectory.show_traj(z)





# Solving for the remaining parameters
tcount = 10
g= 9.81
def solve_for_states(z):

    # x = zpp.eval(t) 
    # xddot = zpp.eval(t, 2)

    # xi = x[0:3] # quadrotor x,y,z
    # phi_i = x[3] # quadrotor phi
    # Tiqi = [0,0,0] # quadrotor tension vector (with direction)

    m_i = 1 #quadrotor mass
    J_i = np.eye(3)
    e3 = np.array([0,0,1]) # z vector

    # f_i = np.zeros(3) # output resultant force vector
    # M_i = np.zeros(3) # output moment vector

    prog = MathematicalProgram()
    # fi = prog.NewContinuousVariables(tcount, name="fi")
    fi = prog.NewContinuousVariables(tcount, 3, name="fi")
    Ri = prog.NewContinuousVariables(tcount, 3, name="Ri")
    Tiqi = prog.NewContinuousVariables(tcount, 3, name="Tiqi")


    for t in range(tcount):

        # Sum of forces
        lhs = m_i*zpp.eval(t,2)[0:3]
        rhs = np.dot(fi[t],np.dot(Ri[t],(e3))) - m_i*g*e3.T + Tiqi[t]
        diff = lhs-rhs
        prog.AddConstraint(lhs == rhs)
        
        # Sum of moments
        # Ji * Omegadot_i + Omega_i x Ji*Omegai = SumOfMoments_i
        # prog.AddLinearConstraint(J_i*zpp.eval())

        # fi*Ri.dot(e3) = m_i*xddot + m_i*g*e3 + Tiqi
    result = Solve(prog)
    good = result.is_success()
    print(good)
    return good

    print(result)


solve_for_states(z)