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