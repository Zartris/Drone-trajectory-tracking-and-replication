from cvxpy import *
import numpy as np
from init_params import *
import picos as pic
from gurobipy import *

# define the objects
mp = params()
model = DynModel(mp)
mld = MLD(mp, model)

N = 10  # horizon length
N_max = 100

# variables definition here
x = Variable(mp.nstates, N + 1)
z = Variable(mp.nstates, N + 1)
u = Variable(mp.ninputs, N)
d = Bool(mp.nlogics, N)

# penalize the logical varables for obstacles to zeros
d_f = np.ones((mp.nlogics, 1))
d_f[0:4, 0:1] = np.zeros((4, 1))

# Parameters definition here
x_0 = Parameter(mp.nstates, 1)
x_r = Parameter(mp.nstates, 1)
E5 = Parameter(len(mld.E1), 1)
goal_r = Parameter(6, 1)


# for testing purpose
# x_init = Variable(mp.nstates,1)
# x_ref = Variable(mp.nstates,1)
# Eaff = Variable(len(mld.E1),1)
# goal_ref = variable(6,1)


class TrajectoryPlanner:
    def __init__(self):
        a = 0

    def MPC(self):
        # problem formulation of hybrid mpc
        states = []
        for t in range(N):
            cost = quad_form((x[:, t + 1] - x_r), mp.Q1) + quad_form((u[:, t]), mp.R)

            constr = [x[:, t + 1] == model.A2 * x[:, t] + model.B2 * u[:, t],

                      # constarints on the inputs
                      mp.u_min <= u[0, t], u[0, t] <= mp.u_max,
                      mp.u_min <= u[1, t], u[1, t] <= mp.u_max,
                      mp.u_min <= u[2, t], u[2, t] <= mp.u_max,
                      -np.pi <= u[3, t], u[3, t] <= np.pi]

            states.append(Problem(Minimize(cost), constr))

        if (mp.act_term_cnst):
            constr = [mld.P * x[:, N] <= goal_r]

        prob = sum(states)
        prob.constraints += [x[:, 0] == x_0]
        return prob

    def HMPC(self):
        # problem formulation of hybrid mpc
        states = []
        for t in range(N):
            q1 = quad_form((x[:, t + 1] - x_r), mp.Q1)
            print(x[:, t + 1])
            q2 = quad_form((z[:, t + 1] - x_r), mp.Q2)
            print(d[:, t])
            print(d_f)
            p1 = (d[:, t] - d_f)
            q3 = quad_form((d[:, t] - d_f), mp.Q3)
            q4 = quad_form((u[:, t]), mp.R)
            cost = quad_form((x[:, t + 1] - x_r), mp.Q1) + quad_form((z[:, t + 1] - x_r), mp.Q2) + quad_form(
                (d[:, t] - d_f), mp.Q3) + quad_form((u[:, t]), mp.R)
            # cost = quad_form((x[:, t+1] - x_r),mp.Q1) + quad_form((w[:, t]),mp.R)
            constr = [x[:, t + 1] == z[:, t],
                      mld.E2 * d[:, t] + mld.E3 * z[:, t] <= mld.E1 * u[:, t] + mld.E4 * x[:, t] + E5,
                      # constarints on the inputs
                      # x[:,t+1] == model.A2*x[:,t] + model.B2*w[:,t],
                      mp.u_min <= u[0, t], u[0, t] <= mp.u_max,
                      mp.u_min <= u[1, t], u[1, t] <= mp.u_max,
                      mp.u_min <= u[2, t], u[2, t] <= mp.u_max,
                      -np.pi <= u[3, t], u[3, t] <= np.pi]

            states.append(Problem(Minimize(cost), constr))

        if (mp.act_term_cnst):
            constr = [mld.P * x[:, N] <= goal_r]

        prob = sum(states)
        prob.constraints += [x[:, 0] == x_0]
        return prob

    def HMPC_test(self, x_ref, goal_ref):
        # problem formulation of hybrid mpc
        states = []
        for t in range(N):
            cost = quad_form((x[:, t + 1] - x_ref), mp.Q1) + quad_form((z[:, t + 1] - x_ref), mp.Q2) + quad_form(
                (d[:, t] - d_f), mp.Q3) + quad_form((u[:, t]), mp.R)
            # cost = quad_form((x[:, t+1] - x_r),mp.Q1) + quad_form((w[:, t]),mp.R)
            constr = [x[:, t + 1] == z[:, t],
                      mld.E2 * d[:, t] + mld.E3 * z[:, t] <= mld.E1 * u[:, t] + mld.E4 * x[:, t] + mld.E5,
                      # constarints on the inputs
                      # x[:,t+1] == model.A2*x[:,t] + model.B2*w[:,t],
                      mp.u_min <= u[0, t], u[0, t] <= mp.u_max,
                      mp.u_min <= u[1, t], u[1, t] <= mp.u_max,
                      mp.u_min <= u[2, t], u[2, t] <= mp.u_max,
                      -np.pi <= u[3, t], u[3, t] <= np.pi]

            states.append(Problem(Minimize(cost), constr))

        # if(mp.act_term_cnst):
        # 	constr = [mld.P*x[:,N] <= goal_ref]

        prob = sum(states)
        prob.constraints += [x[:, 0] == x_0]
        # prob.constraints += [z[:,0] == x_0]
        return prob

    def GetStaticObstPos(self):

        # Determine the position of obstacles based on the start and end indx of quad
        stat_obst_pos = np.zeros((mp.n_stat_obst, 3))
        indx_array = np.zeros((mp.n_stat_obst, 1))

        i = 0
        for k in range(len(mp.obst_centers)):
            if (k != mp.startindx and k != mp.endindx):
                indx_array[i] = k
                i += 1
        for k in range(len(indx_array)):
            stat_obst_pos[k][:] = mp.obst_centers[int(indx_array[k])]

        return stat_obst_pos

    def CalculateE5Static(self):
        static_ref = np.zeros((6, mp.n_stat_obst))
        E5_static = np.zeros((22 * mp.n_stat_obst, 1))

        l_st_obs = mp.obst_rad + mp.quad_rad
        l_dy_obs = 2 * mp.quad_rad

        # Get the positions of static obstacles
        stat_obst_pos = self.GetStaticObstPos()

        for i in range(mp.n_stat_obst):
            static_ref[:, i] = np.array((l_st_obs + int(stat_obst_pos[i][0]),
                                         l_st_obs + int(stat_obst_pos[i][1]),
                                         l_st_obs + int(stat_obst_pos[i][2]),
                                         l_st_obs - int(stat_obst_pos[i][0]),
                                         l_st_obs - int(stat_obst_pos[i][1]),
                                         l_st_obs - int(stat_obst_pos[i][2])))
            E5_static[22 * i:22 * (i + 1)] = np.concatenate(
                (np.add(mld.Big_M.reshape((6, 1)), static_ref[:, i].reshape((6, 1))),
                 -static_ref[:, i].reshape((6, 1)), mld.d_bin2.reshape((10, 1))))
        return E5_static

    def GetDynamObstPos(self):
        dynam_obst_pos = np.zeros((mp.n_dynam_obst, 3))
        # code starts here
        for i in range(mp.n_dynam_obst):
            dynam_obst_pos[i, :] = np.array((20, 20, 20))

        return dynam_obst_pos

    def CalculateE5Dynam(self):

        l_dy_obs = 2 * mp.quad_rad;
        dynamic_ref = np.zeros((6, mp.n_dynam_obst))
        dynam_obst_pos = self.GetDynamObstPos()
        E5_dynam = np.zeros((22 * mp.n_dynam_obst, 1))
        # code starts here

        for i in range(mp.n_dynam_obst):
            dynamic_ref[:, i] = np.array((l_dy_obs + int(dynam_obst_pos[i][0]),
                                          l_dy_obs + int(dynam_obst_pos[i][1]),
                                          l_dy_obs + int(dynam_obst_pos[i][2]),
                                          l_dy_obs - int(dynam_obst_pos[i][0]),
                                          l_dy_obs - int(dynam_obst_pos[i][1]),
                                          l_dy_obs - int(dynam_obst_pos[i][2])))
            E5_dynam[22 * i:22 * (i + 1)] = np.concatenate(
                (np.add(mld.Big_M.reshape((6, 1)), dynamic_ref[:, i].reshape((6, 1))),
                 -dynamic_ref[:, i].reshape((6, 1)), mld.d_bin2.reshape((10, 1))))

        return E5_dynam

    def SimulateQuad(self, x, u):

        # Also add the integral part of the Discrete SS model
        zd = mp.obst_centers[mp.endindx, 2]
        DInt = np.array([0, 0, 0, 0, 0, 0, 0, 0, -mp.Ts])
        # print(DInt)
        z_int = DInt * zd
        print(model.A2 * x + model.B2.dot(u.reshape((4, 1))) + z_int.reshape((9, 1)))
        return model.A2 * x + model.B2.dot(u.reshape((4, 1))) + z_int.reshape((9, 1))

    def run(self):
        # while not rospy.is_shutdown():
        # pos_init = np.transpose(np.array([0,0,0,0,0,0,0,0,0]))
        # x_0.value = np.transpose(np.array([0,0,0,0,0,0,0,0,0]))
        # based on the initial poition of quad define the MPC controller
        my_prob = self.HMPC()
        # print(my_prob)

        # state the reference points
        x_r.value = np.concatenate(
            (mp.obst_centers[mp.endindx], np.array([0, 0, 0, 0, np.pi / 4, int(mp.obst_centers[mp.endindx, 2])])))
        # print(x_r.value)

        # define the goal_ref for terminal constraints i-e solve the polyhedral set
        s = 0.2
        l_r = mp.obst_rad + s
        goal_r.value = np.array([l_r + int(mp.obst_centers[mp.endindx, 0]),
                                 l_r + int(mp.obst_centers[mp.endindx, 1]),
                                 l_r + int(mp.obst_centers[mp.endindx, 2]),
                                 l_r - int(mp.obst_centers[mp.endindx, 0]),
                                 l_r - int(mp.obst_centers[mp.endindx, 1]),
                                 l_r - int(mp.obst_centers[mp.endindx, 2])])

        E5_static = self.CalculateE5Static()
        # print(E5_static.shape)

        # define the variable for simulation
        z = np.zeros((N_max, mp.nstates))
        u = np.zeros((N_max, mp.ninputs))

        # print(my_prob)

        # loop starts here
        count = 0
        while (count < N_max - 1):
            x_0.value = z[count, :]

            # Get the dynamic part of affine matrix of MLD system
            E5_dynam = self.CalculateE5Dynam()
            E5.value = np.concatenate((mld.M.reshape((mp.nstates, 1)), mld.M.reshape((mp.nstates, 1)),
                                       E5_static.reshape((22 * mp.n_stat_obst, 1)),
                                       E5_dynam.reshape((22 * mp.n_dynam_obst, 1)),
                                       mld.M1.reshape((mp.nstates, 1)), mld.M1.reshape((mp.nstates, 1)),
                                       np.zeros((2 * mp.nstates, 1)), np.zeros((mp.nobst + 1, 1))))
            # print(abc.shape)

            # print(mld.E2.shape)

            # solve the receding horizon problem
            my_prob.solve(solver=GUROBI)
            count = count + 1
            # simulate the system
            z[count, :] = self.SimulateQuad(x.value[:, 0], u.transpose()[:, 0]).reshape((1, 9))
            print("x:")
            # print(x.value)
            print(z[count, 0:3])

    # for testing purpose

    def test(self):

        x_ref = np.concatenate(
            (mp.obst_centers[mp.endindx], np.array([0, 0, 0, 0, np.pi / 4, int(mp.obst_centers[mp.endindx, 2])])))

        s = 0.2
        l_r = mp.obst_rad + s
        goal_ref = np.array([l_r + int(mp.obst_centers[mp.endindx, 0]),
                             l_r + int(mp.obst_centers[mp.endindx, 1]),
                             l_r + int(mp.obst_centers[mp.endindx, 2]),
                             l_r - int(mp.obst_centers[mp.endindx, 0]),
                             l_r - int(mp.obst_centers[mp.endindx, 1]),
                             l_r - int(mp.obst_centers[mp.endindx, 2])])

        E5_static = self.CalculateE5Static()
        E5_dynam = self.CalculateE5Dynam()
        mld.E5 = np.concatenate((mld.M.reshape((mp.nstates, 1)), mld.M.reshape((mp.nstates, 1)),
                                 E5_static.reshape((22 * mp.n_stat_obst, 1)),
                                 E5_dynam.reshape((22 * mp.n_dynam_obst, 1)),
                                 mld.M1.reshape((mp.nstates, 1)), mld.M1.reshape((mp.nstates, 1)),
                                 np.zeros((2 * mp.nstates, 1)), np.zeros((mp.nobst + 1, 1))))

        # print(mld.E2.shape)

        my_prob = self.HMPC_test(x_ref, goal_ref)

        # define the variable for simulation
        z = np.zeros((N_max, mp.nstates))
        u = np.zeros((N_max, mp.ninputs))

        count = 0

        while (count < N_max - 1):
            x_0.value = z[count, :]

            my_prob.solve(solver=GUROBI)
            count = count + 1
            # simulate the system
            z[count, :] = self.SimulateQuad(x.value[:, 0], u[:, 0]).reshape((1, 9))
            print("x:")
            # print(x.value)
            print(z[count, 0:3])


if __name__ == "__main__":
    # rospy.init_node("Trajectory Planner")
    trajplanner = TrajectoryPlanner()
    trajplanner.run()
# trajplanner.test()
