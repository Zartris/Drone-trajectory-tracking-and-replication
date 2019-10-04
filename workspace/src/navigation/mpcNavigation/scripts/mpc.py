import cvxpy as cp
import numpy as np
from init_params import *
import picos as pic
from gurobipy import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import *
from tf.msg import *
import matplotlib.pyplot as plt
from math import *
import time

# define the objects
mp = params()
model = DynModel(mp)
mld = MLD(mp, model)

N = 5  # horizon length
N_max = 1000

# variables definition here
x = cp.Variable(shape=(mp.nstates, N + 1))
z = cp.Variable(shape=(mp.nstates, N + 1))
u = cp.Variable(shape=(mp.ninputs, N))  # u[0] = x velocity, u[1] = y velocity, u[2] = z velocity, u[3] = yaw
d = cp.Variable(shape=(mp.nlogics, N))

# penalize the logical varables for obstacles to zeros
d_f = np.ones((mp.nlogics, 1))
d_f[0:4, 0:1] = np.zeros((4, 1))

# Parameters definition here
x_0 = cp.Parameter(mp.nstates, 1)
x_r = cp.Parameter(mp.nstates, 1)
E5 = cp.Parameter(1, 1)
goal_r = cp.Parameter(6, 1)


class MPCNavigator:
    def __init__(self):
        pass

    def MPC(self):
        states = []
        for t in range(N):
            print("start", x[:, t + 1] - x_r)
            print("Q1", mp.Q1)
            q = cp.quad_form((x[:, t + 1] - x_r), mp.Q1)
            print("bla", q)
            cost = cp.quad_form((x[:, t + 1] - x_r), mp.Q1) + cp.quad_form((u[:, t]), mp.R)

            constr = [x[:, t + 1] == model.A2 * x[:, t] + model.B2 * u[:, t],
                      # constarints on the inputs
                      mp.u_min <= u[0, t], u[0, t] <= mp.u_max,
                      mp.u_min <= u[1, t], u[1, t] <= mp.u_max,
                      mp.u_min <= u[2, t], u[2, t] <= mp.u_max,
                      -np.pi <= u[3, t], u[3, t] <= np.pi]

            states.append(cp.Problem(cp.Minimize(cost), constr))

        if (mp.act_term_cnst):
            constr = [mld.P * x[:, N] <= goal_r]

        prob = sum(states)
        cnst = prob.constraints + [x[:, 0] == x_0]
        prob2 = cp.Problem(prob.objective, cnst)

        return prob2

    def polyFit(self, ts, xs, ys, zs):
        """
        :ivar
        ts : timestamps
        xs : x coordinates of path (in camera_coords)
        ys : y coordinates of path (in camera_coords)
        zs : z coordinates of path (in camera_coords)
        :rtype: functions for x,y,z from a timestamp.
        """
        degree = 3

        # Set up the canonical least squares form
        Ats = np.vander(ts, degree)

        # Solve for a least squares estimate
        (xcoeffs, _, _, _) = np.linalg.lstsq(Ats, xs)
        (ycoeffs, _, _, _) = np.linalg.lstsq(Ats, ys)
        (zcoeffs, _, _, _) = np.linalg.lstsq(Ats, zs)

        # Extract coefficients and create polynomials in x and y
        fx = np.poly1d(xcoeffs)
        fy = np.poly1d(ycoeffs)
        fz = np.poly1d(zcoeffs)

        return fx, fy, fz


    # def GetStaticObstPos(self):
    #
    #     # Determine the position of obstacles based on the start and end indx of quad
    #     stat_obst_pos = np.zeros((mp.n_stat_obst, 3))
    #     indx_array = np.zeros((mp.n_stat_obst, 1))
    #
    #     i = 0
    #     for k in range(len(mp.obst_centers)):
    #         if (k != mp.startindx and k != mp.endindx):
    #             indx_array[i] = k
    #             i += 1
    #     for k in range(len(indx_array)):
    #         stat_obst_pos[k][:] = mp.obst_centers[int(indx_array[k])]
    #
    #     return stat_obst_pos

    # def CalculateE5Static(self):
    #     static_ref = np.zeros((6, mp.n_stat_obst))
    #     E5_static = np.zeros((22 * mp.n_stat_obst, 1))
    #
    #     l_st_obs = mp.obst_rad + mp.quad_rad
    #     l_dy_obs = 2 * mp.quad_rad
    #
    #     # Get the positions of static obstacles
    #     stat_obst_pos = self.GetStaticObstPos()
    #
    #     for i in range(mp.n_stat_obst):
    #         static_ref[:, i] = np.array((l_st_obs + int(stat_obst_pos[i][0]),
    #                                      l_st_obs + int(stat_obst_pos[i][1]),
    #                                      l_st_obs + int(stat_obst_pos[i][2]),
    #                                      l_st_obs - int(stat_obst_pos[i][0]),
    #                                      l_st_obs - int(stat_obst_pos[i][1]),
    #                                      l_st_obs - int(stat_obst_pos[i][2])))
    #         E5_static[22 * i:22 * (i + 1)] = np.concatenate(
    #             (np.add(mld.Big_M.reshape((6, 1)), static_ref[:, i].reshape((6, 1))),
    #              -static_ref[:, i].reshape((6, 1)), mld.d_bin2.reshape((10, 1))))
    #     return E5_static

    # def GetDynamObstPos(self):
    #     dynam_obst_pos = np.zeros((mp.n_dynam_obst, 3))
    #     # code starts here
    #     for i in range(mp.n_dynam_obst):
    #         dynam_obst_pos[i, :] = np.array((20, 20, 20))
    #
    #     return dynam_obst_pos

    # def CalculateE5Dynam(self):
    #
    #     l_dy_obs = 2 * mp.quad_rad
    #     dynamic_ref = np.zeros((6, mp.n_dynam_obst))
    #     dynam_obst_pos = self.GetDynamObstPos()
    #     E5_dynam = np.zeros((22 * mp.n_dynam_obst, 1))
    #     # code starts here
    #
    #     for i in range(mp.n_dynam_obst):
    #         dynamic_ref[:, i] = np.array((l_dy_obs + int(dynam_obst_pos[i][0]),
    #                                       l_dy_obs + int(dynam_obst_pos[i][1]),
    #                                       l_dy_obs + int(dynam_obst_pos[i][2]),
    #                                       l_dy_obs - int(dynam_obst_pos[i][0]),
    #                                       l_dy_obs - int(dynam_obst_pos[i][1]),
    #                                       l_dy_obs - int(dynam_obst_pos[i][2])))
    #         E5_dynam[22 * i:22 * (i + 1)] = np.concatenate(
    #             (np.add(mld.Big_M.reshape((6, 1)), dynamic_ref[:, i].reshape((6, 1))),
    #              -dynamic_ref[:, i].reshape((6, 1)), mld.d_bin2.reshape((10, 1))))
    #
    #     return E5_dynam

    def SimulateQuad(self, x1, u1):
        # Also add the integral part of the Discrete SS model
        zd = mp.obst_centers[mp.endindx, 2]
        DInt = np.array([0, 0, 0, 0, 0, 0, 0, 0, -mp.Ts])
        # print(DInt)
        z_int = DInt * zd
        # print("x1",x1.reshape(9,1))
        # print("A", model.A2)
        # print("Ax *",model.A2 * x1.reshape(9,1))
        Ax = model.A2.dot(x1.reshape(9, 1))
        # print("Ax dot",Ax, np.shape(Ax))
        Bu = model.B2.dot(u1.reshape((4, 1)))
        # print("Bu",Bu, np.shape(Bu))
        AxBu = Ax + Bu
        # print("AxBu",AxBu, np.shape(AxBu))
        # print("z",z_int.reshape((9, 1)))

        print(AxBu + z_int.reshape((9, 1)))
        return AxBu + z_int.reshape((9, 1))

    def CalcInput(self, A, B, C, x, u, target):
        x_0 = x[:]
        x = cp.Variable((x.shape[0], N + 1))
        u = cp.Variable((u.shape[0], N))

        # MPC controller
        states = []
        for t in range(N):
            constr = [x[:, t + 1] == A * x[:, t] + B * u[:, t] + C.reshape(4, )]
            constr += [abs(u[:, t]) <= 1]
            constr += [x[2, t + 1] <= 1]  # Max speed
            constr += [x[2, t + 1] >= -1]  # Min speed

            #  cost = sum_squares(u[:,t])
            cost = cp.sum_squares(abs(x[0, t] - target[0])) * 10.0 * t
            cost += cp.sum_squares(abs(x[1, t] - target[1])) * 10.0 * t
            cost += cp.sum_squares(abs(x[2, t] - target[2])) * 10.0 * t

            if t == N - 1:
                cost += (x[0, t + 1] - target[0]) ** 2 * 10000.0
                cost += (x[1, t + 1] - target[1]) ** 2 * 10000.0
                cost += (x[2, t + 1] - target[2]) ** 2 * 10000.0


            cost = cp.quad_form((x[:, t + 1] - x_r), mp.Q1) + cp.quad_form((u[:, t]), mp.R)

            constr = [x[:, t + 1] == model.A2 * x[:, t] + model.B2 * u[:, t],
                      # constarints on the inputs
                      mp.u_min <= u[0, t], u[0, t] <= mp.u_max,
                      mp.u_min <= u[1, t], u[1, t] <= mp.u_max,
                      mp.u_min <= u[2, t], u[2, t] <= mp.u_max,
                      -1 <= u[3, t], u[3, t] <= 1]

            states.append(cp.Problem(cp.Minimize(cost), constr))


        prob = sum(states)
        # add constraints and create new problem
        cnst = prob.constraints + [x[:, 0] == x_0.reshape(4, ), x[2, T] == 0.0]
        prob2 = cp.Problem(prob.objective, cnst)
        # prob.solve()
        # print(x[2, T] == 0.0)
        # prob.constraints += [x[:, 0] == x_0.reshape(4,), x[2, T] == 0.0]

        start = time.time()
        #  result=prob.solve(verbose=True)
        result = prob2.solve()
        elapsed_time = time.time() - start
        print("calc time:{0}".format(elapsed_time) + "[sec]")
        print(prob.value)

        if prob.status != cp.OPTIMAL:
            print("Cannot calc opt")

        #  print(prob.status)
        return u, x, prob.value

    def run(self):
        # while not rospy.is_shutdown():
        # pos_init = np.transpose(np.array([0,0,0,0,0,0,0,0,0]))
        # x_0.value = np.transpose(np.array([0,0,0,0,0,0,0,0,0]))
        # based on the initial poition of quad define the MPC controller
        my_prob = self.MPC()

        # define the variable for simulation
        z = np.zeros((N_max, mp.nstates))
        # u = np.zeros((N_max, mp.ninputs))

        # print(my_prob)

        # loop starts here
        count = 0
        while (count < N_max - 1):
            x_0.value = z[count, :]

            # print(abc.shape)

            # print(mld.E2.shape)

            # solve the receding horizon problem
            result = my_prob.solve(solver=cp.GUROBI)
            count = count + 1
            # simulate the system
            z[count, :] = self.SimulateQuad(x.value[:, 0], u.value[:, 0]).reshape((1, 9))
            print("x:")
            # print(x.value)
            print(z[count, 0:3])

    # def run2(self):
    #     x0 = np.array([[0.0, 0.0, 0.0, 0.0]]).T  # [x,y,v theta]
    #     print(x0)
    #     x = x0
    #     u = np.array([[0.0, 0.0, 0.0, 0.0]]).T  # [a,beta]
    #     plt.figure(num=None, figsize=(12, 12))
    #
    #     mincost = 1000
    #
    #     for i in range(1000):
    #         A, B, C = LinealizeCarModel(x, u, dt, lr)
    #         ustar, xstar, cost = CalcInput(A, B, C, x, u)
    #
    #         u[0, 0] = GetListFromMatrix(ustar.value[0, :])[0]
    #         u[1, 0] = float(ustar[1, 0].value)
    #
    #         x = np.matmul(A, x) + np.matmul(B, u)
    #
    #         plt.subplot(3, 1, 1)
    #         plt.plot(target[0], target[1], 'xb')
    #         plt.plot(x[0], x[1], '.r')
    #         plt.plot(GetListFromMatrix(xstar.value[0, :]), GetListFromMatrix(
    #             xstar.value[1, :]), '-b')
    #         plt.axis("equal")
    #         plt.xlabel("x[m]")
    #         plt.ylabel("y[m]")
    #         plt.grid(True)
    #
    #         plt.subplot(3, 1, 2)
    #         plt.cla()
    #         plt.plot(GetListFromMatrix(xstar.value[2, :]), '-b')
    #         plt.plot(GetListFromMatrix(xstar.value[3, :]), '-r')
    #         plt.ylim([-1.0, 1.0])
    #         plt.ylabel("velocity[m/s]")
    #         plt.xlabel("horizon")
    #         plt.grid(True)
    #
    #         plt.subplot(3, 1, 3)
    #         plt.cla()
    #         plt.plot(GetListFromMatrix(ustar.value[0, :]), '-r', label="a")
    #         plt.plot(GetListFromMatrix(ustar.value[1, :]), '-b', label="b")
    #         plt.ylim([-0.5, 0.5])
    #         plt.legend()
    #         plt.grid(True)
    #
    #         #  plt.pause(0.0001)
    #
    #         #  raw_input()
    #
    #         # check goal
    #         dis = np.linalg.norm([x[0] - target[0], x[1] - target[1]])
    #         if (dis < 0.1):
    #             print("Goal")
    #             break
    #
    #     plt.show()



if __name__ == "__main__":
    ts = range(0, 3)
    xs = [0, 1, 2]
    ys = [1, 1, 2]
    zs = [3, 2, 1]

    mpcNav = MPCNavigator()
    fx, fy, fz = mpcNav.polyFit(ts, xs, ys, zs)
    print(fx(1), fy(1), fz(1))

    mpcNav.run()

    # rospy.init_node("Trajectory Planner")
    # trajplanner = TrajectoryPlanner()
    # trajplanner.run()
