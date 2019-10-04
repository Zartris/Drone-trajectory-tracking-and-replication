import numpy as np
from scipy.signal import cont2discrete as c2d
import scipy.io as sio


class params(object):
    ## Parameters definition
    def __init__(self):
        # model parameters
        self.m = 1.5  # mass of the quadcopter (Kg) 0.468
        self.g = 9.8  # gravity (m/sec^2)
        self.kx = 0.25  # drag force coefficents(kg/sec)
        self.ky = 0.25  # drag force coefficents(kg/sec)
        self.kz = 0.25  # drag force coefficents(kg/sec)

        # dynamics parameters
        self.Ts = 0.1  # sampling time (both of MPC and simulated vehicle)
        self.nstates = 9  # number of states (x,y,z,)
        self.ninputs = 4  # number of inputs (x_v, y_v, z_v, yaw)
        # self.avoid_stat_obst = 1  # 0 don't avoid static obst, 1 otherwise
        # self.avoid_dynm_obst = 1  # 0 don't avoid dynamic obst, 1 otherwise
        # self.activate_obst = 1  # 0 if there are no obstacle, 1 otherwise
        # self.obst_rad = 0.5  # spherical radius of the obstacle

        self.obst_centers = np.array([[0, 0, 0],
                                      [3, 4, 5],
                                      [3, 0, 0]])  # x, y and z coordinates of the all region of interests

        # self.activate_quad = 1  # 0 if there are no obstacle, 1 otherwise
        self.act_term_cnst = 1

        self.startindx = 0  # index showing the start position of the quad.
        self.endindx = 1  # index showing the end position of the quad.

        # self.nquads = 2

        # self.quad_centers   = np.zeros((self.nquads,3))

        # for i in range(0,self.nquads-1):
        # 	self.quad_centers[i] = self.obst_centers[self.startindx[i]]

        self.quad_rad = 0.35  # spherical radius of the quadcopter

        self.n_stat_obst = len(self.obst_centers) - 2  # number of static obstacles
        self.n_dynam_obst = self.nquads - 2  # number of dynamic obstacles

        self.init_pos = np.array([[0, 0, 0, 0, 0, 0, 0, 0, 0],
                                  [0, 0, 0, 0, 0, 0, 0, 0, 0]])

        self.nobst = self.n_stat_obst + self.n_dynam_obst
        self.nlogics = self.nobst * 7 + 1
        if self.nobst == 1:
            self.nlogics = self.nlogics - 1

        # limits of the states and control
        self.u_min = -1
        self.u_max = 1
        self.angle_max = np.pi / 2
        self.angle_min = -np.pi / 2

        self.N = 15

        self.states_max = np.transpose(np.array([30, 30, 30, 5, 5, 5, 5, np.pi, 30]))
        self.input_max = np.transpose(np.array([1, 1, 1, np.pi]))

        # define the penalty matrices for the MPC
        self.Q1 = np.identity(self.nstates)
        self.Q2 = self.Q1
        # self.Q3 = np.identity(self.nlogics)
        self.R = 0.00001 * np.identity(self.ninputs)


class DynModel(object):
    def __init__(self, params):
        # define the state transition and input matrices for SS model
        self.A = np.zeros((params.nstates, params.nstates))
        self.B = np.zeros((params.nstates, params.ninputs))
        self.C = np.zeros((params.nstates))
        self.D = np.zeros((params.ninputs))

        self.A[0][3] = 1
        self.A[1][4] = 1
        self.A[2][5] = 1
        self.A[3][3] = -params.kx / params.m
        self.A[4][4] = -params.ky / params.m
        self.A[5][5] = -params.kz / params.m
        self.A[7][6] = 1
        self.A[8][2] = 1

        self.B[3][0] = 1 / params.m
        self.B[4][1] = -1 / params.m
        self.B[5][2] = 1 / params.m
        self.B[6][3] = 1

        # convert the continuous SS model to Discrete SS model
        self.Ad, self.Bd, self.Cd, self.Dd, self.dt = c2d((self.A, self.B, self.C, self.D), params.Ts, method='zoh')

        # Split the system into two matrices,
        # 	1 ==> Obstacle area, x(k+1) = I*x(k)
        #	2 ==> Normal area, x(k+1) = A*x(k) + Bu()
        self.A1 = np.identity(params.nstates)
        self.B1 = np.zeros((params.nstates, params.ninputs))

        self.A2 = self.Ad
        self.B2 = self.Bd
        self.C2 = self.Cd


class MLD(object):
    def __init__(self, params, model):

        I = np.identity(params.nstates)
        self.M = params.states_max

        self.M1 = self.M
        # M1					= np.dot(model.A1,params.states_max) + np.dot(model.B1,params.input_max)
        self.M2 = np.add(np.dot(model.A2, params.states_max), np.dot(model.B2, params.input_max))
        self.Big_M = 50 * np.ones((6, 1))
        m = -50 * np.identity(6)
        ZER = np.zeros((params.nstates, params.ninputs))
        ZEROS = np.zeros((params.nstates, params.nstates))

        d_bin1 = np.array([[-1, 1, 1, 1, 1, 1, 1], [1, -1, 0, 0, 0, 0, 0],
                           [1, 0, -1, 0, 0, 0, 0], [1, 0, 0, -1, 0, 0, 0],
                           [1, 0, 0, 0, -1, 0, 0], [1, 0, 0, 0, 0, -1, 0],
                           [1, 0, 0, 0, 0, 0, -1], [0, -1, 0, 0, -1, 0, 0],
                           [0, 0, -1, 0, 0, -1, 0], [0, 0, 0, -1, 0, 0, -1]])
        self.d_bin2 = np.transpose(np.array([5, 0, 0, 0, 0, 0, 0, -1, -1, -1]))
        # E2_temp1 = np.zeros((22 * params.nobst, params.nlogics))
        # E2_temp2 = np.zeros((4 * params.nstates, params.nlogics))
        # E2_temp3 = np.zeros((params.nobst + 1, params.nlogics))
        # E4_temp1 = np.zeros((22 * params.nobst, params.nstates))

        self.P = np.zeros((6, params.nstates))
        self.P[0:3, 0:3] = np.identity(3)
        self.P[3:6, 0:3] = -np.identity(3)

        # d_f = np.ones((params.nlogics, 1))
        # d_f[0:4, 0:1] = np.zeros((4, 1))

        # for i in range(params.nobst):
        #     E2_temp1[22 * i:22 * i + 6, i:i + 1] = self.Big_M
        #
        #     if params.nobst == 1:
        #         E2_temp1[22 * i + 6:22 * i + 12, 6 * i + params.nobst:6 * (i + 1) + params.nobst] = m
        #         E2_temp1[22 * i + 12:22 * i + 22, 6 * i + params.nobst:6 * (i + 1) + params.nobst] = d_bin1[:, 1:7]
        #     else:
        #         E2_temp1[22 * i + 6:22 * i + 12, 6 * i + params.nobst + 1:6 * (i + 1) + params.nobst + 1] = m
        #         E2_temp1[22 * i + 12:22 * i + 22, 6 * i + params.nobst + 1:6 * (i + 1) + params.nobst + 1] = d_bin1[:,
        #                                                                                                      1:7]
        #
        #     E2_temp1[22 * i + 12:22 * (i + 1), i:i + 1] = d_bin1[:, 0:1]
        #     # E4_temp1[22*i:22*i+6,:] = -P
        #     # E4_temp1[22*i+6:22*i+12,:] = P
        #     E4_temp1[22 * i:22 * i + 12, :] = np.concatenate((-self.P, self.P))

        # print(E4_temp1)
        # if params.nobst == 1:
        #     self.pos_d = params.nobst - 1
        # else:
        #     self.pos_d = params.nobst
        # E2_temp2[:, self.pos_d] = np.concatenate((self.M1, self.M1, -self.M2, -self.M2))
        # # print(np.concatenate((M1, M1,-M2,-M2)))
        #
        # if params.nobst == 2:
        #     E2_temp3[0:params.nobst + 1, 0:params.nobst + 1] = np.array([[1, 0, -1],
        #                                                                  [0, 1, -1],
        #                                                                  [-1, -1, 1]])
        # elif params.nobst == 3:
        #     E2_temp3[0:params.nobst + 1, 0:params.nobst + 1] = np.array([[1, 0, 0, -1],
        #                                                                  [0, 1, 0, -1],
        #                                                                  [0, 0, 1, -1],
        #                                                                  [-1, -1, -1, 1]])
        # elif params.nobst == 4:
        #     E2_temp3[0:params.nobst + 1, 0:params.nobst + 1] = np.array([[1, 0, 0, 0, -1],
        #                                                                  [0, 1, 0, 0, -1],
        #                                                                  [0, 0, 1, 0, -1],
        #                                                                  [0, 0, 0, 1, -1],
        #                                                                  [-1, -1, -1, -1, 1]])
        # else:
        #     E2_temp3 = E2_temp3
        #
        # self.E1 = np.concatenate((ZER, ZER, np.zeros((22 * params.nobst, params.ninputs)),
        #                           -model.B1, model.B1, -model.B2, model.B2,
        #                           np.zeros((params.nobst + 1, params.ninputs))))
        # self.E2 = np.concatenate((np.zeros((2 * params.nstates, params.nlogics)), E2_temp1, E2_temp2, E2_temp3))
        # self.E3 = np.concatenate((-I, I, np.zeros((22 * params.nobst, params.nstates)), -I, I, -I, I,
        #                           np.zeros((params.nobst + 1, params.nstates))))
        # self.E4 = np.concatenate((ZEROS, ZEROS, E4_temp1, -model.A1, model.A1, -model.A2, model.A2,
        #                           np.zeros((params.nobst + 1, params.nstates))))
        # self.E5 = np.zeros((len(self.E1), 1))
        #
        # # save the matrices in the .mat file
        # sio.savemat('E1.mat', {'vect': self.E1})
        # sio.savemat('E2.mat', {'vect': self.E2})
        # sio.savemat('E3.mat', {'vect': self.E3})
        # sio.savemat('E4.mat', {'vect': self.E4})
