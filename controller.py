import numpy as np
import math

class min_ener_controller:
    def __init__(self):
        pass 

    def get_energy_cons(self):
        pass

    def cVec(self, B, R, lamda_dot, alpha, omega, R_dot, lamda, C, A, w2):
        val = (B.transpose() * np.linalg.inv(R) * (lamda_dot + alpha + omega * R_dot * R.transpose() * lamda - C * np.linalg.inv(A) * lamda)) / w2
        return val

    def R_dot_calc(self, phi):
        val = [[-math.sin(phi), - math.cos(phi), 0], [math.cos(phi), -math.sin(phi), 0], [0, 0, 0]];
        return val.reshape(2, 3)

    def R_calc(self, phi):
        val = np.array([[math.cos(phi), - math.sin(phi), 0], [math.sin(phi), math.cos(phi), 0], [0, 0, 1]])
        return val.reshape(2, 3)

    def MERV(self, C3, C4, alpha, t, tf):
        val = - (C4 * alpha * math.exp( - C3 ^ (1/2) * t) * (math.exp(C3 ^ (1/2) * t) - math.exp(2 * C3 ^ (1/2) * t) - \
            math.exp(C3 ^ (1/2) * tf) + math.exp(C3 ^ (1/2) * t) * math.exp(C3 ^ (1/2) * tf)))/(C3 * (math.exp(C3 ^ (1/2) * tf) + 1))
        return val

    def find_lambda(self, w2, A, x_dot, k2, w1, Q, x_dot_dot, C, omega, R, R_dot):
        Q_inv = np.linalg.inv(Q)
        val = w2 * (A * x_dot) / k2 - 2 * w1 * (A^2) * Q_inv * x_dot_dot / (k2 ^ 2) - \
            2 * w1 * A * C * (Q_inv * x_dot) / (k2 ^ 2) + 2 * w1 * omega * (A^2) * Q_inv * R * R_dot.transpose() * x_dot / (k2 ^ 2)
        return val

    def d_lamda(self, lamda, del_t):
        '''
        lamda - Array of values of lagragian multiplier
        del_t - timestep value considered in problem
        '''
        vals = []
        for i in range(len(lamda)):
            if i == len(lamda):
                vals.append(lamda[i,:])
            else:
                vals.append((lamda[i+1,:] - lamda[i,:])/del_t)
        return vals
