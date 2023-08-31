import math
import numpy as np
import random
import matplotlib.pyplot as plt
import pandas as pd


class KFilter:
    def __init__(self, T):
        v = 2.778 / 285
        w = v / (500 / (2 * math.pi))
        F = [[1, math.sin(w * T) / (w), 0, (math.cos(w * T) - 1) / (w), 0, 0],
             [0, math.cos(w * T), 0, -1 * math.cos(w * T), 0, 0],
             [0, (1 - math.cos(w * T)) / (w), 1, math.sin(w * T) / (w), 0, 0],
             [0, math.sin(w * T), 0, math.cos(w * T), 0, 0],
             [0, 0, 0, 0, 1, T],
             [0, 0, 0, 0, 0, 1]]
        self.F = np.array(F)
        self.Q = np.zeros((3, 3))
        self.Q[0][0] = 4 ** 2
        self.Q[1][1] = 1 ** 2
        self.Q[2][2] = 4 ** 2
        self.R = np.zeros((3, 3))
        self.R[0, 0] = 1 ** 2
        self.R[1, 1] = 1 ** 2
        self.R[2, 2] = 0.4 ** 2
        self.P = np.zeros((6, 6))
        self.P[0, 0] = 100
        self.P[1, 1] = 25
        self.P[2, 2] = 100
        self.P[3, 3] = 25
        self.P[4, 4] = 100
        self.P[5, 5] = 25
        self.T = T
        G = [[self.T**3/3, self.T**2/2,  0],
             [self.T**2/2, self.T, 0],
             [self.T**2/2, self.T**3/3, 0],
             [self.T, self.T**2/2,0],
             [0, 0, self.T**2/2],
             [0, 0, self.T]]
        self.G = np.array(G)
        H = [[1, 0, 0, 0, 0, 0],
             [0, 0, 1, 0, 0, 0],
             [0, 0, 0, 0, 1, 0]]
        self.H = np.array(H)


    def KF(self,obser, target_estimate):
        """
        :param obser: 当前时刻的观察
        :param target_estimate: 上一时刻对当前时刻目标的估计
        :return:下一时刻目标的估计
        """
        obser_1 = np.zeros(3)
        obser_1[0:2] = obser
        x_pre = np.dot(self.F, target_estimate)
        P_pre = np.dot(np.dot(self.F, self.P), self.F.T) + np.dot(np.dot(self.G, self.Q), self.G.T)
        z_pre = np.dot(self.H, x_pre)
        K = np.dot(np.dot(P_pre, self.H.T), np.linalg.inv(np.dot(np.dot(self.H, P_pre), self.H.T) + self.R))
        predict = x_pre + np.dot(K, obser_1 - z_pre)
        self.P = P_pre - np.dot(np.dot(K, self.H), P_pre)
        return [predict[0],predict[1]]





# if __name__ == '__main__':
#     S = 180
#     T = 0.1
#     N = int(S / T)
#     v = 2.778 / 285
#     w = v / (500 / (2 * math.pi))
#
#     pos_flight = [125.35, 43.88, 10]
#     pos_flight = np.array(pos_flight)
#     target_initial = [125.3501, 2.778, 43.8801, 0, 0, 0]
#     target_initial = np.array(target_initial)
#     target_pos = np.zeros((6, N))
#     target_pos[:, 0] = target_initial.T
#
#     for i in range(N - 1):
#         ww = np.zeros(6)
#         ww[0] = 0.0000004*(random.random()-0.5)
#         ww[1] = 0.000001 * (random.random() - 0.5)
#         ww[2] = 0.0000004 * (random.random() - 0.5)
#         ww[3] = 0.000001 * (random.random() - 0.5)
#         ww[4] = 0.4 * (random.random() - 0.5)
#         ww[5] = 0.01 * (random.random() - 0.5)
#         target_pos[:, i + 1] = np.dot(F, target_pos[:, i]) + ww
#     for i in range(N):
#         #obser[0, i] = np.sqrt((juli(target_pos[[0,2], i], pos_flight[0:2]))**2 + pos_flight[2]**2)
#         obser[0, i] = target_pos[0, i] + 0.000001*(random.random()-0.5)
#         obser[1, i] = target_pos[2, i] + 0.000001*(random.random()-0.5)
#         obser[2, i] = 0
#     target_estimate = np.zeros((6, N))
#     P = np.zeros((6, 6))
#     P[0, 0] = 10
#     P[1, 1] = 5
#     P[2, 2] = 10
#     P[3, 3] = 5
#     P[4, 4] = 10
#     P[5, 5] = 5
#     for i in range(N):
#         if i==0:
#             target_estimate[:, i], P = KF(obser[:, i], target_initial, F, Q, R, P)
#         else:
#             target_estimate[:, i], P = KF(obser[:, i], target_estimate[:, i-1], F, Q, R, P)
#     fig = plt.figure()
#     plt.scatter(target_pos[0, :], target_pos[2, :], c='red', s=2, label='legend')
#     plt.scatter(target_estimate[0, :], target_estimate[2, :], c='blue', s=2, label='legend')
#     plt.show()
#









