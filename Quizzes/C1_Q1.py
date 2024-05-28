# print("Hello World!")
import numpy as np
from scipy.linalg import logm
from scipy.linalg import expm
import modern_robotics as mr

T_sa = np.array([[0,-1,0,0],
                 [0,0,-1,0],
                 [1,0,0,1],
                 [0,0,0,1]])
T_as = np.array([[0,0,1,1],
                 [-1,0,0,0],
                 [0,-1,0,0],
                 [0,0,0,1]])
T_sb = np.array([[1,0,0,0],
                 [0,0,1,2],
                 [0,-1,0,0],
                 [0,0,0,1]])
# print(np.linalg.inv(T_sb))

# R_sa = np.array([[0, -1, 0],
#                  [0, 0, -1],
#                  [1, 0, 0]])
# p_sa = np.array([0, 0, 1]).T

# p_b = np.array([1, 2, 3, 1]).T
# p_s = np.array([3, 2, 1]).T
# T_ab = np.dot(np.linalg.inv(T_sa), T_sb)
# print(T_ab)

V_s = np.array([3, 2, 1, -1, -2, -3])

R_sa = T_sa[:3, :3]
p_sa = T_sa[:3, 3]

# theta = np.arccos((np.trace(R_sa) - 1) / 2)
# print(round(theta, 2))

p_x = np.array([[0, -p_sa[2], p_sa[1]],
                [p_sa[2], 0, -p_sa[0]],
                [-p_sa[1], p_sa[0], 0]])

Ad_T_sa = np.block([[R_sa, np.zeros((3, 3))], [p_x @ R_sa, R_sa]])

V_a = np.linalg.inv(Ad_T_sa) @ V_s
print(V_a)

# S_theta = np.array([[0, -2, 1, 3],
#                     [2, 0, -0, 0],
#                     [-1, 0, 0, 0],
#                     [0, 0, 0, 0]])

# T = expm(S_theta)
# print(T)

# F_b = np.array([1, 0, 0, 2, 1, 0])
# T_bs = np.linalg.inv(T_sb)

# R_bs = T_bs[:3, :3]
# p_bs = T_bs[:3, 3]

# p_x = np.array([[0, -p_bs[2], p_bs[1]],
#                 [p_bs[2], 0, -p_bs[0]],
#                 [-p_bs[1], p_bs[0], 0]])

# Ad_T_bs = np.block([[R_bs, np.zeros((3, 3))], [p_x @ R_bs, R_bs]])
# F_s = Ad_T_bs.T @ F_b
# print(F_s)

# T = np.array([[0, -1, 0, 3],
#               [1, 0, 0, 0],
#               [0, 0, 1, 1],
#               [0, 0, 0, 1]])
# R = T[:3, :3]
# p = T[:3, 3]
# print(R.T)
# print(-R.T @ p)

# V = np.array([1, 0, 0, 0, 2, 3])

# se3_mat = mr.VecTose3(V)
# print(se3_mat)

# q = np.array([0, 0, 2])
# s_hat = np.array([1, 0, 0])
# h = 1

# S = mr.ScrewToAxis(q, s_hat, h)
# print(S)

# S_theta = np.array([[0, -1.5708, 0, 2.3562],
#                     [1.5708, 0, 0, -2.3562],
#                     [0, 0, 0, 1],
#                     [0, 0, 0, 0]])

# T = mr.MatrixExp6(S_theta)
# print(T)

T = np.array([[0, -1, 0, 3],
              [1, 0, 0, 0],
              [0, 0, 1, 1],
              [0, 0, 0, 1]])

S_theta = mr.MatrixLog6(T)

print(np.round(S_theta, decimals=5))

# log_R_sa = logm(R_sa)
# theta = np.sqrt(log_R_sa[0, 1]**2 + log_R_sa[0, 2]**2 + log_R_sa[1, 2]**2) / np.sqrt(2)
# print(theta)

# w = np.array([[0, 0.5, -1],
#               [-0.5, 0, 2],
#               [1, -2, 0]])
# R = expm(w)
# print(R)

# R = np.array([[0, 0, 1],
#               [-1, 0, 0],
#               [0, -1, 0]])
# log_R = logm(R)
# print(log_R)