import numpy as np
import modern_robotics as mr

np.set_printoptions(suppress=True, precision=4)

M = np.array([[1,0,0,3.73],
              [0,1,0,0],
              [0,0,1,2.73],
              [0,0,0,1]])

S = np.array([[0,0,0,0,0,0],
              [0,1,1,1,0,0],
              [1,0,0,0,0,1],
              [0,0,1,-0.73,0,0],
              [-1,0,0,0,0,-3.73],
              [0,1,2.73,3.73,1,0]])

B = np.array([[0,0,0,0,0,0],
              [0,1,1,1,0,0],
              [1,0,0,0,0,1],
              [0,2.73,3.73,2,0,0],
              [2.73,0,0,0,0,0],
              [0,-2.73,-1,0,1,0]])

# theta1 = np.array([-np.pi/2, np.pi/2, np.pi/3, -np.pi/4, 1, np.pi/6])
# T = mr.FKinSpace(M, S, theta1)
# print(T)

# theta2 = np.array([-np.pi/2, np.pi/2, np.pi/3, -np.pi/4, 1, np.pi/6])
# T = mr.FKinBody(M, B, theta2)
# print(T)

S = np.array([[0, 0, 0],
              [0, 0, 0],
              [1, 1, 1],
              [0, 0, 0],
              [0, -1, -2],
              [0, 0, 0]])

# S = np.array([[0, 1, 0],
#               [0, 0, 0],
#               [1, 0, 0],
#               [0, 0, 0],
#               [0, 2, 1],
#               [0, 0, 0]])

B = np.array([[0, -1, 0],
              [1, 0, 0],
              [0, 0, 0],
              [3, 0, 0],
              [0, 3, 0],
              [0, 0, 1]])

# theta3 = np.array([0, np.pi/4, -np.pi/4])

# J = mr.JacobianSpace(S, theta3)
# J = mr.JacobianBody(B, theta3)
# print(J)
# J = np.array([[0, 0, 0, 0],
#               [0, 0, 0, 0],
#               [1, 1, 1, 1],
#               [-1, -1, -1, 0],
#               [3, 2, 1, 1],
#               [0, 0, 0, 0]])

# F = np.array([0, 0, 10, 10, 10, 0])
# tau = np.dot(J.T, F)
# print(tau)

# Jv = np.array([[-0.105, 0, 0.006, -0.045, 0, 0.006, 0],
#                [-0.889, 0.006, 0, -0.844, 0.006, 0, 0],
#                [0, -0.105, 0.889, 0, 0, 0, 0]])
# U, S, Vt = np.linalg.svd(Jv)
# longest_direction = U[:, 0]
# longest_direction = longest_direction / np.linalg.norm(longest_direction)
# print(longest_direction)
# print(S[0])

# A = np.array([[1],
#               [2]])
# b = np.array([3, 4]).T
# x = np.dot(np.linalg.pinv(A), b)
# print(x)

# def func(X):
#     x, y = X
#     return [x**2 - 9, y**2 - 4]

# def Jac(X):
#     x, y = X
#     return np.array([[2*x, 0],
#                      [0, 2*y]])

# X0 = np.array([1, 1])

# for i in range(2):
#     X0 = X0 - np.dot(np.linalg.pinv(Jac(X0)), func(X0))
#     print(X0)

M = np.array([[1,0,0,3],
              [0,1,0,0],
              [0,0,1,0],
              [0,0,0,1]])

S = np.array([[0, 0, 0],
              [0, 0, 0],
              [1, 1, 1],
              [0, 0, 0],
              [0, -1, -2],
              [0, 0, 0]])

T_sd = np.array([[-0.585, -0.811, 0, 0.076],
                 [0.811, -0.585, 0, 2.608],
                 [0, 0, 1, 0],
                 [0, 0, 0, 1]])
theta = np.array([np.pi/4, np.pi/4, np.pi/4])
e_w = 0.001
e_v = 0.0001

theta = mr.IKinSpace(S, M, T_sd, theta, e_w, e_v)
print(theta)