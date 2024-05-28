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

theta2 = np.array([-np.pi/2, np.pi/2, np.pi/3, -np.pi/4, 1, np.pi/6])
T = mr.FKinBody(M, B, theta2)
print(T)