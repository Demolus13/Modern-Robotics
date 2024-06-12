import numpy as np
import modern_robotics as mr

np.set_printoptions(precision=4, suppress=True)

T = 5
t = 3
# s = mr.QuinticTimeScaling(T, t)
# print(f"QuinticTimeScaling: {s}\n")

Xstart = np.array([[1, 0, 0, 0],
                   [0, 1, 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])
Xend = np.array([[0, 0, 1, 1],
                 [1, 0, 0, 2],
                 [0, 1, 0, 3],
                 [0, 0, 0, 1]])
T = 10
N = 10
# traj = mr.ScrewTrajectory(Xstart, Xend, T, N, 3)
# print(f"ScrewTrajectory: {traj[-2]}\n")

traj = mr.CartesianTrajectory(Xstart, Xend, T, N, 5)
print(f"CartesianTrajectory: {traj[-2]}\n")