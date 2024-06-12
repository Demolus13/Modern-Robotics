import os
import numpy as np
import modern_robotics as mr

M01 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.089159], [0, 0, 0, 1]]
M12 = [[0, 0, 1, 0.28], [0, 1, 0, 0.13585], [-1, 0, 0, 0], [0, 0, 0, 1]]
M23 = [[1, 0, 0, 0], [0, 1, 0, -0.1197], [0, 0, 1, 0.395], [0, 0, 0, 1]]
M34 = [[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0.14225], [0, 0, 0, 1]]
M45 = [[1, 0, 0, 0], [0, 1, 0, 0.093], [0, 0, 1, 0], [0, 0, 0, 1]]
M56 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.09465], [0, 0, 0, 1]]
M67 = [[1, 0, 0, 0], [0, 0, 1, 0.0823], [0, -1, 0, 0], [0, 0, 0, 1]]
G1 = np.diag([0.010267495893, 0.010267495893,  0.00666, 3.7, 3.7, 3.7])
G2 = np.diag([0.22689067591, 0.22689067591, 0.0151074, 8.393, 8.393, 8.393])
G3 = np.diag([0.049443313556, 0.049443313556, 0.004095, 2.275, 2.275, 2.275])
G4 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
G5 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
G6 = np.diag([0.0171364731454, 0.0171364731454, 0.033822, 0.1879, 0.1879, 0.1879])
Glist = [G1, G2, G3, G4, G5, G6]
Mlist = [M01, M12, M23, M34, M45, M56, M67] 
Slist = [[0,         0,         0,         0,        0,        0],
         [0,         1,         1,         1,        0,        1],
         [1,         0,         0,         0,       -1,        0],
         [0, -0.089159, -0.089159, -0.089159, -0.10915, 0.005491],
         [0,         0,         0,         0,  0.81725,        0],
         [0,         0,     0.425,   0.81725,        0,  0.81725]]

dthetalist = [0, 0, 0, 0, 0, 0]
ddthetalist = [0, 0, 0, 0, 0, 0]
g = [0, 0, -9.81]
Ftip = [0, 0, 0, 0, 0, 0]

# Initial parameters
dt = 0.005
intRes = 100

# Simulation 1
N1 = int(3/dt)
thetalist1 = np.array([0, 0, 0, 0, 0, 0])
taumat1 = np.zeros((N1, 6))
Ftipmat1 = np.zeros((N1, 6))
thetalist1, dthetalist1 = mr.ForwardDynamicsTrajectory(thetalist1, dthetalist, taumat1, g, Ftipmat1, Mlist, Glist, Slist, dt, intRes)
print(f"Simulation 1: N1: {N1} Done\n")

# Simulation 2
N2 = int(5/dt)
thetalist2 = np.array([0, -1, 0, 0, 0, 0])
taumat2 = np.zeros((N2, 6))
Ftipmat2 = np.zeros((N2, 6))
thetalist2, dthetalist2 = mr.ForwardDynamicsTrajectory(thetalist2, dthetalist, taumat2, g, Ftipmat2, Mlist, Glist, Slist, dt, intRes)
print(f"Simulation 2: N2: {N2} Done\n")

os.makedirs('Configs/Assignment-2/', exist_ok=True)
filepath = os.path.join('Configs/Assignment-2/', 'simulation1.csv')
np.savetxt(filepath, thetalist1, delimiter=',')

filepath = os.path.join('Configs/Assignment-2/', 'simulation2.csv')
np.savetxt(filepath, thetalist2, delimiter=',')
print("Generated CSV files: Done")