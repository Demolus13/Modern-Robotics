import os
import numpy as np
from modern_robotics import *

# Set up the logging configuration to save the results to a file
import logging
logging.basicConfig(filename='Configs/Assignment-1/log.txt', level=logging.INFO, format='%(message)s')

def IKinBodyIterates(Blist, M, T, thetalist0, eomg, ev):

    # Computing the configurations of the robot using the newton-raphson method
    thetalist = np.array(thetalist0).copy()
    i = 0
    maxiterations = 20
    Vb = se3ToVec(MatrixLog6(np.dot(TransInv(FKinBody(M, Blist, \
                                                      thetalist)), T)))
    err = np.linalg.norm([Vb[0], Vb[1], Vb[2]]) > eomg \
          or np.linalg.norm([Vb[3], Vb[4], Vb[5]]) > ev
    
    thetalist_history = [thetalist]
    while err and i < maxiterations:
        thetalist = thetalist \
                    + np.dot(np.linalg.pinv(JacobianBody(Blist, \
                                                         thetalist)), Vb)
        i = i + 1
        Vb \
        = se3ToVec(MatrixLog6(np.dot(TransInv(FKinBody(M, Blist, \
                                                       thetalist)), T)))
        err = np.linalg.norm([Vb[0], Vb[1], Vb[2]]) > eomg \
              or np.linalg.norm([Vb[3], Vb[4], Vb[5]]) > ev
        
        # Store the results in the history list
        thetalist_history.append(thetalist)
        
        # Store the results in the log file
        logging.info(f'Iteration {i} :')
        logging.info('joint vector :\n %s', thetalist)
        logging.info('SE(3) end-effector config :\n %s', FKinBody(M, Blist, thetalist))
        logging.info('error twist V_b :\n %s', Vb)
        logging.info('angular error magnitude ||omega_b|| : %s', np.linalg.norm([Vb[0], Vb[1], Vb[2]]))
        logging.info('linear error magnitude ||v_b|| : %s\n', np.linalg.norm([Vb[3], Vb[4], Vb[5]]))

    return (thetalist, thetalist_history, not err)

# Define the inital configuration of the robot
L1 = 0.425; L2 = 0.392; W1 = 0.109; W2 = 0.082; H1 = 0.089; H2 = 0.095 # Link lengths

B = np.array([[0, 0, 0, 0, 0, 0],
              [1, 0, 0, 0, -1, 0],
              [0, 1, 1, 1, 0, 1],
              [W1 + W2, H2, H2, H2, -W2, 0],
              [0, -L1 - L2, -L2, 0, 0, 0],
              [L1 + L2, 0, 0, 0, 0, 0]])

M = np.array([[-1, 0, 0, L1 + L2],
              [0, 0, 1, W1 + W2],
              [0, 1, 0, H1 - H2],
              [0, 0, 0, 1]])

T = np.array([[0, 1, 0, -0.5],
              [0, 0, -1, 0.1],
              [-1, 0, 0, 0.1],
              [0, 0, 0, 1]])

# Define the initial guess for the joint angles
theta0 = np.array([-0.1, 2, 1.7, 2, 3, -1.5])
eomg = 0.001
ev = 0.0001

# Call the function and display the results
logging.info('Initial guess for the joint angles :\n %s \n', theta0)
thetalist, thetalist_history, success = IKinBodyIterates(B, M, T, theta0, eomg, ev)
print(thetalist, success)

# Save the results to a file
os.makedirs('Configs/Assignment-1/', exist_ok=True)
filepath = os.path.join('Configs/Assignment-1/', 'iterates.csv')
np.savetxt(filepath, thetalist_history, delimiter=',')
