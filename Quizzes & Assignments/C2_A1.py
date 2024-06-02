import os
import numpy as np
from modern_robotics import *

import logging
logging.basicConfig(filename='Configs/log.txt', level=logging.INFO, format='%(message)s')

def IKinBodyIterates(Blist, M, T, thetalist0, eomg, ev):
    """Computes inverse kinematics in the body frame for an open chain robot

    :param Blist: The joint screw axes in the end-effector frame when the
                  manipulator is at the home position, in the format of a
                  matrix with axes as the columns
    :param M: The home configuration of the end-effector
    :param T: The desired end-effector configuration Tsd
    :param thetalist0: An initial guess of joint angles that are close to
                       satisfying Tsd
    :param eomg: A small positive tolerance on the end-effector orientation
                 error. The returned joint angles must give an end-effector
                 orientation error less than eomg
    :param ev: A small positive tolerance on the end-effector linear position
               error. The returned joint angles must give an end-effector
               position error less than ev
    :return thetalist: Joint angles that achieve T within the specified
                       tolerances,
    :return success: A logical value where TRUE means that the function found
                     a solution and FALSE means that it ran through the set
                     number of maximum iterations without finding a solution
                     within the tolerances eomg and ev.
    Uses an iterative Newton-Raphson root-finding method.
    The maximum number of iterations before the algorithm is terminated has
    been hardcoded in as a variable called maxiterations. It is set to 20 at
    the start of the function, but can be changed if needed.

    Example Input:
        Blist = np.array([[0, 0, -1, 2, 0,   0],
                          [0, 0,  0, 0, 1,   0],
                          [0, 0,  1, 0, 0, 0.1]]).T
        M = np.array([[-1, 0,  0, 0],
                      [ 0, 1,  0, 6],
                      [ 0, 0, -1, 2],
                      [ 0, 0,  0, 1]])
        T = np.array([[0, 1,  0,     -5],
                      [1, 0,  0,      4],
                      [0, 0, -1, 1.6858],
                      [0, 0,  0,      1]])
        thetalist0 = np.array([1.5, 2.5, 3])
        eomg = 0.01
        ev = 0.001
    Output:
        (np.array([1.57073819, 2.999667, 3.14153913]), True)
    """
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
        thetalist_history.append(thetalist)
        
        # Display the iteration results
        logging.info(f'Iteration {i} :')
        logging.info('joint vector :\n %s', thetalist)
        logging.info('SE(3) end-effector config :\n %s', FKinBody(M, Blist, thetalist))
        logging.info('error twist V_b :\n %s', Vb)
        logging.info('angular error magnitude ||omega_b|| : %s', np.linalg.norm([Vb[0], Vb[1], Vb[2]]))
        logging.info('linear error magnitude ||v_b|| : %s\n', np.linalg.norm([Vb[3], Vb[4], Vb[5]]))

    return (thetalist, thetalist_history, not err)

# Define the inital configuration of the robot
B = np.array([[0, 0, 1, 0, 0.2, 0.2],
              [0, 0, 1, 0, 0.2, 0.4],
              [0, 0, 1, 0, 0.2, 0.6],
              [0, 0, 1, 0, 0.2, 0.8],
              [0, 0, 1, 0, 0.2, 1.0],
              [0, 0, 0, 0, 0, 1.0]])

M = np.array([[1, 0, 0, 0],
              [1, 0, 0, 0],
              [1, 0, 0, 0],
              [1, 0, 0, 0]])

T = np.array([[0, 1, 0, -0.5],
              [0, 0, -1, 0.1],
              [-1, 0, 0, 0.1],
              [0, 0, 0, 1]])

theta0 = np.array([0, 0, 0, 0, 0, 0])
eomg = 0.001
ev = 0.0001

# Call the function and display the results
thetalist, thetalist_history, success = IKinBodyIterates(B, M, T, theta0, eomg, ev)

# Save the results to a file
os.makedirs('Configs/', exist_ok=True)
filepath = os.path.join('Configs/', 'iterates.csv')
np.savetxt(filepath, thetalist_history, delimiter=',')
