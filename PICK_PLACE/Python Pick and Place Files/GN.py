import numpy as np
from QuadraticError import QuadraticError
from fk import fk
from pose_guess import pose_guess
from J import J
from Err import Err
from angle_guess import angle_guess
import matplotlib.pyplot as plt

def GN(q_initial, target_pose, max_iterations):
    # Initialize joint angles
    q = q_initial
    error_GN = []
    we = np.array([1, 1, 1, 1, 1, 1])
    We = np.diag(we)
    limits = np.array([ #creates an array of joint angle limits
    [-np.deg2rad(360), np.deg2rad(360)], 
    [-np.deg2rad(128.9), np.deg2rad(128.9)],
    [-np.deg2rad(360), np.deg2rad(360)],
    [-np.deg2rad(147.8), np.deg2rad(147.8)],
    [-np.deg2rad(360), np.deg2rad(360)],
    [-np.deg2rad(120.3), np.deg2rad(120.3)],
    [-np.deg2rad(360), np.deg2rad(360)]
    ])

    converged = False  # Flag to check if the method has converged

    # Set up the loop for a maximum number of iterations
    for iteration in range(1, max_iterations + 1):
        # Enforce limitations on joints
        q = np.clip(q, limits[:, 0], limits[:, 1])
        q_current = q

        # Calculate quadratic error
        EG = QuadraticError(target_pose, fk(q_current))

        # Calculate error
        ek = Err(target_pose, fk(q_current))
        j = J(q_current)
        jT=np.transpose(J(q_current))
        gk = np.linalg.multi_dot([jT, We, ek])
        x = np.linalg.multi_dot([jT, We, j])
        y = np.dot(np.linalg.pinv(x), gk)
        y = np.reshape(y, (1, 7))
        q = np.add(q, y)
        
        error_GN.append(EG)
        q = np.squeeze(q)

        # Check for convergence
        if EG < 1e-5:
            converged = True
            print(f'Gauss Newton method converged in {iteration} iterations.')
            break

    if converged:
        error_GN = np.concatenate(error_GN)
        # np.savetxt('error_GN.txt', error_GN)
        # print(f'Initial Joint Angle Guess: {np.rad2deg(q_initial)}.')
        # print(f'Final Quadratic Error for GN: {EG}.')
        # print(f'Iteration: {iteration}.')
        # print('Desired Position and Orientation:')
        # print(target_pose)
        # print('Converged Joint Angles using GN:')
        # print(np.rad2deg(q))
        # print('Converged Position and Orientation using GN (degrees):')
        # B = np.round(fk(q), decimals=4)
        # print(B)
        iteration = np.arange(1, len(error_GN) + 1)
        # plt.plot(iteration, error_GN, label='GN')
        # plt.xlabel('Iterations')
        # plt.ylabel('Error')
        # plt.title('Convergence of Guass-Newton Method')
        # plt.legend()
        # plt.show()
    else:
        print('Did not Converge')
    return converged, q, iteration

# angles1= angle_guess()
# T1 = pose_guess()
# q=GN(angles1,T1,200)
