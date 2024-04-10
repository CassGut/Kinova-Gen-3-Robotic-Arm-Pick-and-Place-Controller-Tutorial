import numpy as np
from QuadraticError import QuadraticError
from fk import fk
from Err import Err
from J import J
from pose_guess import pose_guess
from angle_guess import angle_guess
import matplotlib.pyplot as plt

def WM(q_initial, target_pose, max_iterations, lm):
    # Initialize joint angles
    q = q_initial
    error_WM = []
    we = np.array([1, 1, 1, 1, 1, 1])
    We = np.diag(we)
    wn = np.array([lm, lm, lm, lm, lm, lm, lm])
    Wn = np.diag(wn)
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

        # Set current joint angles to previous joint angles
        q_current = q

        # Calculate quadratic error
        EW = QuadraticError(target_pose, fk(q_current))

        # Calculate error
        ek = Err(target_pose, fk(q_current))

        # Gauss-Newton equation answer is a column of angles
        j = J(q_current)
        JT = np.transpose(j)
        gk = np.linalg.multi_dot([JT,We,ek])
        x = np.linalg.multi_dot([JT,We,j])
        Ak = np.add(x,Wn)
        y = np.dot(np.linalg.inv(Ak),gk)
        y = np.reshape(y,(1,7))
        q = np.add(q,y)

        

        # Fill the error array with the latest value of E
        error_WM.append(EW)
        q = np.squeeze(q)
        
        # Check for convergence
        if EW < 1e-5:
            converged = True
            print(f'Wamplers method converged in {iteration} iterations.')
            break

    if converged:
        error_WM = np.concatenate(error_WM)
        # np.savetxt('error_WM.txt', error_WM)
        # print(f'Initial Joint Angle Guess: {np.rad2deg(q_initial)}.')
        # print(f'Final Quadratic Error for WM: {EW}.')
        # print(f'Iteration: {iteration}.')
        # print('Desired Position and Orientation:')
        # print(target_pose)
        # print('Converged Joint Angles using WM (degrees):')
        # print(np.rad2deg(q))
        # print('Converged Position and Orientation using WM:')
        # B = np.round(fk(q), decimals=4)
        # print(B)
        # iteration = np.arange(1, len(error_WM) + 1)
        # plt.plot(iteration, error_WM, label='WM')
        # plt.xlabel('Iterations')
        # plt.ylabel('Error')
        # plt.title('Convergence of Wamplers Method')
        # plt.legend()
        # plt.show()
    else:
        print('Did not Converge')
    return converged, q, iteration

# angles1= angle_guess()
# T1 = pose_guess()
# WM(angles1,T1,200,1e-6)