# Sugiharas method used for inverse kinematics calculation
# uncomment line 69 to 86 and 93 to 96 to run sperately, recomment to run the stats file  
# inputs are initial angles guess, desired transformation matrix, max iterations and lamda value(see tutorial)
# outputs the converged angles for the desired transformation matrix 
import numpy as np
from QuadraticError import QuadraticError
from fk import fk
from J import J
from Err import Err
from pose_guess import pose_guess
from angle_guess import angle_guess
import matplotlib.pyplot as plt
def SM(q_initial, target_pose, max_iterations, lm):
    # Initialize joint angles
    q = np.deg2rad(q_initial)
    q = np.array(q)
    error_SM = []
    wn = np.array([lm, lm, lm, lm, lm, lm, lm])
    We = np.eye(6)
    
    
    # Set up the loop for a maximum number of iterations
    for iteration in range(1, max_iterations + 1):
        
        # Enforce limitations on joints with 3rd at 0 deg
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
        
        # Set current joint angles to previous joint angle
        q_current = q
        
        # Calculate quadratic error
        ES = QuadraticError(target_pose, fk(q_current))

        # Calculate error
        ek = Err(target_pose, fk(q_current))

        
        Wn = ES*np.eye(7) + np.diag(wn)
        j = J(q_current)
        JT = np.transpose(j)
        gk = np.linalg.multi_dot([JT,We,ek])
        x = np.linalg.multi_dot([JT,We,j])
        Ak = np.add(x,Wn)
        y = np.dot(np.linalg.pinv(Ak),gk)
        y = np.reshape(y,(1,7))
        q = np.add(q,y)
        

        # Fill the error array with the latest value of E
        error_SM.append(ES)
        q = np.squeeze(q)
        
        if ES < 1e-5:
            converged = True
            print(f'Sugiharas Method converged in {iteration} iterations.')
            break

    if converged:
        error_SM = np.concatenate(error_SM)
        # np.savetxt('error_SM.txt', error_SM)
        # print(f'Initial Joint Angle Guess: {np.rad2deg(q_initial)}.')
        # print(f'Final Quadratic Error for SM: {ES}.')
        # print(f'Iteration: {iteration}.')
        # print('Desired Position and Orientation:')
        # print(target_pose)
        # print('Converged Joint Angles using SM (degrees):')
        # print(np.rad2deg(q))
        # print('Converged Position and Orientation using SM:')
        # B = np.round(fk(q), decimals=4)
        # print(B)
        # iteration = np.arange(1, len(error_SM) + 1)
        # plt.plot(iteration, error_SM, label='SM')
        # plt.xlabel('Iterations')
        # plt.ylabel('Error')
        # plt.title('Convergence of Sugiharas Method')
        # plt.legend()
        # plt.show()
    else:
        print('Did not Converge')
        iteration = []
        q = []
    return converged, np.rad2deg(q), iteration

# angles1 = angle_guess()
#  T1 = pose_guess()
# a,b,c = SM(angles1,T1,200,0.028773368)
# print(b)

