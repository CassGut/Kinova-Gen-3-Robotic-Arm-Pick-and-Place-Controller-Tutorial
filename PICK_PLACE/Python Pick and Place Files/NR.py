# Newton Raphson method used for inverse kinematics calculation
# uncomment line 62 to 79 and 84 to 87 to run sperately, recomment to run the stats file
# inputs are initial angles guess, desired transformation matrix and max iterations 
# outputs the converged angles for the desired transformation matrix 
import numpy as np
from QuadraticError import QuadraticError
from fk import fk
from J import J
from angle_guess import angle_guess
from pose_guess import pose_guess
from Err import Err
import matplotlib.pyplot as plt

def NR(q_initial, target_pose, max_iterations):
    # Initialize joint angles
    q = q_initial # 
    error_NR = [] # makes a list to store quadratic error for NR
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
        q = np.clip(q, limits[:, 0], limits[:, 1]) #clip method defines how during iteration if a joint reaches max then at next iteration itll stay the same

        # Set current joint angles to previous joint angles
        q_current = q # each iteration inputs a joint angle guess initally then continuousyly loops the output back in until the quadratic error is low enough. 
      
        # Calculate quadratic error
        EN = QuadraticError(target_pose, fk(q_current)) # calculating Quadratic Error convergence criterion. The 

        # Calculate error for Newton-Raphson equation
        ek = Err(target_pose, fk(q_current))

        x = np.dot(np.linalg.pinv(J(q_current)), ek)
        x = np.reshape(x,(1,7))
        # Newton-Raphson equation answer is a column of angles
        q = np.add(q,x)


        # Fill the error array with the latest value of E
        error_NR.append(EN)
        q = np.squeeze(q)
       
        # Check for convergence
        if EN < 1e-5:
            converged = True
            print(f'Newton-Raphson method converged in {iteration} iterations.')
            break

    if converged:
        error_NR = np.concatenate(error_NR)
        # np.savetxt('error_Nr.txt', error_NR)
        # print(f'Initial Joint Angle Guess: {np.rad2deg(q_initial)}.')
        # print(f'Final Quadratic Error for NR: {EN}.')
        # print(f'Iteration: {iteration}.')
        # print('Desired Position and Orientation:')
        # print(target_pose)
        # print('Converged Joint Angles using NR:')
        # print(np.rad2deg(q))
        # print('Converged Position and Orientation using NR (degrees):')
        # B = np.round(fk(q), decimals=4)
        # print(B)
        # iteration = np.arange(1, len(error_NR) + 1)
        # plt.plot(iteration, error_NR, label='NR')
        # plt.xlabel('Iterations')
        # plt.ylabel('Error')
        # plt.title('Convergence of Newton-Raphson Method')
        # plt.legend()
        # plt.show()
    else:
        print('Newton Raphson Did not Converge')
    return converged, q, iteration

# angles1= angle_guess()
# T1 = pose_guess()
# covergence, angles, i =NR(angles1,T1,100)
# print(i)