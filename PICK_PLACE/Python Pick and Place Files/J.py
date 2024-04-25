# this function calculates the Jacobian of any pose 
# inputs joint angles that repersent pose and outputs the jacobian matrix
import numpy as np
from T import T 
from Rx import Rx
from Rz import Rz
from angle_guess import angle_guess

def J(q_values):
    q1, q2, q3, q4, q5, q6, q7 = q_values

    E1 = T(0, 0, 0.15643)
    E2 = Rx(np.pi)
    E3 = Rz(q1)
    E4 = T(0, 0, -0.12838)
    E5 = T(0, 0.00538, 0)
    E6 = Rx(np.pi/2)
    E7 = Rz(q2)
    E8 = T(0, 0, -0.00638)
    E9 = T(0, -0.21038, 0)
    E10 = Rx(-np.pi/2)
    E11 = Rz(q3)
    E12 = T(0, 0, -0.21038)
    E13 = T(0, 0.00638, 0)
    E14 = Rx(np.pi/2)
    E15 = Rz(q4)
    E16 = T(0, 0, -0.00638)
    E17 = T(0, -0.20843, 0)
    E18 = Rx(-np.pi/2)
    E19 = Rz(q5)
    E20 = T(0, 0, -0.10593)
    E21 = Rx(np.pi/2)
    E22 = Rz(q6)
    E23 = T(0, -0.10593, 0)
    E24 = Rx(-np.pi/2)
    E25 = Rz(q7)
    E26 = T(0, 0, -0.06153)
    E27 = Rx(np.pi)



    # ... Define other transformation matrices (E4 to E27) similarly ...
    A = [None] * 9
    A[1] = np.linalg.multi_dot([E1, E2, E3])
    A[2] = np.linalg.multi_dot([E1, E2, E3, E4, E5, E6, E7])
    A[3] = np.linalg.multi_dot([E1, E2, E3, E4, E5, E6, E7, E8, E9, E10, E11])
    A[4] = np.linalg.multi_dot([E1, E2, E3, E4, E5, E6, E7, E8, E9, E10, E11, E12, E13, E14, E15])
    A[5] = np.linalg.multi_dot([E1, E2, E3, E4, E5, E6, E7, E8, E9, E10, E11, E12, E13, E14, E15, E16, E17, E18, E19])
    A[6] = np.linalg.multi_dot([E1, E2, E3, E4, E5, E6, E7, E8, E9, E10, E11, E12, E13, E14, E15, E16, E17, E18, E19, E20, E21, E22])
    A[7] = np.linalg.multi_dot([E1, E2, E3, E4, E5, E6, E7, E8, E9, E10, E11, E12, E13, E14, E15, E16, E17, E18, E19, E20, E21, E22, E23, E24, E25])
    A[8] = np.linalg.multi_dot([E1, E2, E3, E4, E5, E6, E7, E8, E9, E10, E11, E12, E13, E14, E15, E16, E17, E18, E19, E20, E21, E22, E23, E24, E25, E26, E27])
         

    t0 = np.array([0, 0, 0])
    z0 = np.array([0, 0, 1])
   
    # Initialize lists to store t and z
    t = [np.empty((3, 0))] * 9
    z = [np.empty((3, 0))] * 9

# Loop through A to extract the last column for t and the third column for z
    for i in range(1, 9):
     t[i] = A[i][:-1, -1]
     z[i] = A[i][:-1, 2]
    

# Initialize a list to store the cross products
    cp = [None] * 9

# Calculate the cross products for each pair of vectors (z{i}, (t{end} - t{i}))
    cp[1] = np.cross(z0, (t[-1] - t0))
    cp[2] = np.cross(z[1], (t[-1] - t[1]))
    cp[3] = np.cross(z[2], (t[-1] - t[2]))
    cp[4] = np.cross(z[3], (t[-1] - t[3]))
    cp[5] = np.cross(z[4], (t[-1] - t[4]))
    cp[6] = np.cross(z[5], (t[-1] - t[5]))
    cp[7] = np.cross(z[6], (t[-1] - t[6]))
    cp[8] = np.cross(z[7], (t[-1] - t[7]))

# Reshape the elements of cp
    for i in range(1, 9):
     cp[i] = cp[i].reshape(3, 1)
    
# Reshape the elements of z
    for i in range(1,9):
     z[i] = z[i].reshape(3, 1)

# Ensure all arrays have the same size along axis 1
    z0 = z0.reshape(3, 1)    
    
# Vertically stack cp[1] and z0
    v = [None] * 9
    v[1] = np.vstack([cp[1], z0])
    v[2] = np.vstack([cp[2], z[1]])
    v[3] = np.vstack([cp[3], z[2]])
    v[4] = np.vstack([cp[4], z[3]])
    v[5] = np.vstack([cp[5], z[4]])
    v[6] = np.vstack([cp[6], z[5]])
    v[7] = np.vstack([cp[7], z[6]])
    v[8] = np.vstack([cp[8], z[7]])

# Horizontally stack with the next vertical stack
    #J = v[1:]
    J = np.hstack([v[1],v[2],v[3],v[4],v[5],v[6],v[7],v[8]])
    J = np.delete(J, 0, axis=1)

    return J

# # example usage:
# q_values_example = angle_guess()
# jacobian_matrix = np.around(J(q_values_example),4)
# print(jacobian_matrix)


