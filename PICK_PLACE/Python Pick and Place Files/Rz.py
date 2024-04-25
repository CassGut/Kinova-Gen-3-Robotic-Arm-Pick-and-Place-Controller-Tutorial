# this function inputs a rotation angle around the z axis and returns the rotation matrix
import numpy as np

def Rz(theta):
    Rz_matrix = np.array([
        [np.cos(theta), -np.sin(theta), 0, 0],
        [np.sin(theta), np.cos(theta), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    
    return Rz_matrix

