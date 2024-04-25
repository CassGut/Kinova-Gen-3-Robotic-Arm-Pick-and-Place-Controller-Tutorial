# this function inputs a rotation matrix matrix and returns the euler vector form 
import numpy as np

def rot2eul_zyx(R):
    # Check if the input is a valid rotation matrix
    assert R.shape == (3,  3), "Input must be a  3x3 rotation matrix"
    
    # Extract the three Euler angles
    sy = np.sqrt(R[0,  0]*R[0,  0] + R[1,  0]*R[1,  0])
    singular = sy <  1e-6

    if not singular:
        x = np.arctan2(R[2,  1], R[2,  2])
        y = np.arctan2(-R[2,  0], sy)
        z = np.arctan2(R[1,  0], R[0,  0])
    else:
        x = np.arctan2(-R[1,  2], R[1,  1])
        y = np.arctan2(-R[2,  0], sy)
        z =  0

    # Convert to degrees if needed
    return np.degrees([x, y, z])



