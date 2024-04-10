import numpy as np

def T(dx, dy, dz):
    T_matrix = np.array([
        [1, 0, 0, dx],
        [0, 1, 0, dy],
        [0, 0, 1, dz],
        [0, 0, 0, 1]
    ])
    
    return T_matrix

