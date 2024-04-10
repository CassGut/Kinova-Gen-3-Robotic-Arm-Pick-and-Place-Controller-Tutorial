import numpy as np
def is_diagonal(matrix):
    # Check if the matrix is square
    if len(matrix) != len(matrix[0]):
        return False
    
    # Iterate through rows and columns
    for i in range(len(matrix)):
        for j in range(len(matrix[i])):
            # If an off-diagonal element is not zero, return False
            if i != j and matrix[i][j] !=  0:
                return False
    
    # If no off-diagonal elements are non-zero, return True
    return True
# T=np.array([[1, 0, 0], 
#              [0, 1, 0], 
#              [0, 0, 1]])
# print(is_diagonal(T))