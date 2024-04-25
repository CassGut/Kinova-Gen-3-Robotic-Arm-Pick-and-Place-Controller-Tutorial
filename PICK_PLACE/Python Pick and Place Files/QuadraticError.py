# this function inputs 2 transformation matrices and returns mean square error between them 
# line 7 is the weights for each value in the error vector ( see tutorial)
import numpy as np
from Err import Err

def QuadraticError(Ted, Tec):
    we = np.array([1, 1, 1, 1, 1, 1])
    We = np.diag(we)
    E = 0.5 * np.transpose(Err(Ted, Tec)).dot(We).dot(Err(Ted, Tec))
    return E
