import numpy as np
from Err import Err

def QuadraticError(Ted, Tec):
    we = np.array([1, 1, 1, 1, 1, 1])
    We = np.diag(we)
    E = 0.5 * np.transpose(Err(Ted, Tec)).dot(We).dot(Err(Ted, Tec))
    return E
