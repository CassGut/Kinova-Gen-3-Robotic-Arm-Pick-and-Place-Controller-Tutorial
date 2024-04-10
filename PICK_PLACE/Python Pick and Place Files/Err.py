import numpy as np
from alpha import alpha
from fk import fk
def Err(Ted, Tec):
    Err = np.vstack([(Ted[:-1, -1] - Tec[:-1, -1]).reshape(-1,1), # trnsforms into a 3by1
                 alpha(np.dot(Ted[:3, :3], np.transpose(Tec[:3, :3])))
                ])
    return Err
