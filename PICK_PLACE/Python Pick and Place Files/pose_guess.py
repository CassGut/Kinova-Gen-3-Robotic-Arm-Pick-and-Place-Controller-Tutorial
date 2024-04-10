import random
import numpy as np

def pose_guess():
    # Put all variables into a list
    while True:
        # Generate random points within a cube until one falls inside the semi-sphere
        radius = 0.73512
        max_z = 1.01793
        x = random.uniform(-radius, radius)
        y = random.uniform(-radius, radius)
        z = random.uniform(0, max_z)  # Limiting z to the upper hemisphere

        if (np.abs(x)>=0.15) and (np.abs(y)>=0.15) and  (x**2 + y**2 + z**2 <= radius**2):  # Check if the point is inside the semi-sphere
            
            A = np.array([[ 0,  1,  0 ],
                          [ 1,  0,  0 ],
                          [ 0,  0, -1 ]])
    
            B = np.array([[-1,  0,  0 ],
                          [ 0,  1,  0 ],
                          [ 0,  0, -1 ]])
    
            C = np.array([[-1,  0,  0 ],
                          [ 0,  0,  1 ],
                          [ 0,  1,  0 ]])
    
            D = np.array([[ 0,  0,  1 ],
                          [ 1,  0,  0 ],
                          [ 0,  1,  0 ]])
    
            E = np.array([[ 1,  0,  0 ],
                          [ 0,  0, -1 ],
                          [ 0,  1,  0 ]])
    
            F = np.array([[ 1,  0,  0 ],
                          [ 0, -1,  0 ],
                          [ 0,  0, -1 ]])
    
            G = np.array([[ 0,  0, -1 ],
                          [-1,  0,  0 ],
                          [ 0,  1,  0 ]])
    
            H = np.array([[ 0, -1,  0 ],
                          [-1,  0,  0 ],
                          [ 0,  0, -1 ]])
    
            variables = [A, B, C, D, E, F, G, H]
            rand_orien = random.choice(variables)
            pose_guess = np.eye(4)
            pose_guess[:3, :3] = rand_orien
            pose_guess[0,3] = x
            pose_guess[1,3] = y
            pose_guess[2,3] = z
            
            break 
    
    return pose_guess    
    
    

   
# print(pose_guess())

