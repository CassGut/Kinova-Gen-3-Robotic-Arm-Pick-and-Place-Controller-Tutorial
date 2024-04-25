# these functions input initial angles and final angles and return the shortest distance between them while imposing joint limitations  
import numpy as np
import math

def smallest_angular_distance_limits(initial_angle, final_angle):
    initial_angle = (initial_angle +360) % 360
    final_angle = (final_angle +360) % 360
    # Check if both angles are in the top half
    if (math.sin(np.deg2rad(initial_angle))>=0) and (math.sin(np.deg2rad(final_angle))>0):
        cw = np.abs((initial_angle - final_angle + 360) % 360)
        ccw = np.abs((final_angle - initial_angle + 360) % 360)
        if ccw < cw:
            return ccw
        else:
            return -cw

    # Check if both angles are in the bottom half
    elif (math.sin(np.deg2rad(initial_angle))<=0) and (math.sin(np.deg2rad(final_angle))<0):
        cw = np.abs((initial_angle - final_angle + 360) % 360)
        ccw = np.abs((final_angle - initial_angle + 360) % 360)
        if ccw < cw:
            return ccw
        else:
            return -cw

    # Check if initial angle is in the top half and final angle is in the bottom half
    elif (math.sin(np.deg2rad(initial_angle))>=0) and (math.sin(np.deg2rad(final_angle))<0):
        return -np.abs((initial_angle - final_angle + 360) % 360)

    # Check if initial angle is in the bottom half and final angle is in the top half
    elif (math.sin(np.deg2rad(initial_angle))<=0) and (math.sin(np.deg2rad(final_angle))>0):
        return np.abs((final_angle - initial_angle + 360) % 360)

    # If none of the above conditions are met
    else:
        return "Angles are not clearly in the top or bottom half."
def smallest_angular_distance_nolimits(initial_angle, final_angle):
        initial_angle = (initial_angle +360) % 360
        final_angle = (final_angle +360) % 360    
        cw = np.abs((initial_angle - final_angle + 360) % 360)
        ccw = np.abs((final_angle - initial_angle + 360) % 360)
        if ccw < cw:
            return ccw
        else:
            return -cw
