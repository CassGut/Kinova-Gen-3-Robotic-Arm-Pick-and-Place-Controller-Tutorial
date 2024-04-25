# this file gets the current pose and angles of the robot 
import sys
import os
import numpy as np
from Rx import Rx
from Ry import Ry
from Rz import Rz
from T import T

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient

from kortex_api.autogen.messages import Base_pb2
from kortex_api.Exceptions.KServerException import KServerException


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


def get_angles_pose(base):
    while True:
        # Try to get the current arm's joint angles
        try:
            print("Getting Angles for every joint...")
            input_joint_angles = base.GetMeasuredJointAngles()
        except KServerException as ex:
            print("Unable to get joint angles")
            print("Error_code:{} , Sub_error_code:{} ".format(ex.get_error_code(), ex.get_error_sub_code()))
            print("Caught expected error: {}".format(ex))
            continue  # Go to the next iteration of the loop to try again

        # Try to compute Forward Kinematics from arm's current joint angles
        try:
            print("Computing Forward Kinematics using joint angles...")
            pose = base.ComputeForwardKinematics(input_joint_angles)
        except KServerException as ex:
            print("Unable to compute forward kinematics")
            print("Error_code:{} , Sub_error_code:{} ".format(ex.get_error_code(), ex.get_error_sub_code()))
            print("Caught expected error: {}".format(ex))
            continue  # Go to the next iteration of the loop to try again

        # If both operations were successful, break out of the loop
        joint_angles_array = [joint_angle.value for joint_angle in input_joint_angles.joint_angles]
        return joint_angles_array, pose

# Example usage
def main():
    # Import the utilities helper module
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    import utilities

    # Parse arguments
    args = utilities.parseConnectionArguments()
    
    # Create connection to the device and get the router
    with utilities.DeviceConnection.createTcpConnection(args) as router:

        # Create required services
        base = BaseClient(router)

        # Example core
        print(get_angles_pose(base))
        print(base.GetMeasuredCartesianPose())


if __name__ == "__main__":
    exit(main())
