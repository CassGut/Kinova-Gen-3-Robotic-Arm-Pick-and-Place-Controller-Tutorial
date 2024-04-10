#! /usr/bin/env python3

###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2018 Kinova inc. All rights reserved.
#
# This software may be modified and distributed
# under the terms of the BSD 3-Clause license.
#
# Refer to the LICENSE file for details.
#
###

import sys
import os
import time
import subprocess
from kbhit import KBHit
import threading
import numpy as np
import cv2
import cv2.aruco as aruco
from smallest_angular_distance import smallest_angular_distance_limits
from smallest_angular_distance import smallest_angular_distance_nolimits
from angle_guess import angle_guess
from get_angles_pose import get_angles_pose
from trust_constr import trust_constr 
from fk import fk
from NR import NR
from Rx import Rx
from Ry import Ry
from Rz import Rz
from T import T
from close_gripper import close_gripper
import argparse
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.client_stubs.DeviceManagerClientRpc import DeviceManagerClient
from kortex_api.autogen.client_stubs.DeviceConfigClientRpc import DeviceConfigClient
from kortex_api.autogen.messages import Session_pb2, Base_pb2, Common_pb2
from kortex_api.Exceptions.KServerException import KServerException
gripper_file_path = gripper_file_path = r'C:\Users\rashe\OneDrive\Documents\Kinova Gen3\Kinova_Python_github\Kinova_Python_github\api_python\examples\Project\close_gripper.py'
# Maximum allowed waiting time during actions (in seconds)
TIMEOUT_DURATION = 50

# Actuator speed (deg/s)
SPEED = 30

# aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
# parameters = aruco.DetectorParameters()
# cameraMatrix = np.array([[642.14257641,   0.,         317.1572718 ],
#   [  0.,         641.17787669, 247.15006822],
#   [  0.,           0.,           1.        ]],dtype=np.float32)

distCoeffs = np.zeros((5, 1), dtype=np.float32)  # Assuming no lens distortion
# Create closure to set an event after an END or an ABORT
def check_for_end_or_abort(e):
    """Return a closure checking for END or ABORT notifications

    Arguments:
    e -- event to signal when the action is completed
        (will be set when an END or ABORT occurs)
    """
    def check(notification, e = e):
        print("EVENT : " + \
              Base_pb2.ActionEvent.Name(notification.action_event))
        if notification.action_event == Base_pb2.ACTION_END \
        or notification.action_event == Base_pb2.ACTION_ABORT:
            e.set()
    return check


def move_to_home_position(base):
    # Make sure the arm is in Single Level Servoing mode
    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)
    
    # Move arm to ready position
    print("Moving the arm to a safe position")
    action_type = Base_pb2.RequestedActionType()
    action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
    action_list = base.ReadAllActions(action_type)
    action_handle = None
    for action in action_list.action_list:
        if action.name == "Home":
            action_handle = action.handle

    if action_handle == None:
        print("Can't reach safe position. Exiting")
        return False

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    base.ExecuteActionFromReference(action_handle)
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("Safe position reached")
    else:
        print("Timeout on action notification wait")
    return finished

def move(base, base_cyclic, pose, x, y, z):
    
    print("Starting Cartesian action movement ...")
    action = Base_pb2.Action()
    action.name = "Example Cartesian action movement"
    action.application_data = ""

    feedback = base_cyclic.RefreshFeedback()

    cartesian_pose = action.reach_pose.target_pose
    cartesian_pose.x = pose.x+x
    cartesian_pose.y =pose.y+y
    cartesian_pose.z =pose.z+z
    cartesian_pose.theta_x = pose.theta_x
    cartesian_pose.theta_y = pose.theta_y
    cartesian_pose.theta_z = pose.theta_z

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    print("Executing action")
    base.ExecuteAction(action)

    print("Waiting for movement to finish ...")
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("Cartesian movement completed")
    else:
        print("Timeout on action notification wait")
    return finished

def calibrate(base, base_cyclic, P):
    
    print("Starting Cartesian action movement ...")
    action = Base_pb2.Action()
    action.name = "Example Cartesian action movement"
    action.application_data = ""
    
    # terms = [P, T(0,-0.017,0)] 
    # P = np.linalg.multi_dot(terms)
    theta = rot2eul_zyx(P[:3, :3])
    
    feedback = base_cyclic.RefreshFeedback()
    cartesian_pose = action.reach_pose.target_pose
    cartesian_pose.x = P[0,3]
    cartesian_pose.y = P[1,3]
    cartesian_pose.z = P[2,3]
    cartesian_pose.theta_x = theta[0]
    cartesian_pose.theta_y = theta[1]
    cartesian_pose.theta_z = theta[2]

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    print("Executing action")
    base.ExecuteAction(action)

    print("Waiting for movement to finish ...")
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("Cartesian movement completed")
    else:
        print("Timeout on action notification wait")
    return finished


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


def send_joint_speeds(base,speeds,t):

    joint_speeds = Base_pb2.JointSpeeds()

    actuator_count = base.GetActuatorCount().count
    # The 7DOF robot will spin in the same direction for 10 seconds
    if actuator_count == 7:
        i = 0
        for speed in speeds:
            joint_speed = joint_speeds.joint_speeds.add()
            joint_speed.joint_identifier = i 
            joint_speed.value = speed
            joint_speed.duration = 0
            i = i + 1
        print ("Sending the joint speeds for 20 seconds...")
        base.SendJointSpeedsCommand(joint_speeds)
        time.sleep(t)
    # The 6 DOF robot will alternate between 4 spins, each for 2.5 seconds
    if actuator_count == 6:
        print ("Sending the joint speeds for 10 seconds...")
        for times in range(4):
            del joint_speeds.joint_speeds[:]
            if times % 2:
                speeds = [-SPEED, 0.0, 0.0, SPEED, 0.0, 0.0]
            else:
                speeds = [SPEED, 0.0, 0.0, -SPEED, 0.0, 0.0]
            i = 0
            for speed in speeds:
                joint_speed = joint_speeds.joint_speeds.add()
                joint_speed.joint_identifier = i 
                joint_speed.value = speed
                joint_speed.duration = 0
                i = i + 1
            
            base.SendJointSpeedsCommand(joint_speeds)
            time.sleep(2.5)

    print ("Stopping the robot")
    base.Stop()

    return True


def camera_coor(pose):
    x = pose.x
    y =pose.y
    z =pose.z
    X = pose.theta_x
    Y = pose.theta_y
    Z = pose.theta_z
    rx = Rx(np.deg2rad(X))
    ry = Ry(np.deg2rad(Y))
    rz = Rz(np.deg2rad(Z))
    terms1 = [rz,ry,rx]
    R = np.linalg.multi_dot(terms1)
    M = np.eye(4)
    M[:3, :3] = R[:3, :3]
    M[:3, 3] = [x, y, z]
    terms2= [M,T(0, 0.058, -0.11161)]
    C = np.linalg.multi_dot(terms2)
    return C


def main():
    #//////TIME TO RUN//////
    t = 6
    #///////////////////////
    T1 = np.array([[0, 1, 0,  0.1],
                   [1, 0, 0,  0.35],
                   [0, 0, -1,  0.4],
                   [0,  0,  0,  1]])
# Aruco marker Matrix:
# [[ 0.97812916 -0.04542519  0.20297759  0.38215384]
#  [ 0.08430275  0.97869373 -0.18722079  0.21935885]
#  [-0.19014835  0.20023769  0.96111835  0.03724029]
#  [ 0.          0.          0.          1.        ]]
    #Import the utilities helper module
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    import utilities
    
    args = utilities.parseConnectionArguments()
    
    # Create connection to the device and get the router
    with utilities.DeviceConnection.createTcpConnection(args) as router:

        # Create required services
        base = BaseClient(router)
        base_cyclic = BaseCyclicClient(router)
        # Example core
        success = True
        success &= move_to_home_position(base)
        angles1, pose1 = get_angles_pose(base)
        q1 = angle_guess()
        #angles2 = NR(q1,T1,500)

        angles2 = trust_constr(q1,T1)
        dangles0 = np.empty(7)
        dangles0[0] = smallest_angular_distance_nolimits(angles1[0],angles2[0])
        dangles0[1] = smallest_angular_distance_limits(angles1[1],angles2[1])
        dangles0[2] = smallest_angular_distance_nolimits(angles1[2],angles2[2])
        dangles0[3] = smallest_angular_distance_limits(angles1[3],angles2[3])
        dangles0[4] = smallest_angular_distance_nolimits(angles1[4],angles2[4])
        dangles0[5] = smallest_angular_distance_limits(angles1[5],angles2[5])
        dangles0[6] = smallest_angular_distance_nolimits(angles1[6],angles2[6])
        speeds1 = dangles0/t
        success &= send_joint_speeds(base,speeds1,t)
        angles2,pose2 = get_angles_pose(base)
        print(angles2,pose2)
        input()
        success &= calibrate(base, base_cyclic, T1)
        angles2,pose2 = get_angles_pose(base)
        print(angles2,pose2)
        print(camera_coor(pose2))
        input()
        success &= move(base, base_cyclic, pose2, 0, 0, -0.1651) # change x y z
        # input()
        # time.sleep(0.5)
        # subprocess.run(['python', gripper_file_path])
        # time.sleep(0.5)
        # success &= move_to_home_position(base)
        # angles2,pose2 = get_angles_pose(base)
        # print(angles2,pose2)
        # print(angles2,pose2)
        # subprocess.run(['python', gripper_file_path])
        # success &= calibrate(base, base_cyclic, T1)
        # print(get_angles_pose(base))
        # time.sleep(1)
        # angles3,pose3 = get_angles_pose(base)
        # success &= cartesian_action_movement(base, base_cyclic, pose3, 0, -0.1, 0) # change x y z
        # time.sleep(0.5)
        # subprocess.run(['python', gripper_file_path])
        # time.sleep(2)
        # success &= move_to_home_position(base)
        # angles4,pose4 = get_angles_pose(base)
        # q2 = angle_guess()
        # angles5 = trust_constr(angles4,q2,t1)
        # print(fk(np.deg2rad(angles5)))
        # dangles1 = np.empty(7)
        # dangles1[0] = smallest_angular_distance_nolimits(angles4[0],angles5[0])
        # dangles1[1] = smallest_angular_distance_limits(angles4[1],angles5[1])
        # dangles1[2] = smallest_angular_distance_nolimits(angles4[2],angles5[2])
        # dangles1[3] = smallest_angular_distance_limits(angles4[3],angles5[3])
        # dangles1[4] = smallest_angular_distance_nolimits(angles4[4],angles5[4])
        # dangles1[5] = smallest_angular_distance_limits(angles4[5],angles5[5])
        # dangles1[6] = smallest_angular_distance_nolimits(angles4[6],angles5[6])
        # speeds2 = dangles1/t
        # success &= example_send_joint_speeds(base,speeds2,t)
        # time.sleep(1)
        # angles6,pose6 = get_angles_pose(base)
        # z_down = 0.115
        # example_cartesian_action_movement(base, base_cyclic, pose6,z_down)
        # subprocess.run(['python', gripper_file_path])
        # z_down = -0.1
        # example_cartesian_action_movement(base, base_cyclic, pose6,z_down)
        # success &= example_move_to_home_position(base)
        return 0 if success else 1
if __name__ == "__main__":
    exit(main())