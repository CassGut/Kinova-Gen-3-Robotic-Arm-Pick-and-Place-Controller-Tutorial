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
import threading
import numpy as np
from get_angles_pose import get_angles_pose
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.Exceptions.KServerException import KServerException
from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2
from q_initial_guess import q_initial_guess

# Maximum allowed waiting time during actions (in seconds)
TIMEOUT_DURATION = 30

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
 
def example_move_to_home_position(base):
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

def example_angular_action_movement(base,angle_values):
    
    print("Starting angular action movement ...")
    action = Base_pb2.Action()
    action.name = "Example angular action movement"
    action.application_data = ""

    actuator_count = base.GetActuatorCount()

    # Place arm straight up
    i=0
    for joint_id in range(actuator_count.count):
     joint_angle = action.reach_joint_angles.joint_angles.joint_angles.add()
     joint_angle.joint_identifier = i
     joint_angle.value = angle_values[i]
     i=i+1

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
        print("Angular movement completed")
    else:
        print("Timeout on action notification wait")
    return finished


def cartesian_action_movement(base, base_cyclic, pose, x, y, z):
    
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

def example_cartesian_action_movement(base, base_cyclic):
    
    print("Starting Cartesian action movement ...")
    action = Base_pb2.Action()
    action.name = "Example Cartesian action movement"
    action.application_data = ""
    feedback = base_cyclic.RefreshFeedback()

    cartesian_pose = action.reach_pose.target_pose
    cartesian_pose.x = 0.35
    cartesian_pose.y = 0.35
    cartesian_pose.z = 0.35
    cartesian_pose.theta_x =  180
    cartesian_pose.theta_y = 0
    cartesian_pose.theta_z = 90
# x: 0.35690534114837646
# y: -0.22816400229930878
# z: 0.4503248333930969
# theta_x: -0.2910321056842804
# theta_y: -179.95693969726562
# theta_z: 147.1464080810547

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

def example_cartesian_action_movement1(base, base_cyclic):
    
    print("Starting Cartesian action movement ...")
    action = Base_pb2.Action()
    action.name = "Example Cartesian action movement"
    action.application_data = ""
    feedback = base_cyclic.RefreshFeedback()

    cartesian_pose = action.reach_pose.target_pose
    cartesian_pose.x = 0.35
    cartesian_pose.y = 0
    cartesian_pose.z = 0.35
    cartesian_pose.theta_x =  180
    cartesian_pose.theta_y = 0
    cartesian_pose.theta_z = 90
# x: 0.35690534114837646
# y: -0.22816400229930878
# z: 0.4503248333930969
# theta_x: -0.2910321056842804
# theta_y: -179.95693969726562
# theta_z: 147.1464080810547

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

class JointAngle:
    def __init__(self, joint_identifier, value):
        self.joint_identifier = joint_identifier
        self.value = value

class JointAngles:
    def __init__(self):
        self.joint_angles = []

    def add(self, joint_angle):
        self.joint_angles.append(joint_angle)
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
        base_cyclic = BaseCyclicClient(router)
        #angle_values = example_inverse_kinematics_custom_pose(base, T, q)
        q = [ 180, 0, 0, -45, 0, -120.3, 90]
        
        # Example core
        success = True

        success &= example_move_to_home_position(base)
        success &= example_cartesian_action_movement(base, base_cyclic)
        success &= example_cartesian_action_movement1(base, base_cyclic)
        #success &= example_angular_action_movement(base,q)
        print(get_angles_pose(base))
        print(base.GetMeasuredCartesianPose())
        # print(base.GetMeasuredJointAngles())
 

        # You can also refer to the 110-Waypoints examples if you want to execute
        # a trajectory defined by a series of waypoints in joint space or in Cartesian space

        return 0 if success else 1

if __name__ == "__main__":
    exit(main())


