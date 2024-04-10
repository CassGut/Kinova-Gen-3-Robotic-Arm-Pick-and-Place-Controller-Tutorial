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
from fk import fk
from get_angles_pose import get_angles_pose
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.Exceptions.KServerException import KServerException
from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2
from q_initial_guess import q_initial_guess
from get_angles_pose import get_angles_pose

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
 

def look_position(base, base_cyclic):
    
    print("Starting Cartesian action movement ...")
    action = Base_pb2.Action()
    action.name = "Example Cartesian action movement"
    action.application_data = ""

    feedback = base_cyclic.RefreshFeedback()

    cartesian_pose = action.reach_pose.target_pose
    cartesian_pose.x = 0.3
    cartesian_pose.y = 0.3
    cartesian_pose.z = 0.56
    cartesian_pose.theta_x =  180
    cartesian_pose.theta_y = 0
    cartesian_pose.theta_z = 45

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

def spin(base, SPEED):

    joint_speeds = Base_pb2.JointSpeeds()

    actuator_count = base.GetActuatorCount().count
    # The 7DOF robot will spin in the same direction for 10 seconds
    if actuator_count == 7:
        speeds = [SPEED, 0, 0, 0, 0, 0, 0]
        
        i = 0
        for speed in speeds:
            joint_speed = joint_speeds.joint_speeds.add()
            joint_speed.joint_identifier = i 
            joint_speed.value = speed
            joint_speed.duration = 0
            i = i + 1
        print ("Sending the joint speeds for 10 seconds...")
        base.SendJointSpeedsCommand(joint_speeds)
        time.sleep(1000000)
   

    return True

def main():
    SPEED = 20
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
        q = [138.59751892089844, 342.8174133300781, 0.5525989532470703, 318.83404541015625, 359.99951171875, 239.6773223876953, 2.5826263427734375]
 
        
        # Example core
        success = True
        success &= look_position(base, base_cyclic)
        success &= spin(base, SPEED)
        # print(get_angles_pose(base)[1])
        # print(base.GetMeasuredCartesianPose())
 

        # You can also refer to the 110-Waypoints examples if you want to execute
        # a trajectory defined by a series of waypoints in joint space or in Cartesian space

        return 0 if success else 1

if __name__ == "__main__":
    exit(main())
