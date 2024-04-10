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
from diff import diff
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.client_stubs.DeviceManagerClientRpc import DeviceManagerClient
from kortex_api.autogen.client_stubs.DeviceConfigClientRpc import DeviceConfigClient
from send_angular_waypoint_trajectory import example_move_to_home_position
from kortex_api.autogen.messages import Session_pb2, Base_pb2, Common_pb2
from smallest_angular_distance import smallest_angular_distance_limits
from smallest_angular_distance import smallest_angular_distance_nolimits
from run_IP import run_IP
from fk import fk
from kortex_api.autogen.messages import Session_pb2, Base_pb2, Common_pb2
# Maximum allowed waiting time during actions (in seconds)
TIMEOUT_DURATION = 20

# Actuator speed (deg/s)
SPEED = 30

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

def example_move_to_start_position(base):
    # Make sure the arm is in Single Level Servoing mode
    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)
    
    # Move arm to ready position
    constrained_joint_angles = Base_pb2.ConstrainedJointAngles()

    actuator_count = base.GetActuatorCount().count
    angles = [0.0] * actuator_count

    # Actuator 4 at 90 degrees
    for joint_id in range(len(angles)):
        joint_angle = constrained_joint_angles.joint_angles.joint_angles.add()
        joint_angle.joint_identifier = joint_id
        joint_angle.value = angles[joint_id]

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    print("Reaching joint angles...")
    base.PlayJointTrajectory(constrained_joint_angles)

    print("Waiting for movement to finish ...")
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("Joint angles reached")
    else:
        print("Timeout on action notification wait")
    return finished

def example_send_joint_speeds(base):

    joint_speeds = Base_pb2.JointSpeeds()

    actuator_count = base.GetActuatorCount().count
# The 7DOF robot will spin in the same direction for 10 seconds
    if actuator_count == 7:
     speeds = [SPEED, 0, -SPEED, 0, SPEED, 0, -SPEED]
     durations = [2, 0, 3, 0, 2, 0, 3]  # Different sleep durations between joint movements
     i = 0
     for speed, duration in zip(speeds, durations):
        joint_speed = joint_speeds.joint_speeds.add()
        joint_speed.joint_identifier = i 
        joint_speed.value = speed
        joint_speed.duration = 0  # Set duration to zero for simultaneous execution
        i += 1
     print("Sending the joint speeds...")
     base.SendJointSpeedsCommand(joint_speeds)
    
    # Sleep between joint movements with different durations
     for duration in durations:
        time.sleep(duration)

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

def example_send_joint_speeds2(base, speeds, durations):

    joint_speeds = Base_pb2.JointSpeeds()

    actuator_count = base.GetActuatorCount().count
# The 7DOF robot will spin in the same direction for 10 seconds
    if actuator_count == 7:
     i = 0
     for speed, duration in zip(speeds, durations):
        joint_speed = joint_speeds.joint_speeds.add()
        joint_speed.joint_identifier = i 
        joint_speed.value = speed
        joint_speed.duration = 0  # Set duration to zero for simultaneous execution
        i += 1
     print("Sending the joint speeds...")
     base.SendJointSpeedsCommand(joint_speeds)
    
    # Sleep between joint movements with different durations
     for duration in durations:
        time.sleep(duration)

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
    
    # Import the utilities helper module
    
    T1 = np.array([[-1, 0, 0, -0.5],
                   [0, 0, 1, 0.5],
                   [0, 1, 0, 0.04],
                   [0, 0, 0, 1]])
    T2 = np.array([[1, 0, 0, -0.5],
                   [0, 0, 1, -0.5],
                   [0, -1, 0, 0.04],
                   [0, 0, 0, 1]])
    angles1 = np.array([359.9996337890625, 15.004989624023438, 179.9993896484375, 229.9951629638672, 359.99853515625, 54.99723434448242, 90.00074768066406])
    angles2 = run_IP(T1,angles1)
    dangles0 = np.empty(7)
    dangles0[0] = smallest_angular_distance_nolimits(angles1[0],angles2[0])
    dangles0[1] = smallest_angular_distance_limits(angles1[1],angles2[1])
    dangles0[2] = smallest_angular_distance_nolimits(angles1[2],angles2[2])
    dangles0[3] = smallest_angular_distance_limits(angles1[3],angles2[3])
    dangles0[4] = smallest_angular_distance_nolimits(angles1[4],angles2[4])
    dangles0[5] = smallest_angular_distance_limits(angles1[5],angles2[5])
    dangles0[6] = smallest_angular_distance_nolimits(angles1[6],angles2[6])
    durations0 = np.abs(dangles0)/SPEED
    speeds1 = dangles0/durations0
    angles3 = run_IP(T2,angles2)
    dangles1 = np.empty(7)
    dangles1[0] = smallest_angular_distance_nolimits(angles2[0],angles3[0])
    dangles1[1] = smallest_angular_distance_limits(angles2[1],angles3[1])
    dangles1[2] = smallest_angular_distance_nolimits(angles2[2],angles3[2])
    dangles1[3] = smallest_angular_distance_limits(angles2[3],angles3[3])
    dangles1[4] = smallest_angular_distance_nolimits(angles2[4],angles3[4])
    dangles1[5] = smallest_angular_distance_limits(angles2[5],angles3[5])
    dangles1[6] = smallest_angular_distance_nolimits(angles2[6],angles3[6])
    durations1 = np.abs(dangles1)/SPEED
    speeds2 = dangles1/durations1
    print("home position angles")
    print(angles1)
    print("diffrence")
    print(dangles0)
    print("pick up position angles")
    print(angles2)
    print("diffrence")
    print(dangles1)
    print("drop off position angles")
    print(angles3)
    print("durations to pick up point")
    print(durations0)
    print("duration to drop off point")
    print(durations1)
    print("home position P/O")
    print(fk(np.deg2rad(angles1)))
    print("pick up position P/O")
    print(fk(np.deg2rad(angles2)))
    print("drop off position P/O")
    print(fk(np.deg2rad(angles3)))
    print(speeds1)
    print(speeds2)
 
    
    # create connection to the device and get the router
        #example core
    success = True
    success &= example_move_to_start_position(base)
    time.sleep(5)
    success &= example_send_joint_speeds2(base, speeds1, durations0)
    time.sleep(5)
    success &= example_send_joint_speeds2(base, speeds2, durations1)

    return 0 if success else 1

if __name__ == "__main__":
    exit(main())
