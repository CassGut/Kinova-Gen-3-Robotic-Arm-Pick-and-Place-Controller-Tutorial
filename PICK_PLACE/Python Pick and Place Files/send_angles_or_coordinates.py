# this fucntion send angles or cartesian vectors to the robot and exceutes the movement
# cartesian_action_movement (change the vectors in the function definition) this will send the robot to the set coordinates
# angular_action_movement (set the angles inside the function definition) this will send the robot to the set joint angles
# move (set the delta vectors in the function call) the order is x y and z this will change the coordinates of the robot by values of xyz while keeping the orienation the same
# remember that you are not controlling trajectory given movemnt will automatically have a staright line trajectory 
# the robot might stall if the stright line trajectory is not possible 
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

def angular_action_movement(base,angle_values):
    
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

def cartesian_action_movement(base, base_cyclic):
    
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
        q = [ 180, 0, 0, -45, 0, -120.3, 90]
        
        # Example core
        success = True

        success &= example_move_to_home_position(base)
        # success &= cartesian_action_movement(base, base_cyclic)
        # #success &= angular_action_movement(base,q)
        # pose = base.GetMeasuredCartesianPose()
        # success &= move(base, base_cyclic, pose, 0.1, 0.1, 0.1)
        print(get_angles_pose(base))
        print(base.GetMeasuredCartesianPose())


        return 0 if success else 1

if __name__ == "__main__":
    exit(main())


