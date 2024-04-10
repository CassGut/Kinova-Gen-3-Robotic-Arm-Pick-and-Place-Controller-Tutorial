import keyboard  # Import the keyboard module
import threading
import time



from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.messages import Session_pb2, Base_pb2
import sys
import os
import numpy as np
from kortex_api.Exceptions.KServerException import KServerException
from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2
TIMEOUT_DURATION = 100
# Assume all other necessary imports are done here

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

def look_position(base):
    
    print("Starting angular action movement ...")
    action = Base_pb2.Action()
    action.name = "Example angular action movement"
    action.application_data = ""

    actuator_count = base.GetActuatorCount()

    # Place arm straight up
    i=0
    #[359.99945068359375, 285.90997314453125, 180.03041076660156, 225.782958984375, 359.9552917480469, 280.2957763671875, 89.78185272216797]
    angle_values = [ 360, 286, 180, 225, 0, 280, 90]
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
    
def spin(base, SPEED, pause_time, should_spin):
    actuator_count = base.GetActuatorCount().count
    if actuator_count == 7:
        speed6 = 25  # Initial speed for the 6th joint
        
        while should_spin.is_set():  # Use a threading.Event to control the spinning
            joint_speeds = Base_pb2.JointSpeeds()
            speeds = [SPEED, 0, 0, 0, 0, 0, speed6]
            
            for i, speed in enumerate(speeds):
                joint_speed = joint_speeds.joint_speeds.add()
                joint_speed.joint_identifier = i
                joint_speed.value = speed
                joint_speed.duration = 0
                
            base.SendJointSpeedsCommand(joint_speeds)
            
            time.sleep(pause_time)  # Wait for pause_time seconds before possibly changing the speed
            speed6 = -speed6  # Flip the speed for the next iteration
        
        base.Stop()  # Stop the joint movements immediately
        print("Spin stopped.")

def look_for_marker():
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    import utilities
    
    args = utilities.parseConnectionArguments()
    
    with utilities.DeviceConnection.createTcpConnection(args) as router:
        base = BaseClient(router)
        base_cyclic = BaseCyclicClient(router)
        look_position(base)
        # Setup part omitted for brevity
        should_spin = threading.Event()
        spin_thread = threading.Thread(target=spin, args=(base, 20, 0.15, should_spin))
        try:
            # Initially not spinning
            while True:
                print("Press space to toggle spinning.")
                keyboard.wait('space')  # Wait for space bar press
            
                if not should_spin.is_set():
                    should_spin.set()  # Start spinning
                    if not spin_thread.is_alive():
                        spin_thread = threading.Thread(target=spin, args=(base, 20, 0.15, should_spin))
                        spin_thread.start()
                else:
                    should_spin.clear()  # Stop spinning
            
        except KeyboardInterrupt:
            should_spin.clear()  # Ensure spinning is stopped if script is interrupted
            spin_thread.join()  # Wait for the spin_thread to finish if it's running

if __name__ == "__main__":
    look_for_marker()

