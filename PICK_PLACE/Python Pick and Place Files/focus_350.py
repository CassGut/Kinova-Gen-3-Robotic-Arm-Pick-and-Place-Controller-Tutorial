# this is a focus action command its used to focus the camera when using computer vision to find the coordinates of the cubes 
import sys
import os
import time

from kortex_api.autogen.client_stubs.VisionConfigClientRpc import VisionConfigClient
from kortex_api.autogen.client_stubs.DeviceManagerClientRpc import DeviceManagerClient

from kortex_api.autogen.messages import DeviceConfig_pb2, Session_pb2, DeviceManager_pb2, VisionConfig_pb2

#
# Returns the device identifier of the Vision module, 0 if not found
#
def example_vision_get_device_id(device_manager):
    vision_device_id = 0
    
    # Getting all device routing information (from DeviceManagerClient service)
    all_devices_info = device_manager.ReadAllDevices()

    vision_handles = [ hd for hd in all_devices_info.device_handle if hd.device_type == DeviceConfig_pb2.VISION ]
    if len(vision_handles) == 0:
        print("Error: there is no vision device registered in the devices info")
    elif len(vision_handles) > 1:
        print("Error: there are more than one vision device registered in the devices info")
    else:
        handle = vision_handles[0]
        vision_device_id = handle.device_identifier
        print("Vision module found, device Id: {0}".format(vision_device_id))

    return vision_device_id


def example_wait_for_focus_action():
    print("-- Waiting for 2 seconds to observe the effects of the focus action... --")
    time.sleep(2)


def example_routed_vision_do_autofocus_action(vision_config, vision_device_id):
    sensor_focus_action = VisionConfig_pb2.SensorFocusAction()
    sensor_focus_action.sensor = VisionConfig_pb2.SENSOR_COLOR
    sensor_focus_action.focus_action = VisionConfig_pb2.FOCUSACTION_START_CONTINUOUS_FOCUS
    vision_config.DoSensorFocusAction(sensor_focus_action, vision_device_id)

    example_wait_for_focus_action()



def main():
    # Import the utilities helper module
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    import utilities

    # Parse arguments
    args = utilities.parseConnectionArguments()
    
    # Create connection to the device and get the router
    with utilities.DeviceConnection.createTcpConnection(args) as router:

        device_manager = DeviceManagerClient(router)
        vision_config = VisionConfigClient(router)

        # example core
        vision_device_id = example_vision_get_device_id(device_manager)

        if vision_device_id != 0:
            example_routed_vision_do_autofocus_action(vision_config, vision_device_id)

if __name__ == "__main__":
    main()

