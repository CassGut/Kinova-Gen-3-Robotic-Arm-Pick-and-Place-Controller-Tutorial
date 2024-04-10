import sys
import os
import np
import cv2
import cv2.aruco as aruco
import time
from Rx import Rx
from Ry import Ry
from Rz import Rz
from T import T


from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient

from kortex_api.autogen.messages import Base_pb2
from kortex_api.Exceptions.KServerException import KServerException


def get_pose(base):
    # Current arm's joint angles (in home position)
    try:
        print("Getting Angles for every joint...")
        input_joint_angles = base.GetMeasuredJointAngles()
    except KServerException as ex:
        print("Unable to get joint angles")
        print("Error_code:{} , Sub_error_code:{} ".format(ex.get_error_code(), ex.get_error_sub_code()))
        print("Caught expected error: {}".format(ex))
        return False
    # Computing Foward Kinematics (Angle -> cartesian convert) from arm's current joint angles
    try:
        print("Computing Foward Kinematics using joint angles...")
        pose = base.ComputeForwardKinematics(input_joint_angles)
    except KServerException as ex:
        print("Unable to compute forward kinematics")
        print("Error_code:{} , Sub_error_code:{} ".format(ex.get_error_code(), ex.get_error_sub_code()))
        print("Caught expected error: {}".format(ex))
        return False

    
    return pose

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
        cap = cv2.VideoCapture("rtsp://192.168.1.10/color")

        if not cap.isOpened():
            print("Error: Could not open camera.")
            exit()

        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        parameters = aruco.DetectorParameters()

        cameraMatrix = np.array([[642.14257641, 0., 317.1572718],
                                 [0., 641.17787669, 247.15006822],
                                 [0., 0., 1.]], dtype=np.float32)

        distCoeffs = np.zeros((5, 1), dtype=np.float32)

        while True:
            ret, frame = cap.read()
            if not ret:
                print("Error: Cannot read frame.")
                break

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

            if ids is not None:
                aruco.drawDetectedMarkers(frame, corners, ids)
                rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, 0.027, cameraMatrix, distCoeffs)  # Marker size needs to be specified
                pose = get_pose(base)
                camera_transformation = camera_coor(pose)  # Get the camera's pose relative to the base

                for i in range(len(ids)):
                    rvec, tvec = rvecs[i], tvecs[i]
                    R, _ = cv2.Rodrigues(rvec)  # Convert rvec to rotation matrix

                    # Marker's transformation matrix relative to the camera
                    marker_transformation = np.hstack((R, tvec.reshape(3, 1)))
                    marker_transformation = np.vstack((marker_transformation, [0, 0, 0, 1]))

                    # Calculate marker's pose relative to the base
                    marker_pose_base = np.dot(camera_transformation, marker_transformation)
                    print(marker_pose_base)

                    # Now you have marker_pose_base as the 4x4 transformation matrix of the marker relative to the base
                    # Use marker_pose_base for whatever next steps you need

            cv2.imshow('Camera Feed', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()
        
if __name__ == "__main__":
    exit(main())

