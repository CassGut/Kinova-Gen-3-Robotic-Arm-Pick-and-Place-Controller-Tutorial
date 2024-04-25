# running this file will show you the computer vision which detects the aruco code cubes
# make sure the file path in line 41 is correct for the location of the focus350 file
# this is the camera focus action file 
import subprocess
import cv2
import cv2.aruco as aruco
from T import T
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
import sys
import os
import time
import numpy as np
from Rx import Rx
from Ry import Ry
from Rz import Rz


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
    terms2= [M,T(0, 0.054, -0.138)]
    # terms2= [M,T(0, 0.054, -0.124)]
    C = np.linalg.multi_dot(terms2)
    return C


def EOP():
    global marker_matrices
    vision_sensor_focus_action_filepath = r'C:\Users\rashe\OneDrive\Documents\Kinova Gen3\Kinova_Python_github\Kinova_Python_github\api_python\examples\Project\focus_350.py'
    subprocess.run(['python', vision_sensor_focus_action_filepath])
# Create a VideoCapture object
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
            return 1

        cameraMatrix = np.array([[643.77840171, 0, 311.5361204],
                                 [0, 643.99115635, 248.9306098],
                                 [0, 0, 1]], dtype=np.float32)
        distCoeffs = np.array([0.01331069, 0.1154656, 0.00361715, -0.00244894, -1.04813852], dtype=np.float32)

        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        parameters = aruco.DetectorParameters()

        last_print_time = time.time()

        while True:
            ret, frame = cap.read()
            if not ret:
                print("Error: Cannot read frame.")
                break

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

            if ids is not None:
                for i, marker_id in enumerate(ids.flatten()):
                    # Determine the marker size and category
                    if marker_id in range(1, 6):
                        size = 0.0535
                        s = 0.06
                        marker_size = 0.0535  # marker size
                        cube_size = 0.06
                        category = "large"
                        rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners[i:i+1], marker_size, cameraMatrix, distCoeffs)
                        aruco.drawDetectedMarkers(frame, corners, ids)
                        rotation_matrix, _ = cv2.Rodrigues(rvecs)
                        pose = base.GetMeasuredCartesianPose()
                        camera_pose = camera_coor(pose)
                        rot_m = T(-tvecs[0][0][0], -tvecs[0][0][1], tvecs[0][0][2])
                        terms4 = [camera_pose, rot_m]
                        Aruco_matrix = np.linalg.multi_dot(terms4)
                        A = np.dot(Aruco_matrix, T(0, 0, (s/2)))
                        marker_matrices[category] = A
                    elif marker_id in range(7, 12):
                        size = 0.04
                        category = "medium"
                        rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners[i:i+1], size, cameraMatrix, distCoeffs)
                        rotation_matrix, _ = cv2.Rodrigues(rvecs)
                        pose = base.GetMeasuredCartesianPose()
                        camera_pose = camera_coor(pose)
                        rot_m = T(-tvecs[0][0][0], -tvecs[0][0][1], tvecs[0][0][2])
                        terms4 = [camera_pose, rot_m]
                        Aruco_matrix = np.linalg.multi_dot(terms4)
                        A = np.dot(Aruco_matrix, T(0, 0, (size/2)))
                        marker_matrices[category] = A
                    elif marker_id in range(13, 18):
                        size = 0.027
                        category = "small"
                        rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners[i:i+1], size, cameraMatrix, distCoeffs)
                        rotation_matrix, _ = cv2.Rodrigues(rvecs)
                        pose = base.GetMeasuredCartesianPose()
                        camera_pose = camera_coor(pose)
                        rot_m = T(-tvecs[0][0][0], -tvecs[0][0][1], tvecs[0][0][2])
                        terms4 = [camera_pose, rot_m]
                        Aruco_matrix = np.linalg.multi_dot(terms4)
                        A = np.dot(Aruco_matrix, T(0, 0, (size/2)))
                        marker_matrices[category] = A
                    elif marker_id == 0:
                        size = 0.0535
                        category = "base"
                        rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners[i:i+1], size, cameraMatrix, distCoeffs)
                        rotation_matrix, _ = cv2.Rodrigues(rvecs)
                        pose = base.GetMeasuredCartesianPose()
                        camera_pose = camera_coor(pose)
                        rot_m = T(-tvecs[0][0][0], -tvecs[0][0][1], tvecs[0][0][2])
                        terms4 = [camera_pose, rot_m]
                        marker_matrices = np.linalg.multi_dot(terms4)
                        A = np.dot(marker_matrices, T(0, 0, (size/2)))
                        marker_matrices[category] = A
                    else:
                        continue

            current_time = time.time()
            if current_time - last_print_time >= 2:
                print(marker_matrices)
                print(-tvecs[0][0][0], -tvecs[0][0][1], -tvecs[0][0][2])
                last_print_time = current_time

            cv2.imshow('Camera Feed', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()
    return 0
# Example usage
if __name__ == "__main__":
        EOP() 
