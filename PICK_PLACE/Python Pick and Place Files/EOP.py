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

# def EOP():
    
# # Create a VideoCapture object
#     # Import the utilities helper module
#     sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
#     import utilities

#     # Parse arguments
#     args = utilities.parseConnectionArguments()
    
#     # Create connection to the device and get the router
#     with utilities.DeviceConnection.createTcpConnection(args) as router:

#         # Create required services
#         base = BaseClient(router)
#         # back_spin(base,-12,1)
#         # Example core
#         success = True
        
     
#         # Create a VideoCapture object
#         # vision_sensor_focus_action_filepath = r'C:\Users\rashe\OneDrive\Documents\Kinova Gen3\Kinova_Python_github\Kinova_Python_github\api_python\examples\Project\focus_350.py'
#         # subprocess.run(['python', vision_sensor_focus_action_filepath])
#         cap = cv2.VideoCapture("rtsp://192.168.1.10/color")
        

#         # Check if the camera is opened successfully
#         if not cap.isOpened():
#             print("Error: Could not open camera.")
#             exit()
#         frame_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
#         frame_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    
#         print(f"Frame Width: {frame_width}")
#         print(f"Frame Height: {frame_height}")
#         # Load the predefined dictionary for ArUco markers
#         aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)

#         # Create a detector parameters object
#         parameters = aruco.DetectorParameters()

#         # Define cameraMatrix and distCoeffs based on your camera calibration
#         cameraMatrix = np.array([[630.89015866, 0, 309.15462136],
#                             [0, 630.47280215, 249.58001608],
#                             [0, 0, 1]], dtype=np.float32)
#         distCoeffs = np.array([[0.00359293],
#                         [0.18266391],
#                         [0.00149938],
#                         [-0.00392911],
#                         [-1.31179912]], dtype=np.float32)  # Assuming no lens distortion

#         translation_vectors = []
#         rotation_vectors = []
#         timestamps = []

#         last_print_time = time.time()
        
#         while True:
            
#             ret, frame = cap.read()
#             if not ret:
#                 print("Error: Cannot read frame.")
#                 break

#             gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#             corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

#             if ids is not None:
#                 aruco.drawDetectedMarkers(frame, corners, ids)
#                 rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, 0.041, cameraMatrix, distCoeffs) #0.041

#                 for i in range(len(ids)):
#                     cv2.drawFrameAxes(frame, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.03)
#                     translation_vectors.append(tvecs[i][0])  # tvecs[i] is a 3D vector for the ith marker
#                     rotation_vectors.append(rvecs[i][0])
#                     timestamps.append(time.time())

#             current_time = time.time()
#             if current_time - last_print_time >= 0.5:
#                 # Filter out data older than 1.5 seconds
#                 valid_indices = [i for i, t in enumerate(timestamps) if current_time - t <= 0.5]
#                 if valid_indices:
#                     avg_tvecs = np.mean([translation_vectors[i] for i in valid_indices], axis=0)
#                     avg_rvecs =  np.mean([rotation_vectors[i] for i in valid_indices], axis=0)
            
#                     # Convert the average rotation vector to a rotation matrix
#                     rotation_matrix, _ = cv2.Rodrigues(avg_rvecs)
#                     pose = base.GetMeasuredCartesianPose()
#                     camera_pose = camera_coor(pose)
#                     rot_m = T(-avg_tvecs[0],-avg_tvecs[1],avg_tvecs[2])
#                     terms4 = [camera_pose,rot_m]
#                     Aruco_matrix = np.linalg.multi_dot(terms4)
#                     A = np.dot(Aruco_matrix,T(0,0,0.0225))
                    
                
                
                
            
#                     # Print the average translation vector
#                     print(f"Average Translation Vector - X: {-avg_tvecs[0]:.5f}, Y: {-avg_tvecs[1]:.5f}, Z: {avg_tvecs[2]:.5f} meters")
            
#                      # Print the rotation matrix
#                     print("Average Rotation Matrix:")
#                     print(np.array_str(rotation_matrix, precision=5, suppress_small=True))
                
                
#                     # Print the Camera pose matrix
#                     print("Camera pose Matrix:")
#                     print(camera_pose)
                
#                     # Print the Aruco marker orientation matrix
#                     print("Aruco marker Matrix:")
#                     print(Aruco_matrix)
                    
#                     # Print the Aruco marker orientation matrix
#                     print("Pick_up Matrix:")
#                     print(A)
                    
#                     print(ids)
                    
                    
            
#                 # Reset the containers or remove old data
#                 translation_vectors = [translation_vectors[i] for i in valid_indices]
#                 rotation_vectors = [rotation_vectors[i] for i in valid_indices]
#                 timestamps = [timestamps[i] for i in valid_indices]

#                 last_print_time = current_time


#             # Display the resized frame
#             cv2.imshow('Camera Feed', frame)
#             if cv2.waitKey(1) & 0xFF == ord('q'):
#                 break

#         cap.release()
#         cv2.destroyAllWindows()

#         return 0 if success else 1


# def EOP():
#     cap = cv2.VideoCapture("rtsp://192.168.1.10/color")

#     if not cap.isOpened():
#         print("Error: Could not open camera.")
#         return 1

#     cameraMatrix = np.array([[630.89015866, 0, 309.15462136],
#                              [0, 630.47280215, 249.58001608],
#                              [0, 0, 1]], dtype=np.float32)
#     distCoeffs = np.array([0.00359293, 0.18266391, 0.00149938, -0.00392911, -1.31179912], dtype=np.float32)

#     aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
#     parameters = aruco.DetectorParameters()

#     while True: 
#         ret, frame = cap.read()
#         if not ret:
#             print("Error: Cannot read frame.")
#             break

#         gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#         corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

#         if ids is not None:
#             aruco.drawDetectedMarkers(frame, corners, ids)
#             rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, 0.041, cameraMatrix, distCoeffs)

#             for i, marker_id in enumerate(ids.flatten()):
#                 print(f"Marker ID: {marker_id}")
#                 print(f"Translation Vector: {tvecs[i][0]}")
#                 print(f"Rotation Vector: {rvecs[i][0]}")

#         cv2.imshow('Camera Feed', frame)
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break

#     cap.release()
#     cv2.destroyAllWindows()
#     return 0

# # Global variable to store the matrices
# marker_matrices = {
#     "small": None,
#     "medium": None,
#     "large": None,
#     "base": None
}

def EOP():
    # # global marker_matrices
    # vision_sensor_focus_action_filepath = r'C:\Users\rashe\OneDrive\Documents\Kinova Gen3\Kinova_Python_github\Kinova_Python_github\api_python\examples\Project\focus_350.py'
    # subprocess.run(['python', vision_sensor_focus_action_filepath])
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
                        # rotation_matrix, _ = cv2.Rodrigues(rvecs)
                        # pose = base.GetMeasuredCartesianPose()
                        # camera_pose = camera_coor(pose)
                        # rot_m = T(-tvecs[0][0][0], -tvecs[0][0][1], tvecs[0][0][2])
                        # terms4 = [camera_pose, rot_m]
                        # Aruco_matrix = np.linalg.multi_dot(terms4)
                        # A = np.dot(Aruco_matrix, T(0, 0, (s/2)))
                        # marker_matrices[category] = A
                    elif marker_id in range(7, 12):
                        size = 0.04
                        category = "medium"
                        rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners[i:i+1], size, cameraMatrix, distCoeffs)
                        # rotation_matrix, _ = cv2.Rodrigues(rvecs)
                        # pose = base.GetMeasuredCartesianPose()
                        # camera_pose = camera_coor(pose)
                        # rot_m = T(-tvecs[0][0][0], -tvecs[0][0][1], tvecs[0][0][2])
                        # terms4 = [camera_pose, rot_m]
                        # Aruco_matrix = np.linalg.multi_dot(terms4)
                        # A = np.dot(Aruco_matrix, T(0, 0, (size/2)))
                        # marker_matrices[category] = A
                    elif marker_id in range(13, 18):
                        size = 0.027
                        category = "small"
                        rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners[i:i+1], size, cameraMatrix, distCoeffs)
                        # rotation_matrix, _ = cv2.Rodrigues(rvecs)
                        # pose = base.GetMeasuredCartesianPose()
                        # camera_pose = camera_coor(pose)
                        # rot_m = T(-tvecs[0][0][0], -tvecs[0][0][1], tvecs[0][0][2])
                        # terms4 = [camera_pose, rot_m]
                        # Aruco_matrix = np.linalg.multi_dot(terms4)
                        # A = np.dot(Aruco_matrix, T(0, 0, (size/2)))
                        # marker_matrices[category] = A
                    elif marker_id == 0:
                        size = 0.0535
                        category = "base"
                        rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners[i:i+1], size, cameraMatrix, distCoeffs)
                        # rotation_matrix, _ = cv2.Rodrigues(rvecs)
                        # pose = base.GetMeasuredCartesianPose()
                        # camera_pose = camera_coor(pose)
                        # rot_m = T(-tvecs[0][0][0], -tvecs[0][0][1], tvecs[0][0][2])
                        # terms4 = [camera_pose, rot_m]
                        # marker_matrices = np.linalg.multi_dot(terms4)
                        # A = np.dot(marker_matrices, T(0, 0, (size/2)))
                        # marker_matrices[category] = A
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
