import sys
import os
import cv2
import cv2.aruco as aruco
import numpy as np
import time
from get_camera_pose import camera_pose


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

        camera_transformation = camera_pose()  # Get the camera's pose relative to the base

        for i in range(len(ids)):
            rvec, tvec = rvecs[i], tvecs[i]
            R, _ = cv2.Rodrigues(rvec)  # Convert rvec to rotation matrix

            # Marker's transformation matrix relative to the camera
            marker_transformation = np.hstack((R, tvec.reshape(3, 1)))
            marker_transformation = np.vstack((marker_transformation, [0, 0, 0, 1]))

            # Calculate marker's pose relative to the base
            marker_pose_base = np.dot(camera_transformation, marker_transformation)

            # Now you have marker_pose_base as the 4x4 transformation matrix of the marker relative to the base
            # Use marker_pose_base for whatever next steps you need

    cv2.imshow('Camera Feed', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

