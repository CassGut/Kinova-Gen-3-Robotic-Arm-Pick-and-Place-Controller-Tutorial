# this file is used to generate aruco markers which then put on the cubes 
import cv2
import cv2.aruco as aruco
import numpy as np

# Define the dictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)

# Specify the number of markers you want to generate
num_markers = 19

# Marker size in pixels; adjust this to make the marker larger
# For example, 500x500 pixels will make each marker larger and more easily detected from a distance
marker_size = 175  # Increase this for larger markers

for i in range(num_markers):
    # Generate marker
    marker_image = np.zeros((marker_size, marker_size), dtype=np.uint8)
    marker_image = aruco.generateImageMarker(aruco_dict, i, marker_size, marker_image, 1)

    # Save the marker to file
    cv2.imwrite(f'marker_{i}.png', marker_image)
