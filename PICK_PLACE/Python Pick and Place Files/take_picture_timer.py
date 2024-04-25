# this file runs the camera and takes pictures every time interval specified in line 34
import cv2
import numpy as np
import datetime  # To generate unique filenames based on the current time
import time  # Importing the time module
import subprocess
vision_sensor_focus_action_filepath = r'C:\Users\rashe\OneDrive\Documents\Kinova Gen3\Kinova_Python_github\Kinova_Python_github\api_python\examples\Project\focus_350.py'
subprocess.run(['python', vision_sensor_focus_action_filepath])
def save_image(frame, prefix="capture_", ext=".jpg"):
    # Generate a unique filename with a timestamp
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{prefix}{timestamp}{ext}"
    cv2.imwrite(filename, frame)
    print(f"Image saved successfully as {filename}.")

# Create a VideoCapture object
cap = cv2.VideoCapture("rtsp://192.168.1.10/color")
# Set the resolution to 1920x1080
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)


# Initialize the time tracker
last_capture_time = time.time()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Cannot read frame.")
        break
    cv2.imshow('Camera Feed', frame)

    # Check the time elapsed since the last capture
    if time.time() - last_capture_time > 2.5:  # More than 2 seconds have passed
        save_image(frame)  # Save the frame
        last_capture_time = time.time()  # Reset the last capture time

    # Exit loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
