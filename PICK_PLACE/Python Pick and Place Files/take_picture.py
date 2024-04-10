import cv2
import numpy as np
import datetime  # To generate unique filenames based on the current time


def save_image(frame, prefix="capture_", ext=".jpg"):
    # Generate a unique filename with a timestamp
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{prefix}{timestamp}{ext}"
    cv2.imwrite(filename, frame)
    print(f"Image saved successfully as {filename}.")

# Create a VideoCapture object
cap = cv2.VideoCapture("rtsp://192.168.1.10/color")
# cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Cannot read frame.")
        break

    
    cv2.imshow('Zoomed Camera Feed', frame)

    # Check if spacebar (' ') is pressed to capture an image
    if cv2.waitKey(1) & 0xFF == ord(' '):
        save_image(frame)  # Save the zoomed frame

    # Exit loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

