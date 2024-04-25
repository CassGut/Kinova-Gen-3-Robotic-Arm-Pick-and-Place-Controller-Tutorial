# this file is used to find the instrinsic camera parameters in the form a camera matrix 
# line 20 needs to be changed to where the calibration pictures are saved 
import numpy as np
import cv2 as cv
import glob

# Termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# Assuming each square is 0.026 mm 
square_size = 0.026  # in millimeters
# Object points
objp = np.zeros((6*8, 3), np.float32)
objp[:, :2] = np.mgrid[0:6, 0:8].T.reshape(-1, 2)* square_size

# Arrays to store points
objpoints = []  # 3D points
imgpoints = []  # 2D points

# Path to calibration images
images = glob.glob(r'C:\Users\rashe\OneDrive\Documents\Kinova Gen3\Kinova_Python_github\Kinova_Python_github\api_python\examples\Project\Kiki_cab_pics_pics\*.jpg')


# Other parts of the code remain the same...

for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    ret, corners = cv.findChessboardCorners(gray, (6, 8), None)
    if ret==True:
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)
        cv.drawChessboardCorners(img, (6,8), corners2, ret)
        cv.imshow('img', img)
        cv.waitKey(1000)
        
cv.destroyAllWindows()

ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

print("Camera matrix:\n", mtx)



