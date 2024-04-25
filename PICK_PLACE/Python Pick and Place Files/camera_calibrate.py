# this file is used to find the instrinsic camera parameters in the form a camera matrix 
# line 19 needs to be changed to where the calibration pictures are saved 
import numpy as np
import cv2 as cv
import glob

# Termination criteria for the iterative algorithm used in corner refinement.
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points based on the chessboard dimensions (7x6 here)
objp = np.zeros((6*8, 3), np.float32)*0.026
objp[:, :2] = np.mgrid[0:6, 0:8].T.reshape(-1, 2)

# Arrays to store object points and image points from all images.
objpoints = []  # 3D points in real world space
imgpoints = []  # 2D points in image plane.

# Path to where your calibration images are stored, use '*' for selecting all jpg files
images = glob.glob(r'C:\Users\rashe\OneDrive\Documents\Kinova Gen3\Kinova_Python_github\Kinova_Python_github\api_python\examples\Project\Kiki_cab_pics\*.jpg')


# Ensure you load exactly 20 images
assert len(images) >= 2, "Not enough images found. Make sure there are at least 20 images."

for fname in images[:180]:  # Process only the first 20 images found
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # Find the chessboard corners
    ret, corners = cv.findChessboardCorners(gray, (6, 8), None)

    # If found, add object points, image points (after refining them)
    if ret:
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)

        # Optional: Draw and display the corners
        cv.drawChessboardCorners(img, (6, 8), corners2, ret)
        cv.imshow('img', img)
        cv.waitKey(500)

cv.destroyAllWindows()

# Camera calibration
ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# Print the camera matrix
print("Camera matrix:\n", mtx)

# # Assuming you want to undistort an image using the calculated matrix
# # Make sure to change 'left12.jpg' to an actual image you want to undistort.
# img_path = ''  # Update this path
# img = cv.imread(img_path)
# h, w = img.shape[:2]
# newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))

# # Undistort using the first method
# dst = cv.undistort(img, mtx, dist, None, newcameramtx)

# # Crop the image based on the ROI
# x, y, w, h = roi
# dst = dst[y:y+h, x:x+w]

# # Save or display the undistorted image
# cv.imwrite('calibresult.png', dst)
# cv.imshow('Undistorted Image', dst)
# cv.waitKey(0)
# cv.destroyAllWindows()



