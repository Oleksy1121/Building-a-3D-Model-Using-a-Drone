# modified code taken from official OpenCV page (https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html)
# run this code in cam_calibration directory where images of checkerboad are stores in directory 'images' - obtained by running cam_calib_capture.py


import os, time
import glob
import cv2
import numpy as np

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
objp = np.zeros((5*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:5].T.reshape(-1,2)
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
images = glob.glob(os.path.join("images", "*.jpeg"))

if not os.path.exists("images"):
    print("Directory 'images' not found! Please ensure your current working directory is 'cam_calibration'.")
    exit()

for fname in images:
    print("analysing ", fname)
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (7,5), None)
    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)
        # Draw and display the corners
        cv2.drawChessboardCorners(img, (7,5), corners2, ret)
        cv2.imshow('img', img)
        cv2.waitKey(250)

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
np.savez("calibration_tello", ret=ret, mtx=mtx, dist=dist, rvecs=rvecs, tvecs=tvecs)

print(mtx)
print("fx", mtx[0][0], "fy", mtx[1][1])

cv2.destroyAllWindows()
