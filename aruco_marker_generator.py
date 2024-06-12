### simple script to ARUCO marker with set ID for the project


import sys
import cv2
import numpy as np

MARKER_ID = 10

tag = np.zeros((300, 300, 1), dtype="uint8")
cv2.aruco.drawMarker(cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50), MARKER_ID, 300, tag)

cv2.imwrite("tello_aruco_marker.png", tag)
cv2.imshow("ARUCO marker", tag)
cv2.waitKey(0)
