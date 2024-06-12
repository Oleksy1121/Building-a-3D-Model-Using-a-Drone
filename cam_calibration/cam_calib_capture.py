# following code captures 10 images from Tello drone's camera for calibration purposes
# please use 5x7 checkerboard pattern (see https://docs.opencv.org/4.x/da/d0d/tutorial_camera_calibration_pattern.html)

# code bellow will capture single frame from drone's camera when spacebar is pressed - this allows to align camera between each snapshot
# for the best calibration, please rotate and move checkerboard within field of view of drone's camera for most varied images


from djitellopy import *
from threading import Thread, Event
import keyboard, os, time, csv
import matplotlib.pyplot as plt
import cv2


tello = Tello()

tello.connect()

keep_recording = True
tello.streamon()

while tello.get_frame_read() is None or tello.get_frame_read().frame is None:
    print("frame not ready yet...")
frame_read = tello.get_frame_read()

def camera_feed():
    height, width, _ = frame_read.frame.shape

    while keep_recording:
        img = frame_read.frame
        img = cv2.resize(img, (width, height))
        cv2.imshow("camera", img)
        cv2.waitKey(1)

recording = Thread(target=camera_feed)
recording.start()

for i in range(1, 21):
    print("press space when ready...")
    keyboard.wait('space')
    cv2.imwrite("img_%d.jpeg" % i, frame_read.frame)
    print("frame %d/20 captured" % i)

keep_recording = False
recording.join()

tello.streamoff()

print("Images for calibration ready. Please review them and then move them to 'images' directory.")
