### This is main project file for IiSRL labratory's project 'Building 3D model based on series of photos from Tello drone'
###
### Main class structure has been based on code provided during labratories as skeletal framework for interacting with Ryze Tello drone
###
### Script performs following taks:
###    > drone takeoff
###    > discovering ARUCO marker and centering on it
###    > detecting QR code and reading its data for further flight parameters
###    > aligning drone on startup position in reference to ARUCO marker based on data from QR code
###    > performing repeated circular trajectory using series of rotate-move steps
###    > taking snapshots of the center of traced orbit at each step
###    > safe landing
###


from djitellopy import *
import logging
from threading import Thread, Event, Lock
import keyboard, os, time, csv
import matplotlib.pyplot as plt
import math
from simple_pid import PID
import cv2
import numpy as np
from pyzbar.pyzbar import decode
import json
from enum import Enum, auto
from dotmap import DotMap


### CONSTANTS ###

# ARUCO MARKER DETECTION
ARUCO_MARKER_DICT_TYPE = cv2.aruco.DICT_4X4_50
ARUCO_MARKER_ID = 10

# QR CODE DETECTION
QR_CODE_PREAMBLE = "IiSRL__TELLO_LAB_L2_G5"

# FLIGHT PARAMETERS
SIMULATION_MODE = True # set this variable to False to enable drone flying
MAX_SPEED_CLIP = 50 # sets allowed percentage of drone's absolute max speed in any direction
ORBIT_SEGMENTS = 10 # each circle trajectory will be divided into N segments and at each end point photo will be taken


# enum representing state machine of whole drone control (do not modify)
class DroneState(Enum):
    # general startup states
    FAILURE = "FAILURE"
    UNINITIALIZED = "UNINITIALIZED"
    CONNECTED = "CONNECTED"
    SETUP_READY = "SETUP_READY"
    TAKE_OFF = "TAKE_OFF"

    # states for locating and analyzing ARUCO marker / QR code
    MARKER_DISCOVERY_SEARCH = "MARKER_DISCOVERY_SEARCH"
    MARKER_DISCOVERY_FOUND = "MARKER_DISCOVERY_FOUND"
    MARKER_DISCOVERY_CENTERING = "MARKER_DISCOVERY_CENTERING"
    MARKER_DISCOVERY_DECODING = "MARKER_DISCOVERY_DECODING"
    MARKER_DISCOVERY_DONE = "MARKER_DISCOVERY_DONE"

    # states for aligning drone to origin of trajectory
    MARKER_ALIGNING_SEARCH = "MARKER_ALIGNING_SEARCH"
    MARKER_ALIGNING_FOUND = "MARKER_ALIGNING_FOUND"
    MARKER_ALIGNING_POSITIONING = "MARKER_ALIGNING_POSITIONING"
    MARKER_ALIGNING_DONE = "MARKER_ALIGNING_DONE"

    # states for performing circular trajectory
    ORBITING = "ORBITING"
    ORBITING_COMPLETE = "ORBITING_COMPLETE"

    # states for completing drone tasks
    TRAJECTORY_COMPLETE = "TRAJECTORY_COMPLETE"
    LANDING = "LANDING"
    DONE = "DONE"

    # helper function helps printing enum value
    def __str__(self):
        return str(self.value)


# class for storing current drone state and reporting its changes in human-readable format
class DroneStateVariable:
    def __init__(self, value=DroneState.UNINITIALIZED):
        self._value = value

    def set(self, new_value):
        print("====== drone state change: ( %s --> %s ) ======" % (self._value, new_value))
        self._value = new_value

    def get(self):
        return self._value


# main program class responsible for performing all tasks
class TelloController:

    # provided class to execute kill switch when spacebar is pressed any moment
    class TelloKillSwitch(Thread):
        tc_handler = None

        def __init__(self, tc_handler):
            Thread.__init__(self)
            self.tc_handler = tc_handler

        def run(self):
            keyboard.wait('space')
            self.tc_handler.force_emergency_stop()

    # kill switch - send emergency command to kill all drone's motors
    def force_emergency_stop(self):
        print("!!! EMERGENCY STOP !!!")
        self.tello_drone.send_rc_control(0, 0, 0, 0) # this command does not wait for response
        # self.tello_drone.land() # uncomment only if you are sure what you are doing - this command is blocking
        self.flight_done = True
        self.tello_drone.emergency()
        self.stop_controller.set()

    # provided class for periodically calling callback function
    class TelloTimer(Thread):
        interval = 1.0
        running = None
        func = None

        def __init__(self, interval, event, func):
            Thread.__init__(self)
            self.running = event
            if interval > 0:
                self.interval = interval
            self.func = func

        def run(self):
            while not self.running.wait(self.interval):
                self.func()

    # function collecting data from drone and storing it in CSV file    
    def get_sensor_data(self):
        battery = self.tello_drone.get_battery()

        accelX = self.tello_drone.get_acceleration_x()
        accelY = self.tello_drone.get_acceleration_y()
        accelZ = self.tello_drone.get_acceleration_z()

        roll = self.tello_drone.get_roll()
        pitch = self.tello_drone.get_pitch()
        yaw = self.tello_drone.get_yaw()

        speedX = self.tello_drone.get_speed_x()
        speedY = self.tello_drone.get_speed_y()
        speedZ = self.tello_drone.get_speed_z()

        flightTime = self.tello_drone.get_flight_time()

        self.csv_writer.writerow([accelX, accelY, accelZ, speedX, speedY, speedZ, roll, pitch, yaw, flightTime, battery])

    # function to report battery condition in terminal
    def monitor_battery(self):
        print("battery: ", self.tello_drone.get_battery())

    # main function responsible for all flight procedures
    def flight_function(self):
        self.state.set(DroneState.TAKE_OFF)

        # in simulation mode no flight commands are send to drone
        if self.flight_mode:
            self.tello_drone.takeoff()
            time.sleep(1) # wait for drone to stabilize
        else:
            print("Non-flying mode - no takeoff...")
        self.state.set(DroneState.MARKER_DISCOVERY_SEARCH)
        time.sleep(2)

        # wait for camera handling thread to detect ARUCO marker
        # rotate drone around in case marker is outside field of vision
        for i in range (1, 25):
            time.sleep(1)
            if self.state.get() == DroneState.MARKER_DISCOVERY_SEARCH:
                if self.flight_mode:
                    self.tello_drone.rotate_clockwise(15)
            else:
                break
        if self.state.get() == DroneState.MARKER_DISCOVERY_SEARCH:
            print("marker not found!")
            print("terminating flight...")
            return

        if not self.camera.is_ready:
            print("! camera feed is not ready !")
            return

        # perform rough drone centering on ARUCO marker (marker size is not known yet so use visible marker proportions)
        if self.state.get() == DroneState.MARKER_DISCOVERY_FOUND:
            self.state.set(DroneState.MARKER_DISCOVERY_CENTERING)

            # roughly calculate marker distance from image center in units of its size
            def calculate_proportional_displacement(location, size, screen_center):
                return (location - screen_center) / size

            
            ## create set of PID controlers for centering (individual for each axis)
            # X-axis is directed forward
            x_pid = PID(13, 0.02 , 0.06, setpoint=2)
            x_pid.sample_time = 0.1
            # Y-axis is directed to the right
            y_pid = PID(15, 0.08, 0.0, setpoint=0)
            y_pid.sample_time = 0.1
            # Z-axis is directed downward
            z_pid = PID(4.5, 0.4, 0.05, setpoint=0) 
            z_pid.sample_time = 0.1
            # yaw angle is in range (-180, 180)
            yaw_pid = PID(0.75, 0.04, -0.02, setpoint=0)
            yaw_pid.sample_time = 0.1

            max_speed = abs(MAX_SPEED_CLIP) # sanitize value for safety

            i = 0

            while self.state.get() == DroneState.MARKER_DISCOVERY_CENTERING:
                # be sure data updated by camera handling thread is not invalid
                with self.thread_lock:
                    marker_is_visible = self.current_marker_data.is_visible
                    marker_x = self.current_marker_data.x
                    marker_y = self.current_marker_data.y
                    marker_w = self.current_marker_data.w
                    marker_h = self.current_marker_data.h
                    angle = self.current_marker_data.angle
                    self.speed_commands.x_percentage = 0.0
                    self.speed_commands.y_percentage = 0.0
                    self.speed_commands.z_percentage = 0.0

                # send speed control only when ARUCO marker is visible and detected, stop and hover otherwise
                # IMPORTANT: axes are flipped in command send_rc_control(right/left, forward/backward, up/down)
                if marker_is_visible:
                    x_speed = 0
                    y_speed = 0
                    z_speed = 0
                    # self.tello_drone.send_rc_control(0, 0, 0, 0)

                    # distance from marker corresponds to flight axis X (forward/backward)
                    # real distance cannot be calculated yet - use marker to screen size proportion to ensure positioning plate with QR code will be visible
                    screen_dz = (self.camera.height/6) / marker_h
                    x_speed = -round(np.clip(-max_speed, x_pid(screen_dz), max_speed))

                    # camera image's X-axis (right/left) corresponds to flight axis Y (right/left)
                    screen_dx = -calculate_proportional_displacement(marker_x, marker_w, self.camera.width//2)
                    y_speed = round(np.clip(-max_speed, y_pid(screen_dx), max_speed))

                    # camera image's Y-axis (down/up) corresponds to flight axis Z (down/up)
                    screen_dy = calculate_proportional_displacement(marker_y, marker_h, self.camera.height//2)
                    z_speed = round(np.clip(-max_speed, z_pid(screen_dy), max_speed))

                    # rotate to align with marker (to be perpendicular to field of view)
                    yaw_da = angle
                    yaw_speed = round(np.clip(-max_speed, yaw_pid(yaw_da), max_speed))

                    with self.thread_lock:
                        self.speed_commands.x_percentage = x_speed / max_speed
                        self.speed_commands.y_percentage = y_speed / max_speed
                        self.speed_commands.z_percentage = z_speed / max_speed

                    i = i + 1
                    if i % 500 == 0:
                        print("x_speed", x_speed, "y_speed", y_speed, "z_speed", z_speed, "yaw_speed", yaw_speed)
                        print(screen_dz, screen_dx, screen_dy, angle)
                        pass 
                    if self.flight_mode: 
                        self.tello_drone.send_rc_control(y_speed, x_speed, z_speed, yaw_speed)
                else:
                    self.tello_drone.send_rc_control(0, 0, 0, 0)
 
                # assume drone is centered on marker when all parameters are within error margins
                if marker_is_visible and abs(screen_dz-1) < 0.5 and abs(screen_dx) < 0.5 and abs(screen_dy) < 0.5 and abs(angle) < 5:
                    self.tello_drone.send_rc_control(0, 0, 0, 0)
                    self.state.set(DroneState.MARKER_DISCOVERY_DECODING)
                    break


        # wait for camera thread to find and decode QR code
        for i in range(0, 15):
            time.sleep(1)
            if self.state.get() != DroneState.MARKER_DISCOVERY_DECODING:
                break
        if self.state.get() == DroneState.MARKER_DISCOVERY_DECODING:
            print("QR code not found - timeout...")
            return


        if not self.flight_settings.are_valid:
            print("flight settings not found!")
            return

        # ARUCO marker size is now known - use it to calculate real position of marker in field of view (in cm)
        def align_with_positioning_marker():
            if self.state.get() != DroneState.MARKER_ALIGNING_SEARCH:
                return False

            self.positioning_marker_data.is_valid = False
            
            # rotate dron if for some reason ARUCO marker is not visible for positioning
            # wait for camera thread to detect marker
            for i in range (0, 6):
                time.sleep(1)
                if self.state.get() == DroneState.MARKER_ALIGNING_SEARCH:
                    if self.flight_mode:
                        if i == 0:
                            self.tello_drone.rotate_counter_clockwise(45)
                        else:
                            self.tello_drone.rotate_clockwise(15)
                else:
                    break
            if self.state.get() == DroneState.MARKER_ALIGNING_SEARCH:
                return False

            ## create set of PID controlers for centering (individual for each axis)
            # X-axis is directed forward
            x_pid = PID(0.6, 0.002 , 0.06, setpoint=self.flight_settings.marker_distance)
            x_pid.sample_time = 0.1
            # Y-axis is directed to the right
            y_pid = PID(5, 0.08, 0.0, setpoint=0)
            y_pid.sample_time = 0.1
            # Z-axis is directed downward
            z_pid = PID(2.5, 0.4, 0.05, setpoint=0) 
            z_pid.sample_time = 0.1
            # yaw angle is in range (-180, 180)
            yaw_pid = PID(0.5, 0.04, -0.02, setpoint=0)
            yaw_pid.sample_time = 0.1

            max_speed = abs(MAX_SPEED_CLIP) # sanitize value for safety

            while self.state.get() == DroneState.MARKER_ALIGNING_POSITIONING:
                size = self.flight_settings.marker_size # known real size of marker (in cm)

                # be sure data updated by camera handling thread is not invalid
                with self.thread_lock:
                    marker_is_visible = self.current_marker_data.is_visible
                    marker_x = self.current_marker_data.x
                    marker_y = self.current_marker_data.y
                    marker_w = self.current_marker_data.w
                    marker_h = self.current_marker_data.h
                    angle = self.current_marker_data.angle
                    self.speed_commands.x_percentage = 0.0
                    self.speed_commands.y_percentage = 0.0
                    self.speed_commands.z_percentage = 0.0

                # send speed control only when ARUCO marker is visible and detected, stop and hover otherwise
                # IMPORTANT: axes are flipped in command send_rc_control(right/left, forward/backward, up/down)
                if marker_is_visible:
                    x_speed = 0
                    y_speed = 0
                    z_speed = 0

                    # distance from marker corresponds to flight axis X (forward/backward)
                    # marker size is now known - calculate real distance
                    z = (size / marker_w) * self.camera.width * (self.camera.calib_m * self.camera.focal) / self.camera.sensor_size
                    print("distance: ", d)
                    x_speed = round(np.clip(-max_speed, x_pid(z), max_speed))

                    # camera image's X-axis (right/left) corresponds to flight axis Y (right/left)
                    x = (size / marker_w) * (marker_x - self.camera.width//2)
                    y_speed = round(np.clip(-max_speed, y_pid(x), max_speed))

                    # camera image's Y-axis (down/up) corresponds to flight axis Z (down/up)
                    y = (size / marker_h) * (marker_y - self.camera.height//2)
                    z_speed = round(np.clip(-max_speed, z_pid(y), max_speed))

                    # rotate to align with marker (to be perpendicular to field of view)
                    yaw_da = angle
                    yaw_speed = round(np.clip(-max_speed, yaw_pid(yaw_da), max_speed))

                    with self.thread_lock:
                        self.speed_commands.x_percentage = x_speed / max_speed
                        self.speed_commands.y_percentage = y_speed / max_speed
                        self.speed_commands.z_percentage = z_speed / max_speed
                    
                    if self.flight_mode: 
                        self.tello_drone.send_rc_control(y_speed, x_speed, z_speed, yaw_speed)
                else:
                    self.tello_drone.send_rc_control(0, 0, 0, 0)

                # assume drone is centered on marker when all parameters are within error margins
                if marker_is_visible and abs(z-self.flight_settings.marker_distance) <= 5 and abs(dx) < 0.25*size and abs(dy) < 0.25*size and abs(angle) < 5:
                    self.tello_drone.send_rc_control(0, 0, 0, 0)
                    self.positioning_marker_data.height = self.tello_drone.get_height()
                    self.positioning_marker_data.yaw = self.tello_drone.get_yaw()
                    self.positioning_marker_data.is_valid = True
                    self.state.set(DroneState.MARKER_ALIGNING_DONE)
                    break          

            return True


        
        # main procedure for performing circular trajectory and capture photos
        if self.flight_mode:
            self.tello_drone.send_rc_control(0, 0, 0, 0)

            try:
                # prepare parameters for flight in circle
                r = self.flight_settings.radius
                n = ORBIT_SEGMENTS
                angle_step = 360/n
                height = self.tello_drone.get_height()

                # function to calculate edge point of polygon inscribed in orbit circle
                def calc_point(r, angle_degrees):
                    angle_radians = math.radians(angle_degrees)
                    x = r - r*math.cos(angle_radians)
                    y =     r*math.sin(angle_radians)

                    return round(x), round(y)
                x, y = calc_point(r, angle_step)
                        
                # repeat orbit for each height requested in QR code
                for h in self.flight_settings.heights:
                    # use positioning marker to set drone in start position of trajectory
                    self.state.set(DroneState.MARKER_ALIGNING_SEARCH)
                    if align_with_positioning_marker() == False:
                        print("failed to align with positioning marker!")
                        print("aborting flight...")

                    self.state.set(DroneState.ORBITING)

                    # change height to requested one in this iteration
                    height = self.tello_drone.get_height()
                    if h > height:
                        self.tello_drone.up(h)
                    elif h < height:
                        self.tello_drone.down(h)               

                    # perform series of rotate-move sequences and capture photos
                    for i in range(0, n):
                        yaw = self.tello_drone.get_yaw() - self.positioning_marker_data.yaw # compensate yaw difference from takeoff orientation

                        # calculate required yaw value at each stop point and rotate drone accordingly
                        target_yaw = i*angle_step
                        if target_yaw > 180:
                            target_yaw = target_yaw - 360
                        delta_yaw = int(round(target_yaw - yaw))
                        # print(yaw, i*angle_step, target_yaw, delta_yaw)
                        if delta_yaw < -180:
                            delta_yaw = delta_yaw + 360
                        if delta_yaw < 0:
                            self.tello_drone.rotate_counter_clockwise(abs(delta_yaw))
                        elif delta_yaw > 0:
                            self.tello_drone.rotate_clockwise(delta_yaw) 

                        # take snapshot
                        time.sleep(2) # allow camera feed from drone to catch up (there are always some delays)
                        angle = target_yaw
                        if angle < 0:
                            angle = angle + 180
                        cv2.imwrite("h_%d_angle_%d.jpeg" % (h, angle), self.tello_drone.get_frame_read().frame)
                        time.sleep(1)

                        # move drone slowly to next point on circle
                        self.tello_drone.go_xyz_speed(x, y, 0, 10)
                        time.sleep(1)
                        self.tello_drone.send_rc_control(0, 0, 0, 0)
                        time.sleep(1)
                    
                    
                    self.state.set(DroneState.ORBITING_COMPLETE) 
                    
                    # return with drone to height of positioning marker for next alignment
                    height = self.tello_drone.get_height()
                    if self.positioning_marker_data.height > height:
                        self.tello_drone.up(self.positioning_marker_data.height)
                    elif self.positioning_marker_data.height < height:
                        self.tello_drone.down(self.positioning_marker_data.height) 
            except Exception as e:
                print("exception occured during orbiting: ", e)
        else:
            print("Non-flying mode - no orbiting...")


        self.state.set(DroneState.TRAJECTORY_COMPLETE)
             
        if self.flight_mode:
            self.state.set(DroneState.LANDING)
            print("flight complted - landing...")
            self.tello_drone.send_rc_control(0, 0, 0, 0)
            self.tello_drone.land()
        self.flight_done = True

    
    def video_processing_function(self):
        print("video processing starting...")

        while self.tello_drone.get_frame_read() is None or self.tello_drone.get_frame_read().frame is None:
            print("frame not ready yet...")
        self.camera.is_ready = True

        self.camera.height, self.camera.width, _ = self.tello_drone.get_frame_read().frame.shape
        print("video stream is initialized...")
        self.video_stream_ready = True

        # this function uses camera's calibration matrix to estimate angle between marker's normal and camera's z-axis
        def estimate_marker_camera_angle(marker_screen_points):
            # corner points of marker as visible in camera's image
            pts2d = np.array(marker_screen_points, dtype=np.float32)

            # construct bounding box around marker and convert it's 2D points to 3D points with z=0
            # such bounding box is always aligned with camera axes and is perpendicular to camera's z-axis
            x, y, w, h = cv2.boundingRect(marker_screen_points)
            size = max(w, h)
            pts3d = [[x, y, 0],[x+size, y, 0], [x+size, y+size, 0], [x, y+size, 0]]
            pts3d = np.array(pts3d, dtype=np.float32)

            # find transformation between perpendicular bounding box (3D) and detected marker's corners (interpreted as 2D projection)
            # obtained rotation vector will contain angle between bounding box and marker's normal - which is identical to camera-marker angle
            dist_coeffs = np.zeros((4, 1))
            success, rotation_vector, translation_vector = cv2.solvePnP(pts3d, pts2d, self.camera.calib_matrix, dist_coeffs, flags=0)
            
            if success:
                # parse orientation vector to get angle (in degrees) around correct axis
                R = cv2.Rodrigues(rotation_vector)[0]
                angle = math.degrees(math.asin(R[2][0]))
                return (True, angle)
            
            return (False, None)


        # keep processing video stream until requested otherwise by main program
        while self.keep_recording:
            # capture image frame from video stream for analyzing
            img = self.tello_drone.get_frame_read().frame
            img = cv2.resize(img, (self.camera.width, self.camera.height))

            ### graphical information overlayed on displayed image
            # show battery information
            battery_status_color = (0, 255, 0)
            battery = self.tello_drone.get_battery()
            if battery < 60:
                battery_status_color = (0, 150, 255)
            if battery < 30:
                battery_status_color = (0, 0, 255)
            cv2.putText(img, 'battery: %d%%' % battery, (5, 25), cv2.FONT_HERSHEY_COMPLEX, 1, battery_status_color, 2)

            # show roll-horizon alignment
            if self.flight_mode:
                start_point = (0, self.camera.height//2)
                end_point = (self.camera.width, self.camera.height//2)
                cv2.line(img, start_point, end_point, (0, 0, 0), 2)

                h = (self.camera.width/2) * math.tan(math.radians(self.tello_drone.get_roll()))
                start_point = (0, int((self.camera.height / 2) - h))
                end_point = (self.camera.width, int((self.camera.height / 2) + h))
                cv2.line(img, start_point, end_point, (0, 0, 255), 2)
            else:
                pass
            ###

            # QR code / ARUCO marker detection
            if (self.state.get() == DroneState.MARKER_DISCOVERY_SEARCH or
                self.state.get() == DroneState.MARKER_DISCOVERY_FOUND or
                self.state.get() == DroneState.MARKER_DISCOVERY_CENTERING or
                self.state.get() == DroneState.MARKER_ALIGNING_SEARCH or
                self.state.get() == DroneState.MARKER_ALIGNING_FOUND or
                self.state.get() == DroneState.MARKER_ALIGNING_POSITIONING):        

                with self.thread_lock:
                    self.current_marker_data.is_visible = False
                    self.current_marker_data.x = 0
                    self.current_marker_data.y = 0
                    self.current_marker_data.w = 0
                    self.current_marker_data.h = 0
                    self.current_marker_data.angle = 0

                gray_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                (corners, ids, rejected) = cv2.aruco.detectMarkers(gray_image, cv2.aruco.Dictionary_get(ARUCO_MARKER_DICT_TYPE), parameters=cv2.aruco.DetectorParameters_create())
                if (ids is not None) and len(ids) > 0 and ids[0] == ARUCO_MARKER_ID:
                    # found corners of ARUCO code - convert them to better to handle version
                    pts = np.array(corners[0], np.int32)
                    pts.reshape([-1, 1, 2])

                    # draw outline of detected marker
                    cv2.polylines(img, [pts], True, (0, 0, 255), 3) 

                    # outline rectangle bounding box around marker for easier handling cases where drone looks at marker from an angle
                    x, y, w, h = cv2.boundingRect(pts)

                    # shift marker coordinates from its top left corner to the center
                    x = x + w // 2
                    y = y + h // 2
                    # print(x, y, w, h)

                    # find angle between camera and marker's normal (in degrees)
                    success, angle = estimate_marker_camera_angle(pts)
                    if not success:
                        angle = 0
                    else:
                        # print("angle", angle)
                        pass

                    with self.thread_lock:
                        self.current_marker_data.is_visible = True
                        self.current_marker_data.x = x
                        self.current_marker_data.y = y
                        self.current_marker_data.w = w
                        self.current_marker_data.h = h
                        self.current_marker_data.angle = angle

                    # draw line from marker to center of camera's image
                    image_center_x = self.camera.width // 2
                    image_center_y = self.camera.height // 2
                    cv2.line(img, (x, y), (image_center_x, image_center_y), (255, 0, 255), 2)

                    if self.state.get() == DroneState.MARKER_DISCOVERY_SEARCH:
                        self.state.set(DroneState.MARKER_DISCOVERY_FOUND)
                pass
            elif self.state.get() == DroneState.MARKER_DISCOVERY_DECODING:
                gray_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                qr_codes = decode(gray_image)
                for code in qr_codes:
                    points = code.polygon
                    x, y, w, h = code.rect

                    # found corners of QR code - convert them to better to handle version
                    pts = np.array(points, np.int32)
                    pts.reshape([-1, 1, 2])

                    # draw outline of detected code
                    cv2.polylines(img, [pts], True, (255, 0, 0), 3)
                    
                    # decode data from QR code
                    code_data = code.data.decode('utf-8')
                    code_dict = json.loads(code_data)
                    if "preamble" in code_dict:
                        if code_dict["preamble"] == QR_CODE_PREAMBLE:
                            print("QR code decoded")
                            print(json.dumps(code_dict, indent = 4))
                            self.flight_settings.marker_size = code_dict["data"]["marker_size"]
                            self.flight_settings.marker_distance = code_dict["data"]["marker_distance"]
                            self.flight_settings.radius = code_dict["data"]["radius"]
                            self.flight_settings.heights = code_dict["data"]["heights"]
                            self.flight_settings.are_valid = True
                            self.state.set(DroneState.MARKER_DISCOVERY_DONE)

                    # display content of QR code as an overlay on image
                    json_object = json.dumps(code_dict, indent = 4) 
                    text = json_object
                    y0, dy = code.rect[1], 25
                    for i, line in enumerate(text.split('\n')):
                        y = y0 + i*dy
                        cv2.putText(img, line, (code.rect[0], y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            
            # display code/marker discovery status icon (circle)
            # 'red' during search and 'green' when being detected or used
            if self.state.get() == DroneState.MARKER_DISCOVERY_SEARCH or self.state.get() == DroneState.MARKER_ALIGNING_SEARCH:
                cv2.circle(img, (self.camera.width - 50, 50), 25, (0, 0, 255), -1)
            elif (self.state.get() == DroneState.MARKER_DISCOVERY_FOUND or 
                  self.state.get() == DroneState.MARKER_DISCOVERY_CENTERING or 
                  self.state.get() == DroneState.MARKER_DISCOVERY_DECODING or 
                  self.state.get() == DroneState.MARKER_ALIGNING_FOUND or 
                  self.state.get() == DroneState.MARKER_ALIGNING_POSITIONING):
                cv2.circle(img, (self.camera.width - 50, 50), 25, (0, 255, 0), -1)
            else:
                pass

            # display arrows visualizing currently applied speeds
            if self.state.get() == DroneState.MARKER_DISCOVERY_CENTERING or self.state.get() == DroneState.MARKER_ALIGNING_POSITIONING:
                with self.thread_lock:
                    x_speed = self.speed_commands.x_percentage
                    y_speed = self.speed_commands.y_percentage
                    z_speed = self.speed_commands.z_percentage
                
                if y_speed != 0:
                    cv2.arrowedLine(img, (self.camera.width//2, self.camera.height//2), (round(self.camera.width*(1 + y_speed))//2, self.camera.height//2), (255, 255, 255), 4)
                if z_speed != 0:
                    cv2.arrowedLine(img, (self.camera.width//2, self.camera.height//2), (self.camera.width//2, round(self.camera.height*(1 - z_speed))//2), (255, 255, 255), 4)

            # display downscaled camera video feed for better performance
            cv2.imshow("Tello", cv2.resize(img, (self.camera.width // 2, self.camera.height // 2), interpolation = cv2.INTER_AREA))
            cv2.waitKey(1)
        cv2.destroyAllWindows()

        print("video processing complete...")

    def camera_stream(self):
        print("streaming start...")
        self.tello_drone.streamon()

        self.keep_recording = True
        video_processing = Thread(target=self.video_processing_function)
        video_processing.start()

        while self.recording_active:
            self.keep_recording = True
            time.sleep(1)
        self.keep_recording = False

        video_processing.join()
        time.sleep(1)

        self.tello_drone.streamoff()
        print("streaming done...")

    def graph(self):
        t = []
        roll_data = []
        pitch_data = []
        yaw_data = []
        plt.plot(t, roll_data, label="roll")
        plt.plot(t, pitch_data, label="pitch")
        plt.plot(t, yaw_data, label="yaw")
        plt.legend(loc="upper left")
        plt.ylim(-180, 180)
        cnt = 0
        t_cnt = 0

        while True:
            t.append(t_cnt)
            roll_data.append(self.accelX)
            pitch_data.append(self.accelY)
            yaw_data.append(self.accelZ)

            plt.gca().lines[0].set_xdata(t)
            plt.gca().lines[0].set_ydata(roll_data)
            plt.gca().lines[1].set_xdata(t)
            plt.gca().lines[1].set_ydata(pitch_data)
            plt.gca().lines[2].set_xdata(t)
            plt.gca().lines[2].set_ydata(yaw_data)
            plt.gca().relim()
            plt.gca().autoscale_view()
            plt.pause(0.1)

            cnt += 1
            t_cnt += 1

            if cnt > 2000:
                # cnt = 0
                t.pop(0)
                roll_data.pop(0)
                pitch_data.pop(0)
                yaw_data.pop(0)


    # main class variables and initialization
    tello_drone = None
    stop_controller = None
    flight_done = False

    file = None
    csv_writer = None

    def __init__(self):
        # allows only printing warning and errors (otherwise library polutes console with output of each dron command being sent)
        Tello.LOGGER.setLevel(logging.WARNING)

        # camera parameters
        self.camera = DotMap()
        self.camera.ready = False
        self.camera.width = 0
        self.camera.height = 0
        self.camera.sensor_size = 0
        self.camera.focal = 0
        self.camera.calib_matrix = None
        self.camera.calib_m = 0

        print("reading camera calibration file...")
        calib_data = np.load(os.path.join("cam_calibration", "calibration_tello.npz"))
        if calib_data == None:
            print("calibration data file not found!")
            exit(0)
        else:
            mtx = calib_data['mtx']
            print("calibration matrix:\n", mtx)
            self.camera.sensor_size = 2592 # based on datasheet - 5MP (2592x1936) 
            self.camera.focal = 25 # based on datasheet - 25mm focal length
            self.camera.calib_matrix = mtx
            self.camera.calib_m = (mtx[0][0] + mtx[1][1]) * 0.5

        self.tello_drone = tello.Tello()
        self.state = DroneStateVariable()
                  
        self.flight_mode = not SIMULATION_MODE
        self.kill_switch = self.TelloKillSwitch(self)
        self.kill_switch.start()

        self.thread_lock = Lock()

        #setup main processing threads
        self.stop_controller = Event()
        self.sensor_timer = TelloController.TelloTimer(0.1, self.stop_controller, self.get_sensor_data)
        self.battery_monitor_timer = TelloController.TelloTimer(10, self.stop_controller, self.monitor_battery)
        self.graph_timer = TelloController.TelloTimer(0.25, self.stop_controller, self.graph)

        try:
            # setup CSV file where sensor data will be logged
            self.file = open('flight_data.csv', 'w')
            self.csv_writer = csv.writer(self.file)
            csv_header = ['accel_x', 'accel_y', 'accel_y', 'speed_x', 'speed_y', 'speed_z', 'roll', 'pitch', 'yaw', 'time', 'battery']
            self.csv_writer.writerow(csv_header)

            ### common shared data structures between threads
            # currently traced marker
            self.current_marker_data = DotMap()
            self.current_marker_data.is_visible = False
            self.current_marker_data.x = 0
            self.current_marker_data.y = 0
            self.current_marker_data.w = 0
            self.current_marker_data.h = 0
            self.current_marker_data.angle = 0
            # speed commnds being currently applied
            self.speed_commands = DotMap()
            self.speed_commands.x_percentage = 0
            self.speed_commands.y_percentage = 0
            self.speed_commands.z_percentage = 0
            # setting and reading location of positioning marker
            self.positioning_marker_data = DotMap()
            self.positioning_marker_data.is_valid = False
            self.positioning_marker_data.height = 0
            self.positioning_marker_data.yaw = 0
            # parameters stored in QR code
            self.flight_settings = DotMap()
            self.flight_settings.are_valid = False
            self.flight_settings.marker_size = 0
            self.flight_settings.marker_distance = 0
            self.flight_settings.radius = 0
            self.flight_settings.heights = []


            # connect to Tello drone over direct Wi-Fi connection
            self.tello_drone.connect()
            self.state.set(DroneState.CONNECTED)
            time.sleep(1) # give time to stabilize all IMU readings

            # ensure battery level is sufficient for running program
            print("battery: %d%%" % self.tello_drone.get_battery())
            if self.tello_drone.get_battery() < 15 and self.flight_mode:
                print("battery below 15% - too low to fly!")
                return
            if self.tello_drone.get_battery() < 5:
                print("battery below 5% - too low to interact!")
                return


            # start necessary threads
            self.sensor_timer.start()
            self.battery_monitor_timer.start()
            # self.graph_timer.start()

            # start thread handling stream from Tello's camera
            self.recording_active = True
            self.video_stream_ready = False
            stream = Thread(target=self.camera_stream)
            stream.start()
            while not self.video_stream_ready:
                time.sleep(1)                  

            self.state.set(DroneState.SETUP_READY)

            # run all flight procedures
            self.flight_function()
            self.flight_done = True

            # finish handling video stream
            self.recording_active = False
            stream.join()

            self.tello_drone.end()
            self.stop_controller.set()
            self.state.set(DroneState.DONE)

            self.file.flush()
            self.file.close()
        except Exception as e:
            print("exception occured: ", e)
            if self.state != DroneState.UNINITIALIZED:
                print("landing after exception...")
                self.tello_drone.send_rc_control(0, 0, 0, 0)
                self.tello_drone.land()
                self.tello_drone.end()

            self.state.set(DroneState.FAILURE)
            self.flight_done = True
            self.recording_active = False
            self.stop_controller.set()

            return


if __name__ == '__main__':
    tc = TelloController()

    print("application done")
    exit()