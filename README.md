# Building 3D model based on series of photos from Tello drone 

## Topic description
This is final project for IiSRL laboratory. The aim of the project was to create Python-based scripts to control Ryze Tello drone (non-EDU version) in order to perform 3D scanning of an object. Main objectives were:
* drone takeoff
* finding and decrypting QR code with flight mission parameters
* perform orbiting around object with QR marker placed on it - with flight parameters (height, radius etc.) from QR code
* register series of photos from different angles along radius (camera centered on center of orbit circle)
* safe landing
* use collected photos to generate 3D model of scanned object with usage of Autodesk ReCap software

## Team
* H. Chlebowski
* P. Kraska
* M. Oleszczyk
* P. Zimnowłodzki

## Presentation
Detailed information regarding project and its main concepts can be found in PDF file [presentation](IiSRL_presentation__L2_G5.pdf).

## Completed objectives
All project objectives have been completed.

### Marker reading and positioning
* after takeoff dron correctly finds ARUCO marker and positions itself so marker is centered in its field of view (based on general marker location)
* drone flies closer to marker plate and decode complementary QR code with scanning parameters
* camera calibration files are used to calculate real distance from ARUCO marker and drone flies back to starting position of scanning trajectory (based on parameters from QR code)
<ul>
<img alt="scan_example" src="https://user-images.githubusercontent.com/123629764/214836669-b061ddda-4274-4ff0-a7f4-1786adef48a2.png" width="40%" height="40%">
</ul>

### Orbit flight and images capture
* drone orbits around scanned object using rotate-move pattern and traverses along edge of polygon inscribed in orbit circle (right amount of segments is choosen to avoid any potential collisions)
* drone captures photo of the scanned object at each stop point and images are stored on the PC running python script
* orbit trajectory is repeated on all required heights (information embeded in QR code) and each captured photo is properly named
<ul>
<img alt="scan_example" src="https://user-images.githubusercontent.com/123629764/214829099-dec2f7bc-af30-4500-9eca-136eb61aa894.gif" width="40%" height="40%">
</ul>

### 3D model creation
* collected photos are uploaded to Autodesk ReCap software which generates 3D model of scanned object based on collected images


## Ryze Tello information (from DJI)
Tello's specification: https://www.ryzerobotics.com/tello/specs

Tello's (non-EDU) documentation: https://dl-cdn.ryzerobotics.com/downloads/Tello/Tello%20User%20Manual%20v1.4.pdf

## List of used libraries
<b>All project scripts and packages require Python3</b>

Required libries (with used versions) are listed in <i>requirements.txt</i> file. Run following command to install all python packages on your machine.
```
python -m pip install -r requirements.txt
```

Links to main libraries used in project:

    https://pypi.org/project/djitellopy/
    https://pypi.org/project/simple-pid/
    https://pypi.org/project/pyzbar/
    https://pypi.org/project/matplotlib/
    https://pypi.org/project/cv/
    https://pypi.org/project/numpy/

## 3D scanning procedure - usage

### Connecting with drone
Majoriy of scripts require connection to Tello drone. If not noted otherwise, please be sure to always pair Tello drone with your PC over Wi-Fi network beforehand. For details please refer to official documentation.


### Setup
Firstly, before running main <i>tello.py</i> scipt, you must run series of helper scripts to generate all necessay files (only one time).

<ul>

#### Generating QR code
Script _qr_code_generator.py_ generates QR code for this project with special preamble and information based on script execution parameters.
New file _tello_qr_code.png_ will be created in current working directory.

<table>
    <tr>
        <img alt="code" src="https://user-images.githubusercontent.com/123629764/214854618-26012ad6-1c40-4565-b9a3-01808e06d4a4.png" width="50%" height="50%"> 
    </tr>
    <tr>   
         <img alt="tello_qr_code" src="https://user-images.githubusercontent.com/123629764/214854733-bfb1d695-6212-4618-9025-8136e420cd43.png" width="20%" height="20%"> 
    </tr>

#### Generating ARUCO marker
Script _aruco_marker_generator.py_ generates single ARUCO marker in DICT_4x4_50 mode and with id 10 (please see https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html for details). 
New file _tello_aruco_marker.png_ will be created in current working directory.

> **Warning**
> 
> When printing, make sure that marker size (expressed in cm) is exatly the same as one passed as argument during QR code generation!
    
<img alt="tello_qr_code" src="https://user-images.githubusercontent.com/123629764/214854750-6ec5ecad-213a-4020-9144-e6d7a689f91f.png" width="20%" height="20%"> 


#### Camera calibration
Navigate to <i>cam_calibration</i> subdirectory when executing any of following scripts.

<ul>
    
##### Capturing images
Run _cam_calib_capture.py_ script to capture series of photos of calibration image (checkerboard). 
Script connects to Tello drone and captures 10 images from drone's camera for calibration purposes (move them later to _images_ subfolder after validating - checkerboard must be fully visible).
It will capture single frame from drone's camera when spacebar is pressed - this allows to align camera between each snapshot.
For the best calibration, please rotate and move checkerboard within field of view of drone's camera for most varied images.
    
Please use 5x7 checkerboard pattern (see https://docs.opencv.org/4.x/da/d0d/tutorial_camera_calibration_pattern.html)

<img alt="checkerboard" src="https://user-images.githubusercontent.com/123629764/214841633-aea59893-f090-461d-9574-1cc7278ea8e9.png" width="30%" height="30%">
<img alt="code" src="https://user-images.githubusercontent.com/123629764/214847431-9c2021ae-68a7-4ec1-90c1-6eb38e100f64.png" width="30%" height="30%">   


##### Generating calibration file
Run _cam_calib.py_ script to use captured image for calibration purpose. It displays and parses pictures from _images_ subfolder and generates calibration file _calibration_tello.npz_, which is later required by main script _tello.py_ during startup.
    
<img alt="checkerboard" src="https://user-images.githubusercontent.com/123629764/214841736-1a619173-cdf9-4d88-9ee7-130167568240.png" width="30%" height="30%">
<img alt="image" src="https://user-images.githubusercontent.com/123629764/214845635-3f612264-ca25-40ed-a240-1bec3229def7.png" width="30%" height="30%">

</ul>
</ul>


### Fly and capture photos
When all necessary setup files are prepared, simply run main script <i>tello.py</i>.
Drone will automatically takeoff and perform following actions:
* locate ARUCO marker
* close in on ARUCO marker (roughly - based on marker proportions)
* find and decode QR code from positoning plate (next to ARUCO marker) - all positioning and fly parameters will be stored in the memory
* distance itself from ARUCO marker by specified distance (trajectory starting point) based on marker real size (all data contained in QR code)
* repeat following orbiting steps
    * read next fly height from settings (from QR code)
    * change height position
    * perform repeated sequence of rotate-move steps until completing full circle
    * capture photo at each stop point    
* return to starting point and safely land

> **Warning**
> 
> In case of emergency, special **kill switch** is present in script that forces drone to perform emergency landing, i.e. all motors are instantly turned off. Procedure is triggered by pressing **spacebar key** on keyboard (**window executing script must be in focus**) and immediately sends _emergency_ command to drone.
> Please be prepared to catch drone in such case to avoid crashing it.
    
    
### Generating 3D scan of the object
When drone completes all its tasks, you will find series of photos in executing directory. Name of each file contains height and angle at which photo was captured. 
    
To create 3D object from scan photos, please download and install Adobe ReCap software (please refere to official materials under following link https://knowledge.autodesk.com/support/recap). Next, create new project and upload your photos. Please make sure that **at least 20 photos** were selected for model creation.
    
After uploading photos in Adobe ReCap, application will send them to cloud where procedure to analyze and transform them will begin. It may take even up to half an hour.

<img alt="adobe_recap" src="https://user-images.githubusercontent.com/123629764/214859463-bb1619a3-548a-49ec-b92b-6dcbd1415a20.png" width="40%" height="40%">


## General problems
### Ryze Tello drone (non-EDU version) drift
Tello's position stabilization is based on Vision Positioning System , which helps drone to maintain its current location when not moving (please refer to <i>Vision Positioning System</i> section of documentation for details).
However, during work on the project, positioning system proved to be very prone to non-ideal evironment conditions and constant drift during hovering was observed. To mitigate this problem, special material with dense and distinctive pattern was placed in area of scanning.

<img alt="distinctive_pattern" src="https://user-images.githubusercontent.com/123629764/214823770-736a559b-6572-4fbe-9fb6-9e7ae4f46ec4.png" width="30%" height="30%">

### QR code visibility
QR codes are a great way to encode information into graphical representation that can be later decoded by camera-based system. However, 5MP camera available in Tello drone is not able to properly detect 8cm x 8cm code packed with necessary information from distance over 1.5m. To avoid need of placing big QR code on scanned object, ARUCO marker has been added to positioning plate as a way to improve plate detection (due to simple patterns).

<img alt="positioning_plate" src="https://user-images.githubusercontent.com/123629764/214823900-6d42799c-8a21-4042-b918-7ee64617e888.png" width="30%" height="30%">
    
### Generating 3D model from dron photos
One of the aims of the project was to capture series of photos of an object while flying with Tello drone around it. 
Due to the nature of the drone, all photos are taken from the same pitch angle. This however, seems to pose problem for Autodesk ReCap software when generating 3D model from such images. With constant pitch angle, generated model reproduces rather scaning environment than object in the middle. **This behaviour has been confirmed also during manual testing with external camera and even more dense number of photos.**
    
<img alt="positioning_plate" src="https://user-images.githubusercontent.com/123629764/215015671-297fd108-5571-4ae2-b05c-7cc057b9d6ac.png" width="50%" height="50%">


## Notes
For performance reasons, window with camera feed (and annotations) has been scaled down.
Please be aware that printing debug statements can also impact general performance and responsivenes of drone control.

> **Warning**
> 
> When executing emergency landing (spacebar key) please be sure to correctly terminate program afterward. Library <i>djitellopy</i> (in version 2.4.0) is prone to sporadically sending <i>takeoff</i> commands after such landing.

> **Note**
> 
> During stream of camera image to PC, following video-related warnings/errors may appear. These are safe to ignore.
>
> <img alt="positioning_plate" src="https://user-images.githubusercontent.com/123629764/214846948-a7bf6084-fafc-4fe7-9b52-ab062d15d954.png" width="30%" height="30%">
