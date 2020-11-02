# M5: Integrated system - Description

# Setting up the environment
- Run `sudo apt install python3-tk` to prevent matplotlib from stealing screen focus
- Extract our zip file
- Make sure to update penguinpi.sdf in `catkin_ws/src/penguinpi_description/urdf/penguinpi.sdf` with our sdf file in the ***parent directory of Lab02_M5_Group2_05*** (some changes to the friction values)
## Running on GPU
In order to enable OpenCV's DNN module for YOLOv4 to work properly using GPU.
- Please keep `opencv-python`, uninstall `opencv-contrib-python`, and run our installation & build scripts for...
  - OpenCV -> `install_opencv.sh`
  - CUDA -> `install_cuda_toolkit.sh`
  - cuDNN -> `install_cudnn.sh`
- Ensure that `gpu=1` in the YOLO constructor in `manualSLAM.py`
## Running on CPU
- Keep `opencv-python` and `opencv-contrib-python`, and set `gpu=0` in the YOLO constructor in `manualSLAM.py`

# Scripts/Functions
- manualSLAM.py: main script that is called to implement the integrated system and navigate the robot around the arena to detect objects and aruco markers. Brings together many other scripts and functions such as keyboardcontrolarteststarter.py, YOLO.py, and SLAM functions.
- SLAM folder: contains all the SLAM and position estimation functions necessary to estimate poses of the aruco markers and in turn the robot itself. aruco_detector.py has been modified to only record aruco markers detected that are a certain distance away, based on the magnitude of tvecs. This substantially improves the accuracy of our system.
keyboardControlARtestStarter.py: script used for teleoperation of the robot, takes in left/right/up/down commands from the keyboard and translates into turning and straight movements of the robot, with compensation to prevent veering/drifting
- YOLO.py: A YOLOv4 class that uses OpenCV's DNN module to load pre-trained weights and draw bounding boxes for different objects detected. It also returns a list of position (x,y) estimates of each detected object. It captures objects only when they are within a specific capture window at the centre of the frame, and a threshold is in place to prevent recording duplicate objects.
- CvTimer.py: A timer class that uses OpenCV's getTickFrequency to profile different parts of our code, storing different processes' runtime in ms and rate in Hz
- yolo_cfg folder: contains different `.cfg` and corresponding `_best.weights` files, as well as class labels for different objects being detected i.e. sheep and coke in `obj.names`. In our demo, `custom-yolov4-tiny-detector.cfg` and `custom-yolov4-tiny-detector_best.weights` were used.
- PenguinPiC.py: Modified from original to include failed gazebo command checks, which improves overall performance of the robot
Calibration folder: contains our calibration values such as distCoeffs.txt, intrinsic.txt, baseline.txt, and scale.txt

# Run the robot
- Run roslaunch on either map (the demo map)
- After setting up the environment, open a new terminal and run **`python3 manualSLAM.py`** in the folder `Lab02_M5_Group2_05/`
- Use the left, right, up and down arrows to drive and turn. You only need to press the keyboard button once for it to drive in the specific direction. Press the space button to stop. Do not use WASD keys! These are used for changing forward/turning speeds.
- The yolo script will constantly be outputting its bounding boxes to the image output, however it will only record the position of an object if it’s estimated distance from the robot is below a certain threshold (3m), and the object is centred with respect to the camera. This ensures accurate estimates of the objects, and thus in order to pick up an object you must drive close enough to it and scan the object in a horizontal direction so as to center it.
- Regular 360 degree turns throughout the robots navigation ensures that the robot will be constantly reevaluating its possession with newly seen markers and previously detected markers, thus regular scans are necessary to ensure optimal accuracy.


# Ideal robot path: demo_arena_dev_no_collision.world
1. Make sure robot is in position 0 (x,y,yaw)
2. Start by rotating 360 degree anticlockwise
3. Rotate the robot to face Marker5 (1.374260, 9.588080) and go straight towards it and stop when y~4 position
4. Rotate 360 degree clockwise again, then go towards marker 5 until y~7.5
5. Turn towards Marker29 (-5.372030, 1.050860) and go towards it, until around y~2.5
6. Turn to the left facing Marker13 (-1.346820, -2.386830) and go straight, until it detects the sheep
7. Turn to the left facing Marker11 (4.001650, -1.552220) and go straight until it detects the coke can
8. Rotate 360 degree anticlockwise.
9. Turn to the left facing Marker23 (3.379770, 6.462060) and go straight until y~5
10. Do a final 360 degree anticlockwise turn.

# Result
The robot will detect 5 sheep and 5 coke cans, 10 or 11 aruco markers (possibly 1 ‘phantom’ aruco marker that doesn’t exist, but due to the limited camera capabilities it may mistake it) all within 1m range of the true position. This is all output to the file: Lab02_M5_Map_Group2_05.csv

# Outputs
Lab02_M5_Map_Group2_05.csv: resulting aruco marker and object pose estimates, this script is updated constantly throughout the arena navigation, and will stop updating once manualSLAM.py is shutdown.
