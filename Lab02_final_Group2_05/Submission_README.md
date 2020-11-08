# Final Demo: Integrated system - Description

# Setting up the environment
- Run `sudo apt install python3-tk` to prevent matplotlib from stealing screen focus
- Extract our zip file
- Make sure to update penguinpi.sdf in `catkin_ws/src/penguinpi_description/urdf/penguinpi.sdf` with our sdf file in the ***parent directory of Lab02_final_Group2_05*** (some changes to the friction values)
## Running on GPU
In order to enable OpenCV's DNN module for YOLOv4 to work properly using GPU.
- Please keep `opencv-python`, uninstall `opencv-contrib-python`, and run our installation & build scripts found [here](https://github.com/evan-tan/ece4078-team2-05/tree/master/setup) for...
  - OpenCV -> `install_opencv.sh`
  - CUDA -> `install_cuda_toolkit.sh`
  - cuDNN -> `install_cudnn.sh`
- Ensure that `gpu=1` in the YOLO constructor in `manualSLAM.py`
## Running on CPU
- Keep `opencv-python` and `opencv-contrib-python`, and set `gpu=0` in the YOLO constructor in `manualSLAM.py`

# Scripts/Functions
- manualSLAM.py: main script that is called to implement the integrated system and navigate the robot around the arena to detect objects and aruco markers. Brings together many other scripts and functions such as keyboardcontrolarteststarter.py, YOLO.py, and SLAM functions.
- SLAM folder: contains all the SLAM and position estimation functions necessary to estimate poses of the aruco markers and in turn the robot itself. aruco_detector.py has been modified to only record aruco markers detected that are a certain distance away, based on the magnitude of tvecs. This substantially improves the accuracy of our system.
- YOLO.py: A YOLOv4 class that uses OpenCV's DNN module to load pre-trained weights and draw bounding boxes for different objects detected. It also returns a list of position (x,y) estimates of each detected object. It captures objects only when they are within a specific capture window at the centre of the frame, and a threshold is in place to prevent recording duplicate objects.
- yolo_cfg folder: contains different `.cfg` and corresponding `_best.weights` files, as well as class labels for different objects being detected i.e. sheep and coke in `obj.names`. In our demo, `custom-yolov4-tiny-detector.cfg` and `custom-yolov4-tiny-detector_best.weights` were used.
- CvTimer.py: A timer class that uses OpenCV's getTickFrequency to profile different parts of our code, storing different processes' runtime in ms and rate in Hz
- keyboardControlARtestStarter.py: script used for teleoperation of the robot, takes in left/right/up/down commands from the keyboard and translates into turning and straight movements of the robot, with compensation to prevent veering/drifting. We've also added a member variable that turns YOLO on and off, to toggle this on/off use <kbd>y</kbd>

- PenguinPiC.py: Modified from original to include failed gazebo command checks, which improves overall performance of the robot
Calibration folder: contains our calibration values such as distCoeffs.txt, intrinsic.txt, baseline.txt, and scale.txt
- utils.py: Contains a few common functions like `load_yaml` and `load_calib_params` used across different files such as `aruco_detector.py`, `YOLO.py` and `manualSLAM.py`
- config.yml: A YAML file containing all the different parameters used across different files mainly `aruco_detector.py` and `YOLO.py`

# Run the robot
- Run roslaunch on either map (the demo map)
- After setting up the environment, open a new terminal and run **`python3 manualSLAM.py`** in the folder `Lab02_final_Group2_05/`
- By default YOLO is deactivated, press <kbd>y</kbd> to turn it on.
- Use the left, right, up and down arrows to drive and turn. You only need to press the keyboard button once for it to drive in the specific direction. Press the space button to stop. Do not use WASD keys! These are used for changing forward/turning speeds.
- The yolo script will constantly be outputting its bounding boxes to the image output, however it will only record the position of an object if it’s estimated distance from the robot is below a certain threshold (max_detect_distance in config.yml), and the object is centred with respect to the camera. This ensures accurate estimates of the objects, and thus in order to pick up an object you must drive close enough to it and scan the object in a horizontal direction so as to center it.
- Regular 360 degree turns throughout the robots navigation ensures that the robot will be constantly reevaluating its possession with newly seen markers and previously detected markers, thus regular scans are necessary to ensure optimal accuracy.


# Result
The robot will detect 5 sheep and 5 coke cans, 10 or 11 aruco markers (possibly 1 ‘phantom’ aruco marker that doesn’t exist, but due to the limited camera capabilities it may mistake it) all within 1m range of the true position. This is all output to the file: Lab02_final_Map_Group2_05.csv

# Outputs
Lab02_final_Map_Group2_05.csv: resulting aruco marker and object pose estimates, this script is updated constantly throughout the arena navigation, and will stop updating once manualSLAM.py is shutdown.
