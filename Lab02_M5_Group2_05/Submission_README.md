# How to run our robot

## Setting up the environment
- Run `sudo apt install python3-tk` to prevent matplotlib from stealing screen focus
- Extract our zip file
- Make sure to update penguinpi.sdf in `catkin_ws/src/penguinpi_description/urdf/penguinpi.sdf` with our sdf file in the ***parent directory of Lab02_M5_Group2_05*** (some changes to the friction values)
### Running on GPU
In order to enable OpenCV's DNN module for YOLOv4 to work properly using GPU.
- Please keep `opencv-python`, uninstall `opencv-contrib-python`, and run our installation & build scripts for...
  - OpenCV -> `install_opencv.sh`
  - CUDA -> `install_cuda_toolkit.sh`
  - cuDNN -> `install_cudnn.sh`
- Ensure that `gpu=1` in the YOLO constructor in `manualSLAM.py`
### Running on CPU
- Keep `opencv-python` and `opencv-contrib-python`, and set `gpu=0` in the YOLO constructor in `manualSLAM.py`

# Run the robot
- Run roslaunch on either map (the demo map)
- After setting up the environment, open a new terminal and run **`python3 manualSLAM.py`** in the folder `Lab02_M5_Group2_05/`
- Press arrow keys to control the robot: up = forwards, down = reverse, left = left, right = right. You only need to press the keyboard button once for it to drive in the specific direction. Press the space button to stop.
- Our WASD keys are used for changing speed functions, however we please ask that you do not use this, as our robot has only been calibrated to drive at a certain speed (60 ticks/s forward, and 28 ticks/s turning). So please do not use any keys except the arrow keys.
- Wheel_calibration values:
  - baseline: `1.142968276500685443e+00`
  - scale: `4.629629629629629373e-03`

These values should be in the respective txt files, but in case they get lost these are the values we had. Within the `manualSLAM.py` script we further scaled our baseline value by `1.0` and scale by `0.5`.

# Ideal robot path
1. Make sure robot is in position 0 (x,y,yaw)
2. Start by rotating 360 degree anticlockwise
3. Rotate the robot to face Marker5 (1.374260, 9.588080) and go straight towards it and stop when y~4 position
4. Rotate 360 degree anticlockwise again, then go towards marker 5 until y~7.5
5. Turn towards Marker29 (-5.372030, 1.050860) and go towards it, until around y~2.5
6. Turn to the left facing Marker13 (-1.346820, -2.386830) and go straight, until it detects the sheep
7. Turn to the left facing Marker11 (4.001650, -1.552220) and go straight until it detects the coke can
8. Rotate 360 degree anticlockwise.
9. Turn to the left facing Marker23 (3.379770, 6.462060) and go straight until y~5
10. Do a final 360 degree clockwise turn.

# Result
The robot will detect 5 sheeps, 5 or 6 cokes (possibly 1 false coke from aruco marker falsely detected as coke), 10 or 11 aruco markers (possibly 1 falsely detected aruco marker) all within 1m range of the true position. This will be constantly updated in the file estimated_poses.csv throughout the test run.
