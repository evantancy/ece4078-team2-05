# How to run our robot

- Make sure to update penguinpi.sdf in catkin_ws/src/penguinpi_description/urdf/penguinpi.sdf with our sdf file
	(some changes to the friction value)
- After setting up the environment, open a new terminal and run python3 manualSLAM.py
- Press arrow keys to control the robot: up = forwards, down = reverse, left = left, right = right. Use spacebar to stop.
- Our WASD keys are used for changing speed functions, however we please ask that you do not use this, as our robot has only been calibrated to drive at a certain speed (60 ticks/s forward, and 28 ticks/s turning). So please do not use any keys except the arrow keys.
- Wheel_calibration values: 1.142968276500685443e+00 for baseline and 4.629629629629629373e-03 for scale. These values should be in the respective txt files, but in case they get lost these are the values we had. Within the manualSLAM.py script we multiplied our baseline value by 1.1.
- slam.txt- the slam.txt file for one of our best test runs has been included in this submission, in case the values are overided, here are our values:
  "AR_tag_list": [
    11,
    5,
    9,
    7,
    3,
    1
  ],
  "map": [
    [
      1.330861183035292,
      -0.8339833768024145,
      3.531267361932977,
      1.8600193409397807,
      -1.4289422179737608,
      -2.1661805408529156
    ],
    [
      1.75109678374075,
      1.5534509057424748,
      0.8315943161453128,
      4.977775503121363,
      4.39832366872863,
      -0.14604414040194363
    ]

Highlights for running the robot in another environment:
- If a marker has been detected, start scanning by rotating a couple of times left and right before moving on to the next marker.
- If you can, please do your best to align the predicted arrow with the actual direction the robot is facing (by pressing right and left arrows multiple times until it matches) before moving forward or backward to scan a marker. However our marker does not deviate too far from the true position, so this hopefully shouldnt be an issue.

Recommended path for full mark in penguinpi_arena (all within 1.5m range):
- From start position, scan left and right around 1-2 times to get an accurate position of markers 11 and 5. 
- After scanning, facing slightly to the right of the sheep (so we don't collide) go towards marker 9 and the robot will scan marker 9.
- Steer around 90 degrees to the left after passing the sheep, towards marker 7. Once almost perpendicular to marker 7, rotate to the right to scan it.
- Steer around 150 degrees to the left, towards marker 3, until it is succefully picked up. Then drive towards it until you are within a metre of it.
- rotate 360 degrees on the spot to pick up markers 7 and 9 again (if they are visible), until you face marker 3 again.
- Next, rotate to the left (away from marker 3) on a path towards marker 1, drive staright until marker 1 is visible, and adjust the path so that when possible you are travelling straight down (-pi/2 radians in world frame).
- Stop once you almost reach the beginning of the corridor, then turn left (from the robots perspective) to pick up markers 11 and 5 again, then stop.
- End
	
Please see our video (~2 mins) titled "PenguinPi_M2_Best_Run.mp4" to see one of our best test runs and the path we have described above, also note the video is running at 4x speed.


