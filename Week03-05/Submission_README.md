# How to run our robot

- Make sure to update penguinpi.sdf in catkin_ws/src/penguinpi_description/urdf/penguinpi.sdf
	(some changes to the friction value)
- After setting up the environment, open a new terminal and run python3 manualSLAM.py
- Press arrow keys to control the robot 

Highlights for running the robot in another environment:
	1. If there are any marker on sight, start scanning by rotating couple of times before moving on to the next marker.
	2. Make sure to align the predicted arrow with the actual direction the robot is facing (by pressing right and left arrows multiple times until it matches) before moving forward or backward to scan a marker.

Recommended path for full mark in penguinpi_arena (all within 1.5m range):
	- Wheel_calibration values: 3.6s and 27.7 seconds
	1. From start position, scan left and right around three times to get an accurate position of the first 2 markers. 
	2. After scanning, facing slightly to the right of the sheep (so we don't collide) go towards the thirds marker and the robot will scan the third marker.
	3. Steer around 90 degree to the left, towards the 4th marker. Rotate if needed to scan the marker. 
	4. Steer around 90 degree to the left, towards the 5th marker. Rotate if needed to scan the marker. 
	5. Scan left and right to make sure the direction of the arrow is still aligned with the robot. Then go to the last marker and scan.


