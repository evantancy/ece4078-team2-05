# How to run our robot

- Make sure to update penguinpi.sdf in catkin_ws/src/penguinpi_description/urdf/penguinpi.sdf with our sdf file (some changes to the friction value)
- Also attached are multiple folders and scripts contain slam functions used for M2, which were used for a pose estimations. These include Robot.py and Measurements.py.
- Our camera calibration values can be found in the folder camera_calibration, and our wheel parameters can be found in calibration/wheel_calibration. These are both referenced in the script from their respective folders, so no changes are necessary .
- After setting up the environment, open a new terminal and run python3 autonav_starter.py to generate the map!

Mapping algorithm:
- The robot first scans 360 degrees, detecting all markers in its FOV
- If there is only one marker in sight it will drive towards it. If there are two markers in sight it will drive towards the closest marker unless it has just come from that marker, in which case it will drive to the next closest. If there are 3 or more markers, the robot will prioritise driving to the marker that it has not seen before, otherwise it will drive to a marker it has seen before but has not already driven to, if none of these ocnditions are met it drives towards the closest marker.
- The above two steps are repeated until all markers in the arena have been detected, and then the map is genereted with a sufficient amount of paths found.

Results:
- As seen in the map.txt file generated, ou results are very accurate, simply using some slam functions for pose estimation. Please note that the EKF was left out for this milestone.
- Including the starting point, 19 paths were found, of which 2 are technically invalid, but due to our robot only being able to get so close to the aruco markers without crashing, there was no way around detecting these paths (paths 12,21 and 8,1).
- On the rare ocassion when at marker 3, marker 1 has been incorrectly identified as 37 or 42, so just keep this in mind if it is detected.
- Overall the time taken is approximatley 9.4 minutes, however as seen in the video, a conservative estimate for our FPS is around 10, thus our scaling factor for our time should be around 10/30 which would result in a final run time of approx 3.13 mins.
	
Please see our video (~2 mins) titled "M4_Demo.mp4" to see one of our best test runs, also note the video is running at 5x speed.


