# Manually drive the robot inside the arena and perform SLAM using ARUCO
# markers

# Import packages
import numpy as np
import matplotlib.pyplot as plt
import os, sys
import json
import time
# Import keyboard teleoperation components
import PenguinPiC
import keyboardControlARtestStarter as Keyboard
from YOLO import YOLO
# Import SLAM components
sys.path.insert(0, "{}/slam".format(os.getcwd()))
import slam.Slam as Slam
import slam.Robot as Robot
import slam.aruco_detector as aruco
import slam.Measurements as Measurements

# Manual SLAM
class Operate:
    def __init__(self, datadir, ppi):
        # Initialise
        self.ppi = ppi
        self.ppi.set_velocity(0, 0)
        self.img = np.zeros([240, 320, 3], dtype=np.uint8)
        self.aruco_img = np.zeros([240, 320, 3], dtype=np.uint8)

        # Keyboard teleoperation components
        self.keyboard = Keyboard.Keyboard(self.ppi)

        # Get camera / wheel calibration info for SLAM
        camera_matrix, dist_coeffs, scale, baseline = self.getCalibParams(datadir)

        # SLAM components
        self.pibot = Robot.Robot(baseline, scale*0.5, camera_matrix, dist_coeffs) # manually adjusted baseline value to be more accurate
        self.aruco_det = aruco.aruco_detector(self.pibot, marker_length=0.1)
        self.slam = Slam.Slam(self.pibot)

    #def __del__(self):
        #self.ppi.set_velocity(0, 0)

    def getCalibParams(self, datadir):
        # Imports camera / wheel calibration parameters
        fileK = "{}camera_calibration/intrinsic.txt".format(datadir)
        camera_matrix = np.loadtxt(fileK, delimiter=',')
        fileD = "{}camera_calibration/distCoeffs.txt".format(datadir)
        dist_coeffs = np.loadtxt(fileD, delimiter=',')
        fileS = "{}wheel_calibration/scale.txt".format(datadir)
        scale = np.loadtxt(fileS, delimiter=',')
        fileB = "{}wheel_calibration/baseline.txt".format(datadir)
        baseline = np.loadtxt(fileB, delimiter=',')

        return camera_matrix, dist_coeffs, scale, baseline

    def control(self):
        # Import teleoperation control signals
        [lv, rv, _] = self.keyboard.latest_drive_signal()
        self.dt2 = time.time()
        drive_meas = Measurements.DriveMeasurement(lv, rv, self.dt2-self.dt1)
        self.dt1 = time.time()
        self.slam.predict(drive_meas)

    def vision(self):
        # Import camera input and ARUCO marker info
        self.img = self.ppi.get_image()
        lms, aruco_image = self.aruco_det.detect_marker_positions(self.img)
        self.slam.add_landmarks(lms)
        self.slam.update(lms)

    def display(self, fig, ax):
        # Visualize SLAM
        ax[0].cla()
        self.slam.draw_slam_state(ax[0])

        ax[1].cla()
        ax[1].imshow(self.img[:, :, -1::-1])

        plt.pause(0.01)

    def write_map(self, slam):
        # Output SLAM map as a json file
        map_dict = {"AR_tag_list":slam.taglist,
                    "map":slam.markers.tolist(),
                    "covariance":slam.P[3:,3:].tolist()}
        with open("slam.txt", 'w') as map_f:
            json.dump(map_dict, map_f, indent=2)

    def process(self):
        # Show SLAM and camera feed side by side
        fig, ax = plt.subplots(1, 2)
        img_artist = ax[1].imshow(self.img)

        # Main loop
        self.dt1 = time.time()
        while True:
            # Run SLAM
            self.control()
            self.vision()
            yolo.run_inference(self.img)
            yolo.draw_boxes(self.img)

            # Save SLAM map
            self.write_map(self.slam)

            # Output visualisation
            self.display(fig, ax)


if __name__ == "__main__":
    # Location of the calibration files
    currentDir = os.getcwd()
    datadir = "{}/calibration/".format(currentDir)
    # connect to the robot
    ppi = PenguinPiC.PenguinPi()
    yolo = YOLO()
    # Perform Manual SLAM
    operate = Operate(datadir, ppi)
    operate.process()
