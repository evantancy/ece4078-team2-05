# Manually drive the robot inside the arena and perform SLAM using ARUCO
# markers

# Import packages
import numpy as np
import matplotlib.pyplot as plt
import os, sys
import json
import time

# Import keyboard teleoperation components
from PenguinPiC import PenguinPi
import keyboardControlARtestStarter as Keyboard
#from YOLO import YOLO

# Import SLAM components
sys.path.insert(0, "{}/slam".format(os.getcwd()))
import slam.Slam as Slam
import slam.Robot as Robot
import slam.aruco_detector as aruco
import slam.Measurements as Measurements
map_f = 'estimated_poses.csv'
# Manual SLAM
class Operate:
    def __init__(self, datadir, ppi, yolo_obj):
        # Initialise
        #self.yolo = yolo_obj
        self.ppi = ppi
        self.ppi.set_velocity(0, 0)
        self.img = np.zeros([240, 320, 3], dtype=np.uint8)
        self.aruco_img = np.zeros([240, 320, 3], dtype=np.uint8)

        # Keyboard teleoperation components
        self.keyboard = Keyboard.Keyboard(self.ppi)

        # Get camera / wheel calibration info for SLAM
        camera_matrix, dist_coeffs, scale, baseline = self.getCalibParams(datadir)

        # SLAM components
        self.pibot = Robot.Robot(
            baseline, scale * 0.5, camera_matrix, dist_coeffs
        )  # manually adjusted baseline value to be more accurate
        self.aruco_det = aruco.aruco_detector(self.pibot, marker_length=0.1)
        self.slam = Slam.Slam(self.pibot)

    # def __del__(self):
    # self.ppi.set_velocity(0, 0)

    def getCalibParams(self, datadir):
        # Imports camera / wheel calibration parameters
        fileK = "{}camera_calibration/intrinsic.txt".format(datadir)
        camera_matrix = np.loadtxt(fileK, delimiter=",")
        fileD = "{}camera_calibration/distCoeffs.txt".format(datadir)
        dist_coeffs = np.loadtxt(fileD, delimiter=",")
        fileS = "{}wheel_calibration/scale.txt".format(datadir)
        scale = np.loadtxt(fileS, delimiter=",")
        fileB = "{}wheel_calibration/baseline.txt".format(datadir)
        baseline = np.loadtxt(fileB, delimiter=",")

        return camera_matrix, dist_coeffs, scale, baseline

    def control(self):
        # Import teleoperation control signals
        [lv, rv, _] = self.keyboard.latest_drive_signal()
        self.dt2 = time.time()
        drive_meas = Measurements.DriveMeasurement(lv, rv, self.dt2 - self.dt1)
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
        self.marker_list =[]
        for i in range(len(slam.taglist)):
            self.marker_list.append([slam.taglist[i], slam.markers[0][i], slam.markers[1][i]])
        self.marker_list = sorted(self.marker_list, key=lambda x: x[0])
        with open(map_f,'w') as f:
            f.write('object, x, y\n')
            for markers in self.marker_list:
                    f.write('Marker'+str(markers[0])+', '+str(markers[1])+', '+str(markers[2]))
                    f.write('\n')

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
            # pass image into yolo ONCE!!
            #self.yolo.run_inference(self.img)
            #self.yolo.process(self.slam.get_state_vector())

            # Save SLAM map
            self.write_map(self.slam)

            # Output visualisation
            self.display(fig, ax)


if __name__ == "__main__":
    # Location of the calibration files
    currentDir = os.getcwd()
    datadir = "{}/calibration/".format(currentDir)
    # connect to the robot
    ppi = PenguinPi()
    #yolo = YOLO(gpu=1)
    yolo = []
    # Perform Manual SLAM
    operate = Operate(datadir, ppi, yolo)
    operate.process()



