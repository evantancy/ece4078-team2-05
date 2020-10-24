import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import os, sys
import json
import time
import cv2


# Import keyboard teleoperation components
from PenguinPiC import PenguinPi
import keyboardControlARtestStarter as Keyboard
from YOLO import YOLO
from CvTimer import CvTimer

# Import SLAM components
sys.path.insert(0, "{}/slam".format(os.getcwd()))
import slam.Slam as Slam
import slam.Robot as Robot
import slam.aruco_detector as aruco
import slam.Measurements as Measurements

map_f = "estimated_poses.csv"
true_map_f = "TruePose_demo_arena_dev_no_collision.csv"
true_map_no_col_f = "TruePose_demo_arena_dev.csv"
Timer = CvTimer()

# Manual SLAM
class Operate:
    # TODO: Feed SLAM as constructor param
    def __init__(self, datadir: str, ppi: PenguinPi, yolo_obj: YOLO):
        self.yolo = yolo_obj

        self.ppi = ppi
        self.ppi.set_velocity(0, 0)

        self.img = np.zeros([240, 320, 3], dtype=np.uint8)
        self.aruco_img = np.zeros([240, 320, 3], dtype=np.uint8)

        self.keyboard = Keyboard.Keyboard(self.ppi)

        # Get camera / wheel calibration info for SLAM
        camera_matrix, distortion_coeffs, scale, baseline = self.getCalibParams(datadir)

        # SLAM components
        self.pibot = Robot.Robot(
            baseline, scale * 0.5, camera_matrix, distortion_coeffs
        )  # manually adjusted baseline value to be more accurate

        self.aruco_det = aruco.aruco_detector(self.pibot, marker_length=0.1)
        self.slam = Slam.Slam(self.pibot)

    def getCalibParams(self, datadir) -> list:
        # Imports camera / wheel calibration parameters
        fileK = "{}camera_calibration/intrinsic.txt".format(datadir)
        camera_matrix = np.loadtxt(fileK, delimiter=",")
        fileD = "{}camera_calibration/distCoeffs.txt".format(datadir)
        distortion_coeffs = np.loadtxt(fileD, delimiter=",")
        fileS = "{}wheel_calibration/scale.txt".format(datadir)
        scale = np.loadtxt(fileS, delimiter=",")
        fileB = "{}wheel_calibration/baseline.txt".format(datadir)
        baseline = np.loadtxt(fileB, delimiter=",")

        return camera_matrix, distortion_coeffs, scale, baseline

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
        lms, _ = self.aruco_det.detect_marker_positions(self.img)
        self.slam.add_landmarks(lms)
        self.slam.update(lms)

    def display(self, fig, ax):
        # Visualize SLAM
        ax[0].cla()
        self.slam.draw_slam_state(ax[0])
        plt.pause(0.001)
        ax[1].cla()
        ax[1].imshow(self.img[:, :, -1::-1])

    def write_map(self, slam):
        self.marker_list = []
        for i in range(len(slam.taglist)):
            self.marker_list.append(
                [slam.taglist[i], slam.markers[0][i], slam.markers[1][i]]
            )
        self.marker_list = sorted(self.marker_list, key=lambda x: x[0])
        self.seen_objects = sorted(self.seen_objects, key=lambda x: x[0])
        with open(map_f, "w") as f:
            f.write("object, x, y\n")
            for markers in self.marker_list:
                f.write(
                    "Marker"
                    + str(markers[0])
                    + ", "
                    + str(markers[1])
                    + ", "
                    + str(markers[2])
                )
                f.write("\n")
            for markers in self.seen_objects:
                f.write(
                    str(markers[0]) + ", " + str(markers[1]) + ", " + str(markers[2])
                )
                f.write("\n")

    def process(self):
        # Show SLAM and camera feed side by side
        fig, ax = plt.subplots(1, 2)
        # ax[1].imshow(self.img)
        self.dt1 = time.time()
        # objects picked up by YOLO
        self.seen_objects = []
        while True:
            Timer.start("SLAM")
            self.control()
            self.vision()
            Timer.stop("SLAM")

            Timer.start("YOLO")
            # pass image into yolo ONCE!!
            self.yolo.run_inference(self.img)
            self.seen_objects = self.yolo.process(
                self.slam.get_state_vector(), self.seen_objects
            )
            Timer.stop("YOLO")

            Timer.start("write_map")
            # Save SLAM map
            self.write_map(self.slam)
            Timer.stop("write_map")

            slam_time, slam_rate = Timer.get_diagnostics("SLAM")
            slam_label = f"SLAM: {slam_time:3.2f}ms @ {slam_rate:3.2f}Hz"
            self.yolo.write_text(
                slam_label, 0, 25, self.yolo.COLORS[0], position=2, img=self.img
            )

            Timer.start("viz")
            # Output visualisation
            self.display(fig, ax)
            Timer.stop("viz")
            Timer.print_all()

    # def __del__(self):
    # self.ppi.set_velocity(0, 0)


if __name__ == "__main__":
    # Location of the calibration files
    currentDir = os.getcwd()
    datadir = "{}/calibration/".format(currentDir)
    ppi = PenguinPi()
    yolo = YOLO(gpu=1)

    # Perform Manual SLAM
    operate = Operate(datadir, ppi, yolo)
    operate.process()
