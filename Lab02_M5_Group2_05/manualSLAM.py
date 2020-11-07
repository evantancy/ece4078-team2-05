import numpy as np
import matplotlib.pyplot as plt
import os
import sys
import time
from PenguinPiC import PenguinPi
from keyboardControlARtestStarter import Keyboard
from YOLO import YOLO
from CvTimer import CvTimer
from utils import load_calib_params, load_yaml
import cv2

sys.path.insert(0, "{}/slam".format(os.getcwd()))
from SLAMv2 import Extractor
from slam.Slam import Slam
from slam.Robot import Robot
from slam.aruco_detector import aruco_detector
import slam.Measurements as Measurements

# import matplotlib
# matplotlib.use("TkAgg")

map_f = "Lab02_M5_Map_Group2_05.csv"
calib_folder = os.getcwd() + "/calibration/"
params = load_yaml("config.yml")

# Manual SLAM
class Operate:
    def __init__(self):
        self.yolo = YOLO(gpu=1)
        self.ppi = PenguinPi()
        self.ppi.set_velocity(0, 0)
        self.keyboard = Keyboard(self.ppi)

        self.img = np.zeros([240, 320, 3], dtype=np.uint8)

        # Get camera / wheel calibration info for SLAM
        camera_matrix, distortion_coeffs, scale, baseline = load_calib_params(
            calib_folder
        )

        # SLAM components
        self.pibot = Robot(
            wheels_width=baseline,
            wheels_scale=scale * 0.5,
            camera_matrix=camera_matrix,
            camera_dist=distortion_coeffs,
        )  # manually adjusted baseline value to be more accurate

        self.aruco_det = aruco_detector(
            camera_matrix=camera_matrix,
            camera_dist=distortion_coeffs,
            marker_length=params["ARUCO"]["marker_length"],
        )

        self.slam = Slam(self.pibot)
        self.orb_slam = Extractor()
        self._TIMER = CvTimer()
        self.seen_objects = []
        self.scale_objects = []

    def display(self, fig, ax):
        # Visualize SLAM
        ax[0].cla()
        self.slam.draw_slam_state(ax[0])
        plt.pause(0.001)
        ax[1].cla()
        # flip BGR to RGB format
        ax[1].imshow(self.img[:, :, -1::-1])

    def write_map(self, slam):
        self.marker_list = []
        for i in range(len(slam.taglist)):
            self.marker_list.append(
                [slam.taglist[i], slam.markers[0][i], slam.markers[1][i]]
            )
        self.marker_list = sorted(self.marker_list, key=lambda x: x[0])
        self.seen_objects = sorted(self.seen_objects, key=lambda x: x[0])
        # self.scale_objects = sorted(self.scale_objects, key=lambda x: x[0])
        with open(map_f, "w") as f:
            f.write("object, x, y\n")
            for markers in self.marker_list:
                if type(markers[0]) == int:
                    markers[0] = "Marker" + str(markers[0])

                f.write(
                    str(markers[0]) + ", " + str(markers[1]) + ", " + str(markers[2])
                )
                f.write("\n")
            for i in range(len(self.seen_objects)):
                markers = self.seen_objects[i]
                f.write(
                    str(markers[0]) + ", " + str(markers[1]) + ", " + str(markers[2])
                )
                f.write("\n")
                # markers = self.scale_objects[2 * i]
                # f.write(
                #     str(markers[0]) + ", " + str(markers[1]) + ", " + str(markers[2])
                # )
                # f.write("\n")
                # markers = self.scale_objects[2 * i + 1]
                # f.write(
                #     str(markers[0]) + ", " + str(markers[1]) + ", " + str(markers[2])
                # )
                # f.write("\n")

    def process(self):
        # Show SLAM and camera feed side by side
        fig, ax = plt.subplots(1, 2)
        # ax[1].imshow(self.img)
        self.dt1 = time.perf_counter()

        while True:
            self._TIMER.start("control")
            [lv, rv, _] = self.keyboard.latest_drive_signal()
            self.dt2 = time.perf_counter()
            drive_meas = Measurements.DriveMeasurement(lv, rv, self.dt2 - self.dt1)
            self.dt1 = time.perf_counter()
            self.slam.predict(drive_meas)
            self._TIMER.stop("control")

            self._TIMER.start("vision")
            self.img = self.ppi.get_image()
            # img_copy = self.img.copy()

            if self.keyboard.run_yolo:
                self.yolo.run_inference(self.img)
                self.seen_objects = self.yolo.process(
                    self.slam.get_state_vector(), self.seen_objects
                )
            lms, _ = self.aruco_det.detect_marker_positions(self.img)

            self.slam.add_landmarks(lms)
            self.slam.update(lms)
            self._TIMER.stop("vision")

            # Save SLAM map
            self.write_map(self.slam)

            # orb_slam
            # self.orb_slam.process_frame(img_copy)
            # cv2.imshow("copy", img_copy)

            control_time, control_rate = self._TIMER.get_diagnostics("control")
            control_label = f"control: {control_time:3.2f}ms @ {control_rate:3.2f}Hz"
            # self.yolo.write_text(
            #     control_label, 0, 25, self.yolo.COLORS[0], position=2, img=self.img
            # )
            vision_time, vision_rate = self._TIMER.get_diagnostics("vision")
            vision_label = f"vision: {vision_time:3.2f}ms @ {vision_rate:3.2f}Hz"
            # self.yolo.write_text(
            #     vision_label, 0, 25, self.yolo.COLORS[0], position=3, img=self.img
            # )

            self._TIMER.start("plot")
            self.display(fig, ax)
            self._TIMER.stop("plot")

            plot_time, plot_rate = self._TIMER.get_diagnostics("plot")
            plot_label = f"plot: {plot_time:3.2f}ms @ {plot_rate:3.2f}Hz"
            # Not showing due to axes being cleared
            # self.yolo.write_text(
            #     plot_label, 0, 25, self.yolo.COLORS[0], position=4, img=self.img
            # )
            # self._TIMER.print_all()


if __name__ == "__main__":
    # Perform Manual SLAM
    operate = Operate()
    operate.process()
