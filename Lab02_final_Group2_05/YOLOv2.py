import cv2
import numpy as np
import time
import os
from typing import List
from CvTimer import CvTimer
from utils import load_yaml, load_calib_params
from slam.Measurements import MarkerMeasurement

yolo_params = load_yaml("config.yml")["YOLO"]
calib_folder = os.getcwd() + "/calibration/"


class YOLO:
    CONFIDENCE_THRESHOLD = yolo_params["THRESHOLD"]["confidence"]
    NMS_THRESHOLD = yolo_params["THRESHOLD"]["NMS"]
    COLORS = yolo_params["DRAWING"]["colors"]
    SIZE = yolo_params["NETWORK_SIZE"]
    FONT = cv2.FONT_HERSHEY_SIMPLEX
    FONT_SIZE = yolo_params["DRAWING"]["font_size"]
    FONT_THICCNESS = yolo_params["DRAWING"]["font_thiccness"]
    TEXT_OFFSET = yolo_params["DRAWING"]["text_offset"]
    CAMERA_MAT, DIST_COEFFS, _, _ = load_calib_params(calib_folder)
    FOV_HORZ = yolo_params["FOV"]["horizontal"]
    FOV_VERT = yolo_params["FOV"]["vertical"]
    MAX_DISTANCE = yolo_params["THRESHOLD"]["max_detect_distance"]
    MAX_MATCH_ERROR = yolo_params["THRESHOLD"]["max_match_error"]
    DUPE_THRESH = yolo_params["THRESHOLD"]["dupe_abs_dist"]
    DUPE_X_THRESH = yolo_params["THRESHOLD"]["dupe_x"]
    DUPE_Y_THRESH = yolo_params["THRESHOLD"]["dupe_y"]
    CAPTURE_WINDOW = yolo_params["THRESHOLD"]["capture_window"]

    def __init__(
        self,
        gpu: int = 0,
        weights_path: str = yolo_params["PATH"]["weights"],
        cfg_path: str = yolo_params["PATH"]["cfg"],
        classes_path: str = yolo_params["PATH"]["classes"],
    ):
        """
        Args:
            gpu (int, optional): Use CUDA backend via GPU or not. Defaults to 0.
            weights_path (str, optional): Load weights file. Defaults to params["PATH"]["weights"].
            cfg_path (str, optional): Load cfg file. Defaults to params["PATH"]["cfg"].
            classes_path (str, optional): Load classes/labels/objects. Defaults to params["PATH"]["classes"].
        """
        # load labels/classes from obj.names file
        self._classes = []
        with open(classes_path, "r") as f:
            self._classes = [line.strip() for line in f.readlines()]

        # self.COLORS = np.random.uniform(0, 255, size=(len(self._classes), 3))

        # load YOLO neural network
        net = cv2.dnn.readNet(weights_path, cfg_path)
        if gpu:
            net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
            net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
        else:
            net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

        self._model = cv2.dnn_DetectionModel(net)
        self._model.setInputParams(size=self.SIZE["medium"], scale=1 / 255)

        self._TIMER = CvTimer()
        self.object_counts = dict()
        for label in self._classes:
            self.object_counts[label] = 0

    def run_inference(
        self, cv2_frame: np.ndarray, external_timer: CvTimer = None
    ) -> None:
        # detecting objects
        Timer = external_timer
        if external_timer is None:
            Timer = self._TIMER
        Timer.start("YOLO")
        Timer.start("inference")

        self._img = cv2_frame
        self._classes_detected, self._confidences, self._boxes = self._model.detect(
            self._img, self.CONFIDENCE_THRESHOLD, self.NMS_THRESHOLD
        )

        Timer.stop("inference")

    def process(
        self,
        robot_pose=np.zeros(3),
        last_seen_objects=[],
        external_timer: CvTimer = None,
    ) -> None:
        Timer = external_timer
        if external_timer is None:
            Timer = self._TIMER
        Timer.start("draw")

        if len(robot_pose) != 3:
            robot_pose = robot_pose[0:3]

        # uncomment for colors that change every iteration
        # self.COLORS = np.random.uniform(0, 255, size=(len(self._classes), 3))

        FRAME_HEIGHT, FRAME_WIDTH, _ = self._img.shape  # pixels
        # WIDTH or HEIGHT based should NOT matter, but values vary by 1 ish
        FOCAL_LENGTH_W = (FRAME_WIDTH / 2) / np.tan(self.FOV_HORZ / 2)  # pixels
        FOCAL_LENGTH_H = (FRAME_HEIGHT / 2) / np.tan(self.FOV_VERT / 2)  # pixels

        # Gazebo dimensions in metres
        COKE = {"x": 0.06, "y": 0.06, "z": 0.14}
        SHEEP = {"x": 0.108, "y": 0.223, "z": 0.204}
        SCALE_HEIGHT = None
        SCALE_WIDTH = None

        seen_objects = []
        # START DRAWING BOXES
        for (class_id, confidence, box) in zip(
            self._classes_detected, self._confidences, self._boxes
        ):
            # mod operator to loop through all possible colors we specify if we have lots of classes
            color = self.COLORS[int(class_id) % len(self.COLORS)]
            # class_id is an np.array of 1 element
            label_name = self._classes[class_id[0]]
            label = "%s: %.2f" % (label_name, confidence)

            x, y, width, height = box
            centre_x = int(x + width / 2)
            centre_y = int(y + height / 2)

            # Create artificial square aruco marker from object's box corners, taking height as "marker length"
            # Is this able to resolve tilt angle of "marker?"
            # TODO: Determine sheep pose += 0.1 discrepancy
            # box_corners = [
            #     [centre_x - height / 2, centre_y - height / 2],
            #     [centre_x + height / 2, centre_y - height / 2],
            #     [centre_x + height / 2, centre_y + height / 2],
            #     [centre_x - height / 2, centre_y + height / 2],
            # ]
            box_corners = [
                [x, y],
                [x + width, y],
                [x + width, y + height],
                [x, y + height],
            ]
            box_corners = [np.asarray([box_corners], dtype=np.float32)]

            cv2.rectangle(self._img, box, color, self.FONT_THICCNESS)
            cv2.circle(self._img, (centre_x, centre_y), 1, color, 2)
            # -1 to draw ABOVE the box
            self.write_text(label, x, y, color, 14, position=-1)

            annot_dist = None
            focal_dist = None
            marker_length = 0.1
            if label_name == "sheep":
                # bounding box dimensions @ 1m
                SCALE_HEIGHT = 125
                # SCALE_WIDTH = 135.5
                # DO NOT USE WIDTH FOR SHEEP (ASYMMETRICAL!)
                marker_length *= 93 / 61
                # expected_width = SHEEP["y"]
                # expected_height = SHEEP["z"]
                annot_dist = SCALE_HEIGHT / height
                # some_term = b ** 2 - 4 * a * (c - np.log(height))
                # if some_term > 0:
                #     annot_dist = (-b - np.sqrt(some_term)) / (2 * a)
                # else:
                #     annot_dist = 10
                # focal_dist = (expected_height * FOCAL_LENGTH_H) / height

            elif label_name == "coke":
                # bounding box dimensions @ 1m
                SCALE_HEIGHT = 93.5
                SCALE_WIDTH = 37.5
                marker_length *= 124 / 61
                # expected_width = COKE["y"]
                # expected_height = COKE["z"]
                # # units:(m), distance based on bounding boxes
                distance_w = SCALE_WIDTH / width
                distance_h = SCALE_HEIGHT / height
                annot_dist = (distance_w + distance_h) / 2

                # units:(m), distance based on focal lengths
                # f_dist_w = (expected_width * FOCAL_LENGTH_W) / width
                # f_dist_h = (expected_height * FOCAL_LENGTH_H) / height
                # focal_dist = (f_dist_w + f_dist_h) / 2
            marker_length = 0.1
            # Extract pose from tvecs matrix
            _, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                box_corners, marker_length, self.CAMERA_MAT, self.DIST_COEFFS
            )
            if tvecs is None:
                continue

            tvecs = tvecs.T
            obj_pose_2d = np.block([[tvecs[2, :]], [-tvecs[0, :]]])
            obj_pose_2d = np.mean(obj_pose_2d, axis=1).reshape(-1, 1)
            obj_x = obj_pose_2d[0] * np.cos(robot_pose[2]) + robot_pose[0]
            obj_y = obj_pose_2d[1] * np.sin(robot_pose[2]) + robot_pose[1]

            dist_to_object = np.sqrt(obj_pose_2d[0] ** 2 + obj_pose_2d[1] ** 2)
            if dist_to_object < self.MAX_DISTANCE:
                if len(seen_objects) == 0:
                    # Initial seen_objects
                    self.object_counts[label_name] = 1
                    n = self.object_counts[label_name]
                    save_label = label_name + str(n)
                    obj_measurement = MarkerMeasurement(obj_pose_2d, save_label)
                    seen_objects.append(obj_measurement)

                # Update seen objects: determine if add new entry or update existing
                for i in range(len(seen_objects)):
                    pos = seen_objects[i].position
                    delta_obj_x = abs(pos[0] - obj_x)
                    delta_obj_y = abs(pos[1] - obj_y)
                    delta_obj_dist = np.sqrt(delta_obj_x ** 2 + delta_obj_y ** 2)
                    # If within threshold, update existing single object
                    if delta_obj_dist > self.DUPE_THRESH:
                        print("New found")
                        self.object_counts[label_name] += 1
                        n = self.object_counts[label_name]
                        save_label = label_name + str(n)
                        obj_measurement = MarkerMeasurement(obj_pose_2d, save_label)
                        seen_objects.append(obj_measurement)

            # # Calculate discrepancy between 2 methods
            # # match_error = abs((annot_dist - focal_dist) / annot_dist)

            # # Removes dependence on capturing objects in centre of frame
            # centre_offset = centre_x - FRAME_WIDTH / 2
            # focal_length = (FOCAL_LENGTH_W + FOCAL_LENGTH_H) / 2
            # angle_obj_to_robot = np.arctan2(centre_offset, focal_length)

            # # World frame
            # # obj_x = (
            # #     annot_dist * np.cos(robot_pose[2] - angle_obj_to_robot) + robot_pose[0]
            # # )
            # # obj_y = (
            # #     annot_dist * np.sin(robot_pose[2] - angle_obj_to_robot) + robot_pose[1]
            # # )
            # obj_x = annot_dist * np.cos(robot_pose[2]) + robot_pose[0]
            # obj_y = annot_dist * np.sin(robot_pose[2]) + robot_pose[1]
            # # print(f'{label_name}: {match_error:.2f}')

            # save_pose = True
            # # check if box centre is within capture window
            # if abs((x + width / 2) - FRAME_WIDTH / 2) > self.CAPTURE_WINDOW:
            #     save_pose = False
            # # check if within threshold
            # if annot_dist > self.MAX_DISTANCE:
            #     save_pose = False
            # check error between annotation vs focal length methods
            # if match_error > MAX_MATCH_ERROR:
            #     print(f"YOLO: match error exceeded")
            #     save_pose = False

            # if save_pose:
            #     pass
            #     # seen_objects.append([label_name, obj_x[0], obj_y[0]])
            #     # print(
            #     #     f"YOLO: Detected {label_name} @x:{obj_x[0]:.2f} y:{obj_y[0]:.2f} D:{annot_dist:.2f}"
            #     # )

            x_label = f"Xw: {np.around(obj_x, 2)}"
            y_label = f"Yw: {np.around(obj_y, 2)}"
            w_label = f"W: {width}"
            h_label = f"H: {height}"
            dist_label = f"D: {annot_dist:.2f}"
            # print(f"{label_name}: {annot_dist:.2f}")
            self.write_text(x_label, x, y + height, color, 14, position=1)
            self.write_text(y_label, x, y + height, color, 14, position=2)
            self.write_text(w_label, x, y + height + 30, color, 14, position=1)
            self.write_text(h_label, x, y + height + 30, color, 14, position=2)
            self.write_text(dist_label, x, y, color, 14, position=-2)

        inf_time = Timer.get_diagnostics("inference")[0]
        inf_label = f"INF(ms): {inf_time:.2f}ms"
        # self.write_text(inf_label, 0, 25, self.COLORS[0], position=1)

        Timer.stop("draw")

        draw_time = Timer.get_diagnostics("draw")[0]
        draw_label = f"DRW(ms): {draw_time:3.2f}"
        # self.write_text(draw_label, 0, 25, self.COLORS[0], position=2)

        process_time = draw_time + inf_time
        fps = 1000 / process_time
        fps_label = f"YOLO: {process_time:3.2f}ms @{fps:3.2f}Hz"
        # self.write_text(fps_label, 0, 25, self.COLORS[0], position=1)

        # cv2.imshow("YOLO", self._img)
        Timer.stop("YOLO")

        key = cv2.waitKey(1)
        if key == 27 or key == ord("q"):
            cv2.destroyAllWindows()
            # del self
            # self._img.release()

        return seen_objects

    def write_text(
        self,
        label: str = "",
        x: int = 0,
        y: int = 0,
        color: tuple = (255, 255, 255),
        offset: int = None,
        position: int = 1,
        img=None,
    ) -> None:
        """
        Args:
            label (str, optional): Text to write on frame. Defaults to "".
            x (int, optional): Location in x (horizontal). Defaults to 0.
            y (int, optional): Location in y (vertical). Defaults to 0.
            color (tuple, optional): Defaults to (255, 255, 255).
            offset (int, optional): Defaults to None.
            position (int, optional): Offset multiplier. Think of it as a positional argument for text on the OpenCV window. Defaults to 1. Setting it to -1 writes the text above bounding boxes.
        """
        if offset is None:
            offset = self.TEXT_OFFSET
        if img is None:
            img = self._img
        cv2.putText(
            img,
            label,
            (x, y + offset * position),
            self.FONT,
            self.FONT_SIZE,
            color,
            thickness=self.FONT_THICCNESS,
        )

    # def __del__(self):
    #     # Some weird funky stuff going on with manualSLAM.py
    #     cv2.destroyAllWindows("YOLO")
