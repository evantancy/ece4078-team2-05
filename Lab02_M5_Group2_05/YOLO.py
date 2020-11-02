import cv2
import numpy as np
import time
import sys
from typing import List
from CvTimer import CvTimer


class YOLO:
    CONFIDENCE_THRESHOLD = 0.35
    NMS_THRESHOLD = 0.2
    # hot pink baby, white, green
    # BGR! not RGB
    COLORS = [(255, 51, 255), (255, 255, 255), (0, 255, 0)]
    SIZE = {
        "small": (320, 320),
        "medium": (416, 416),
        "large": (608, 608),
        "custom_1": (512, 512),
    }
    FONT = cv2.FONT_HERSHEY_SIMPLEX
    FONT_SIZE = 0.5
    # thiccness needs to be an integer!!
    FONT_THICCNESS = 2
    # number of pixels
    TEXT_OFFSET = 18

    # TODO: Change to absolute path
    def __init__(
        self,
        gpu: int = 0,
        weights_path: str = "yolo_cfg/custom-yolov4-tiny-detector_best.weights",
        cfg_path: str = "yolo_cfg/custom-yolov4-tiny-detector.cfg",
        classes_path: str = "yolo_cfg/obj.names",
    ):
        """
        Args:
            gpu (int, optional): Defaults to 0.
            weights_path (str, optional): Defaults to "yolo_cfg/custom-yolov4-tiny-detector_final.weights".
            cfg_path (str, optional): Defaults to "yolo_cfg/custom-yolov4-tiny-detector.cfg".
            classes_path (str, optional): Defaults to "yolo_cfg/obj.names".

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
        self.seen_objects = dict()

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
        self, robot_pose=np.zeros(3), seen_objects=[], external_timer: CvTimer = None
    ) -> None:

        Timer = external_timer
        if external_timer is None:
            Timer = self._TIMER
        Timer.start("draw")

        # uncomment for colors that change every iteration
        # self.COLORS = np.random.uniform(0, 255, size=(len(self._classes), 3))

        FRAME_HEIGHT, FRAME_WIDTH, _ = self._img.shape
        # FOV_HORZ = 1.0855
        # FOV_VERT = 0.8517
        SCALE_HEIGHT = 0
        SCALE_WIDTH = 0
        MAX_DISTANCE = 3
        DUPLICATE_THRESHOLD = 1.5
        CAPTURE_WINDOW = 50

        # START DRAWING BOXES
        for (class_id, confidence, box) in zip(
            self._classes_detected, self._confidences, self._boxes
        ):
            # mod operator to loop through all possible colors we specify if we have lots of classes
            color = self.COLORS[int(class_id) % len(self.COLORS)]
            # class_id is an np.array of 1 element
            label_name = self._classes[class_id[0]]
            label = "%s : %.2f" % (label_name, confidence)

            # box has the format (x,y,width,height)
            x, y, width, height = box
            centre_x = int(x + width / 2)
            centre_y = int(y + height / 2)

            cv2.rectangle(self._img, box, color, self.FONT_THICCNESS)
            cv2.circle(self._img, (centre_x, centre_y), 1, color, 2)

            # -1 to draw ABOVE the box
            self.write_text(label, x, y, color, 14, position=-1)

            # END OF DRAWING BOXES
            # START PROCESS DISTANCES

            if label_name == "sheep":
                # 122 = 61 height @ 2m
                # 146.5 = 293 height @ 0.5m
                SCALE_HEIGHT = 125.5
                SCALE_WIDTH = 135.5
            elif label_name == "coke":
                # 92 = 92 height @ 1m
                SCALE_HEIGHT = 93.5
                SCALE_WIDTH = 37.5

            focal_length_w = width / SCALE_WIDTH
            focal_length_h = height / SCALE_HEIGHT
            distance_w = SCALE_WIDTH / width
            distance_h = SCALE_HEIGHT / height
            dist = (distance_w + distance_h) / 2

            # World frame
            obj_x = dist * np.cos(robot_pose[2]) + robot_pose[0]
            obj_y = dist * np.sin(robot_pose[2]) + robot_pose[1]
            save_pose = False
            # check if box centre is within of centre of frame
            if abs((x + width / 2) - FRAME_WIDTH / 2) < CAPTURE_WINDOW:
                # check if within threshold
                if dist < MAX_DISTANCE:
                    save_pose = True
                    if seen_objects is not None:
                        for obj in seen_objects:
                            obj_dist = np.sqrt(
                                (obj[1] - obj_x) ** 2 + (obj[2] - obj_y) ** 2
                            )
                            # check for duplicate entries
                            if obj_dist < DUPLICATE_THRESHOLD:
                                save_pose = False

            if save_pose:
                seen_objects.append([label_name, obj_x[0], obj_y[0]])
                print(f"Detected {label_name} @x:{obj_x[0]:.2f} y:{obj_y[0]:.2f}")
                # print(seen_objects)

            x_label = f"Xw: {np.around(obj_x, 2)}"
            y_label = f"Yw: {np.around(obj_y, 2)}"
            w_label = f"W: {width}"
            h_label = f"H: {height}"
            dist_label = f"D(m): {dist:.2f}"
            self.write_text(x_label, x, y + height, color, 14, position=1)
            self.write_text(y_label, x, y + height, color, 14, position=2)
            self.write_text(w_label, x, y + height + 30, color, 14, position=1)
            self.write_text(h_label, x, y + height + 30, color, 14, position=2)
            self.write_text(dist_label, x, y, color, 14, position=-2)

        # END PROCESS DISTANCES
        # PRINT DIAGNOSTICS ON OPENCV FRAME
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
        self.write_text(fps_label, 0, 25, self.COLORS[0], position=1)

        # cv2.imshow("YOLO", self._img)
        Timer.stop("YOLO")

        key = cv2.waitKey(1)
        if key == 27 or key == ord("q"):
            # del self
            cv2.destroyAllWindows()
            self._img.release()

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
            self.FONT_THICCNESS,
        )

    # def __del__(self):
    #     # Some weird funky stuff going on with manualSLAM.py
    #     cv2.destroyAllWindows("YOLO")
