import cv2
import numpy as np
import time
import sys
from typing import List


class YOLO:
    CONFIDENCE_THRESHOLD = 0.4
    SCORE_THRESHOLD = 0.2
    NMS_THRESHOLD = 0.4
    # hot pink baby, white, green, red
    # BGR! not RGB
    COLORS = [(255, 51, 255), (255, 255, 255), (0, 255, 0), (0, 0, 255)]
    SIZE = {"small": (320, 320), "medium": (416, 416), "large": (608, 608)}
    FONT = cv2.FONT_HERSHEY_SIMPLEX
    FONT_SIZE = 0.5
    # thiccness needs to be an integer!!
    FONT_THICCNESS = 2
    # number of pixels
    TEXT_OFFSET = 18

    def __init__(
        self,
        weights_path: str = "yolo_cfg/custom-yolov4-tiny-detector_final.weights",
        cfg_path: str = "yolo_cfg/custom-yolov4-tiny-detector.cfg",
        classes_path: str = "yolo_cfg/obj.names",
    ) -> None:
        """
        Args:
            weights_path (str, optional): Defaults to "yolo_cfg/custom-yolov4-tiny-detector_final.weights".
            cfg_path (str, optional): Defaults to "yolo_cfg/custom-yolov4-tiny-detector.cfg".
            classes_path (str, optional): Defaults to "yolo_cfg/obj.names".

        """
        # load labels/classes from obj.names file
        self._classes = []
        with open(classes_path, "r") as f:
            self._classes = [line.strip() for line in f.readlines()]

        # load YOLO neural network
        net = cv2.dnn.readNet(weights_path, cfg_path)
        net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
        net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
        # net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA_FP16)
        self._model = cv2.dnn_DetectionModel(net)
        self._model.setInputParams(size=self.SIZE["medium"], scale=1 / 255)

        self.__inference_time = 0
        self.__draw_time = 0

    def run_inference(self, cv2_frame) -> None:
        # detecting objects
        start_inf = time.time()

        self._img = cv2_frame
        self.classes_detected, self.confidences, self.boxes = self._model.detect(
            self._img, self.CONFIDENCE_THRESHOLD, self.NMS_THRESHOLD
        )

        self.__inference_time = (time.time() - start_inf) * 1000

    def process(self, robot_pose=np.zeros(3)) -> None:
        start_draw = time.time()

        # START DRAWING BOXES
        for (class_id, confidence, box) in zip(
            self.classes_detected, self.confidences, self.boxes
        ):
            # mod operator to loop through all possible colors we specify if we have lots of classes
            color = self.COLORS[int(class_id) % len(self.COLORS)]
            # class_id is an np.array of 1 element
            label_name = self._classes[class_id[0]]
            label = "%s : %.3f" % (label_name, confidence)
            cv2.rectangle(self._img, box, color, self.FONT_THICCNESS)

            # box has the format (x,y,width,height)
            x, y, width, height = box
            # -1 to draw ABOVE the box
            self.write_text(label, x, y, color, 14, position=-1)

            # END OF DRAWING BOXES
            # START PROCESS DISTANCES

            if label_name == "sheep":
                # 122 = 61 height @ 2m
                # 146.5 = 293 height @ 0.5m
                dist = 134.25 / height
            elif label_name == "coke":
                # 92 = 92 height @ 1m
                dist = 92 / height

            obj_x = dist * np.cos(robot_pose[2]) + robot_pose[0]
            obj_y = dist * np.sin(robot_pose[2]) + robot_pose[1]
            x_label = "Xw:" + str(np.around(obj_x, 2))
            y_label = "Yw:" + str(np.around(obj_y, 2))
            dist_label = "D(m): %.2f" % (dist)
            self.write_text(x_label, x, y + height, color, 14, position=1)
            self.write_text(y_label, x, y + height, color, 14, position=2)
            self.write_text(dist_label, x, y, color, 14, position=-2)
        # END PROCESS DISTANCES

        inf_label = "INF(ms): %6.3fms" % (self.__inference_time)
        self.write_text(inf_label, 0, 25, self.COLORS[0], position=1)

        self.__draw_time = (time.time() - start_draw) * 1000

        draw_label = "DRW(ms): %6.3fms" % (self.__draw_time)
        self.write_text(draw_label, 0, 25, self.COLORS[0], position=2)

        fps_label = "FPS: %6.2fHz" % (
            1 * 1000 / (self.__draw_time + self.__inference_time)
        )
        self.write_text(fps_label, 0, 25, self.COLORS[0], position=3)

        cv2.imshow("YOLO", self._img)
        key = cv2.waitKey(1)
        if key == 27 or key == ord("q"):
            # del self
            cv2.destroyAllWindows()
            self._img.release()

    def write_text(
        self,
        label: str = "",
        x: int = 0,
        y: int = 0,
        color: tuple = (255, 255, 255),
        offset: int = None,
        position: int = 1,
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
        cv2.putText(
            self._img,
            label,
            (x, y + offset * position),
            self.FONT,
            self.FONT_SIZE,
            color,
            self.FONT_THICCNESS,
        )

    # TODO: Triangulation
    def generate_3d_pos(self):
        fov_horz = 1.0855
        foz_vert = 0.8517

    # def __del__(self):
    #     # Some weird funky stuff going on with manualSLAM.py
    #     cv2.destroyAllWindows("YOLO")
