import cv2
import numpy as np
import time
import sys


class YOLO:
    def __init__(
        self,
        weights_path: str = "yolo_cfg/custom-yolov4-tiny-detector_final.weights",
        cfg_path: str = "yolo_cfg/custom-yolov4-tiny-detector.cfg",
        classes_path: str = "yolo_cfg/obj.names",
    ) -> None:
        """
        Args:
            weights_path (str, optional): Defaults to "yolo_cfg/custom-yolov4-detector_final.weights".
            cfg_path (str, optional): Defaults to "yolo_cfg/custom-yolov4-detector.cfg".
            classes_path (str, optional): Defaults to "yolo_cfg/obj.names".
        """
        # Load YOLO neural network
        self._net = cv2.dnn.readNet(weights_path, cfg_path)
        self._net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
        self._net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)

        # load labels/classes from obj.names file
        self._classes = []
        with open(classes_path, "r") as f:
            self._classes = [line.strip() for line in f.readlines()]

        self.__CONFIDENCE_THRESHOLD = 0.2
        self.__SCORE_THRESHOLD = 0.2
        self.__IOU_THRESHOLD = 0.2

        self.colors = np.random.uniform(0, 255, size=(len(self._classes), 3))
        self.frame_id = 0
        self.starting_time = time.time()

    def run_inference(self, img=None) -> None:
        """
        Args:
            img (cv2 frame, optional): Defaults to None.
        """
        self.frame_id += 1
        height, width, channels = img.shape
        # detecting objects
        blob = cv2.dnn.blobFromImage(
            img, 1 / 255.0, (416, 416), (0, 0, 0), True, crop=False
        )
        # use blob as input to neural network
        self._net.setInput(blob)
        # output layers
        ln = self._net.getLayerNames()
        output_layers = [ln[i[0] - 1] for i in self._net.getUnconnectedOutLayers()]
        output_layers = self._net.forward(output_layers)

        # showing informations on the screen
        self.class_ids = []
        self.confidences = []
        self.boxes = []

        for output in output_layers:
            for detection in output:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > self.__CONFIDENCE_THRESHOLD:
                    # extract bounding box features
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)

                    # box width and height
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)

                    # top left of bounding box
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)
                    self.boxes.append([x, y, int(w), int(h)])
                    self.confidences.append(float(confidence))
                    self.class_ids.append(class_id)

        # non-max suppression to prevent overlapping boxes
        self._indices = cv2.dnn.NMSBoxes(
            self.boxes, self.confidences, self.__SCORE_THRESHOLD, self.__IOU_THRESHOLD
        )

    def draw_boxes(self, img=None):

        font = cv2.FONT_HERSHEY_PLAIN
        for i in range(len(self.boxes)):
            if i in self._indices:
                x, y, w, h = self.boxes[i]
                label = str(self._classes[self.class_ids[i]])
                confidence = self.confidences[i]
                color = self.colors[self.class_ids[i]]
                cv2.rectangle(img, (x, y), (x + w, y + h), color, 2)
                cv2.putText(
                    img,
                    label + " " + str(round(confidence, 2)),
                    (x, y + 30),
                    font,
                    1,
                    (255, 255, 255),
                    2,
                )
        fps = self.frame_id / (time.time() - self.starting_time)
        cv2.putText(img, "FPS:" + str(round(fps, 2)), (10, 50), font, 2, (0, 0, 0), 1)
        # uncomment if you want to show new window!
        cv2.imshow("YOLO IMAGE", img)
        key = cv2.waitKey(1)
        if key == 27 or key == "q":
            del(self)

    def __del__(self):
        # Some weird funky stuff going on with manualSLAM.py
        cv2.destroyWindow("YOLO IMAGE")
