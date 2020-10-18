import cv2
import time
import sys
import numpy as np

sys.path.append("../")
from PenguinPiC import PenguinPi
from KeyboardController import Keyboard


ppi = PenguinPi()
controller = Keyboard(ppi)

CONFIDENCE_THRESHOLD = 0.6
NMS_THRESHOLD = 0.4
COLORS = [(0, 255, 255), (255, 255, 0), (0, 255, 0), (255, 0, 0)]

class_names = []
with open("cfg/obj.names", "r") as f:
    class_names = [cname.strip() for cname in f.readlines()]

weights_path = "cfg/custom-yolov4-tiny-detector_final.weights"
cfg_path = "cfg/custom-yolov4-tiny-detector.cfg"

# load YOLOv4 network
net = cv2.dnn.readNet(weights_path, cfg_path)
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
# net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA_FP16)

model = cv2.dnn_DetectionModel(net)
model.setInputParams(size=(416, 416), scale=1 / 255)
while True:
    start = time.time()

    frame = ppi.get_image()
    classes, scores, boxes = model.detect(frame, CONFIDENCE_THRESHOLD, NMS_THRESHOLD)
    # end = time.time()

    start_drawing = time.time()
    for (classid, score, box) in zip(classes, scores, boxes):
        color = COLORS[int(classid) % len(COLORS)]
        label = "%s : %f" % (class_names[classid[0]], score)
        cv2.rectangle(frame, box, color, 2)
        cv2.putText(
            frame, label, (box[0], box[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2
        )

    end_drawing = time.time()
    end = time.time()

    fps_label = "FPS: %.2f" % (1 / (end - start))
    drawing_label = "DRW: %.2fms" % ((end_drawing - start_drawing) * 1000)
    cv2.putText(frame, fps_label, (0, 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)
    cv2.putText(
        frame, drawing_label, (0, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2
    )
    cv2.imshow("detections", frame)
    key = cv2.waitKey(1)
    if key == 27 or key == ord("q"):
        break
