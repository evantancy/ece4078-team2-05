import cv2
import numpy as np
import time
import sys

sys.path.append("../")
from PenguinPiC import PenguinPi
from KeyboardController import Keyboard

if __name__ == "__main__":
    # configure path to .weights and .cfg files
    weights_path = "cfg/custom-yolov4-detector_final.weights"
    cfg_path = "cfg/custom-yolov4-detector.cfg"
    CONFIDENCE_THRESHOLD = 0.2

    # load YOLOv4 network
    net = cv2.dnn.readNet(weights_path, cfg_path)
    net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
    # net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
    net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA_FP16)
    layer_names = net.getLayerNames()
    output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]

    # load labels/classes from obj.names file
    classes = []
    with open("cfg/obj.names", "r") as f:
        classes = [line.strip() for line in f.readlines()]

    ppi = PenguinPi()
    controller = Keyboard(ppi)
    colors = np.random.uniform(0, 255, size=(len(classes), 3))
    font = cv2.FONT_HERSHEY_PLAIN
    frame_id = 0
    starting_time = time.time()

    while True:
        img = ppi.get_image()
        height, width, channels = img.shape
        frame_id += 1

        # detecting objects
        blob = cv2.dnn.blobFromImage(
            img, 1 / 255.0, (416, 416), (0, 0, 0), True, crop=False
        )
        # use blob as input to neural network
        net.setInput(blob)
        # output layers
        output_layers = net.forward(output_layers)

        # showing informations on the screen
        class_ids, confidences, boxes = [], [], []

        for output in output_layers:
            for detection in output:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > CONFIDENCE_THRESHOLD:
                    # extract bounding box features
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)

                    # box width and height
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)

                    # top left of bounding box
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)
                    boxes.append([x, y, int(w), int(h)])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)
        SCORE_THRESHOLD = 0.2
        IOU_THRESHOLD = 0.2
        # non-max suppression to prevent overlapping boxes
        indexes = cv2.dnn.NMSBoxes(boxes, confidences, SCORE_THRESHOLD, IOU_THRESHOLD)

        # draing boxes on the frames
        for i in range(len(boxes)):
            if i in indexes:
                x, y, w, h = boxes[i]
                label = str(classes[class_ids[i]])
                confidence = confidences[i]
                color = colors[class_ids[i]]
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
        fps = frame_id / (time.time() - starting_time)
        cv2.putText(img, "FPS:" + str(round(fps, 2)), (10, 50), font, 2, (0, 0, 0), 1)
        cv2.imshow("Image", img)

        key = cv2.waitKey(1)
        if key == 27 or key == "q":
            break
    cv2.destroyAllWindows()
