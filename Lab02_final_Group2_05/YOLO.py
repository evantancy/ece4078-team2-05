import cv2
import numpy as np
from CvTimer import CvTimer
from utils import load_yaml

# TODO: make sure this works with absolute and relative paths
params = load_yaml("config.yml")["YOLO"]


class YOLO:
    CONFIDENCE_THRESHOLD = params["THRESHOLD"]["confidence"]
    NMS_THRESHOLD = params["THRESHOLD"]["NMS"]
    COLORS = params["DRAWING"]["colors"]
    SIZE = params["NETWORK_SIZE"]
    FONT = cv2.FONT_HERSHEY_SIMPLEX
    FONT_SIZE = params["DRAWING"]["font_size"]
    FONT_THICCNESS = params["DRAWING"]["font_thiccness"]
    TEXT_OFFSET = params["DRAWING"]["text_offset"]
    FOV = {
        "vertical": params["FOV"]["vertical"],
        "horizontal": params["FOV"]["horizontal"],
    }

    MAX_MATCH_ERROR = 0.2
    DUPE_THRESH = params["THRESHOLD"]["dupe_abs_dist"]
    DUPE_X_THRESH = params["THRESHOLD"]["dupe_x"]
    DUPE_Y_THRESH = params["THRESHOLD"]["dupe_y"]
    CAPTURE_WINDOW = params["THRESHOLD"]["capture_window"]

    def __init__(
        self,
        gpu: int = 0,
        weights_path: str = params["PATH"]["weights"],
        cfg_path: str = params["PATH"]["cfg"],
        classes_path: str = params["PATH"]["classes"],
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
        self._object_counts = dict()
        for label in self._classes:
            self._object_counts[label] = 0

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
        seen_objects=[],
        external_timer: CvTimer = None,
    ) -> None:

        Timer = external_timer
        if external_timer is None:
            Timer = self._TIMER
        Timer.start("draw")

        # uncomment for colors that change every iteration
        # self.COLORS = np.random.uniform(0, 255, size=(len(self._classes), 3))

        FRAME_HEIGHT, FRAME_WIDTH, _ = self._img.shape  # pixels
        # WIDTH or HEIGHT based should NOT matter, but values vary by 1 ish
        FOCAL_LENGTH_W = (FRAME_WIDTH / 2) / np.tan(
            self.FOV["horizontal"] / 2
        )  # pixels
        FOCAL_LENGTH_H = (FRAME_HEIGHT / 2) / np.tan(self.FOV["vertical"] / 2)  # pixels
        COKE = {"x": 0.06, "y": 0.06, "z": 0.14}
        SHEEP = {"x": 0.108, "y": 0.223, "z": 0.204}
        SCALE_HEIGHT = None
        SCALE_WIDTH = None

        # START DRAWING BOXES
        for (class_id, confidence, box) in zip(
            self._classes_detected, self._confidences, self._boxes
        ):
            # mod operator to loop through all possible colors we specify if we have lots of classes
            color = self.COLORS[int(class_id) % len(self.COLORS)]
            # class_id is an np.array of 1 element
            label_name = self._classes[class_id[0]]
            label = "%s: %.2f" % (label_name, confidence)
            MAX_DISTANCE = params["THRESHOLD"]["max_distance_" + label_name]  # metres

            # box has the format (x,y,width,height)
            x, y, width, height = box
            centre_x = int(x + width / 2)
            centre_y = int(y + height / 2)

            cv2.rectangle(self._img, box, color, self.FONT_THICCNESS)
            cv2.circle(self._img, (centre_x, centre_y), 1, color, 2)
            # -1 to draw ABOVE the box
            self.write_text(label, x, y, color, 14, position=-1)

            annot_dist = None
            scale_dist = None
            focal_dist = None
            if label_name == "sheep":
                # bounding box dimensions @ 1m
                # DO NOT USE WIDTH FOR SHEEP (ASYMMETRICAL!)
                SCALE_HEIGHT = 125
                # SCALE_WIDTH = 135.5
                a, b, c = 0.1198, -1.0862, 5.8189
                coeffs = [a, b, c - np.log(height)]
                inner_term = b ** 2 - 4 * a * (c - np.log(height))
                if inner_term > 0:
                    annot_dist = (-b - np.sqrt(inner_term)) / (2 * a)
                else:
                    annot_dist = 99
                scale_dist = SCALE_HEIGHT / height
                expected_height = SHEEP["z"]
                focal_dist = (expected_height * FOCAL_LENGTH_H) / height

            elif label_name == "coke":
                # bounding box dimensions @ 1m
                SCALE_HEIGHT = 93.5
                SCALE_WIDTH = 37.5

                coeffs = [
                    -0.0623473695584497,
                    0.699609391058590,
                    -2.72872515672850,
                    6.67872601634846 - np.log(height),
                ]
                curr_soln = np.roots(coeffs)
                check_real = np.isreal(curr_soln)
                annot_dist = curr_soln[check_real == True]
                annot_dist = np.real(annot_dist)
                if len(annot_dist) != 1:
                    annot_dist = 99
                expected_height = COKE["z"]
                expected_width = COKE["x"]
                # units:(m), distance based on focal lengths
                # f_dist_w = (expected_width * FOCAL_LENGTH_W) / width
                f_dist_h = (expected_height * FOCAL_LENGTH_H) / height
                focal_dist = f_dist_h

                # units:(m), distance based on bounding boxes
                distance_w = SCALE_WIDTH / width
                distance_h = SCALE_HEIGHT / height
                scale_dist = (distance_w + distance_h) / 2

            centre_offset = centre_x - FRAME_WIDTH / 2
            focal_length = (FOCAL_LENGTH_W + FOCAL_LENGTH_H) / 2
            # Remove dependence on capturing objects in centre of frame
            # angle between object in 2D frame and robot's pose
            alpha = np.arctan2(centre_offset, focal_length)

            # World frame
            obj_x = annot_dist * np.cos(robot_pose[2] - alpha) + robot_pose[0]
            obj_y = annot_dist * np.sin(robot_pose[2] - alpha) + robot_pose[1]
            scale_x = scale_dist * np.cos(robot_pose[2] - alpha) + robot_pose[0]
            scale_y = scale_dist * np.sin(robot_pose[2] - alpha) + robot_pose[1]
            focal_x = focal_dist * np.cos(robot_pose[2] - alpha) + robot_pose[0]
            focal_y = focal_dist * np.sin(robot_pose[2] - alpha) + robot_pose[1]

            # Calculate discrepancy between 2 methods
            # match_error = abs((annot_dist - focal_dist) / annot_dist)

            save_pose = False
            # check if box centre is within capture window
            if abs((x + width / 2) - FRAME_WIDTH / 2) < self.CAPTURE_WINDOW:
                if annot_dist < MAX_DISTANCE:
                    save_pose = True
                    if len(seen_objects) != 0:
                        for obj in seen_objects:
                            label = obj[0]
                            # only compare cokes and sheeps with each other
                            if label[0] == label_name[0]:
                                delta_obj_x = abs(obj[1] - obj_x[0])
                                delta_obj_y = abs(obj[2] - obj_y[0])
                                delta_obj_dist = np.sqrt(
                                    delta_obj_x ** 2 + delta_obj_y ** 2
                                )
                                # check for duplicate entries
                                if delta_obj_dist < self.DUPE_THRESH:
                                    save_pose = False

                            # if delta_obj_x < DUPE_X_THRESH and delta_obj_y < DUPE_Y_THRESH:
                            #     save_pose = False

            if save_pose:
                n = None
                if self._object_counts[label_name] < 10:
                    self._object_counts[label_name] += 1
                    n = self._object_counts[label_name]
                else:
                    print("Duplicate found")
                    # TODO: Remove duplicates
                label_name = label_name + str(n)
                seen_objects.append([label_name, obj_x[0], obj_y[0]])
                seen_objects.append([label_name, scale_x[0], scale_y[0]])
                seen_objects.append([label_name, focal_x[0], focal_y[0]])
                print(
                    f"Detected {label_name} @x:{obj_x[0]:.2f} y:{obj_y[0]:.2f} D: {np.around(annot_dist,3)}"
                )
                print(
                    f"         {label_name} @x:{scale_x[0]:.2f} y:{scale_y[0]:.2f} D: {scale_dist:.3f}"
                )
                print(
                    f"         {label_name} @x:{focal_x[0]:.2f} y:{focal_y[0]:.2f} D: {focal_dist:.3f}"
                )

            x_label = f"Xw: {np.around(obj_x, 2)}"
            y_label = f"Yw: {np.around(obj_y, 2)}"
            w_label = f"W: {width}"
            h_label = f"H: {height}"
            dist_label = f"D(m): {np.around(annot_dist,2)}"
            self.write_text(x_label, x, y + height, color, 14, position=1)
            self.write_text(y_label, x, y + height, color, 14, position=2)
            # self.write_text(w_label, x, y + height + 30, color, 14, position=1)
            # self.write_text(h_label, x, y + height + 30, color, 14, position=2)
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

    def detect_duplicates(self, seen_objects=[]):
        pass

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
