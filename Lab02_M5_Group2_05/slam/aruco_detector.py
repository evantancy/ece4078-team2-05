import numpy as np
import cv2
import slam.Measurements as Measurements

class aruco_detector:
    def __init__(self, robot, marker_length=0.1):
        self.camera_matrix = robot.camera_matrix
        self.distortion_params = robot.camera_dist

        self.marker_length = marker_length
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)

    def detect_marker_positions(self, img):
        # Perform detection
        corners, ids, rejected = cv2.aruco.detectMarkers(img, self.aruco_dict, parameters=self.aruco_params)
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_length, self.camera_matrix, self.distortion_params)
        id_flag = []
        # print(tvecs)
        # print(tvecs.shape)

        if ids is None:
            return [], img

        if tvecs is not None:
            for i in range(tvecs.shape[0]):
                # print(tvecs[i][0])
                total = np.linalg.norm(tvecs[i][0])
                print(total)
                if total > 15:
                    id_flag.append(ids[i][0])



        # Compute the marker positions
        measurements = []
        seen_ids = []
        for i in range(len(ids)):
            idi = ids[i,0]
            # Some markers appear multiple times but should only be handled once.

            if np.any(id_flag == idi):
                continue
            else:
                if idi in seen_ids:
                    continue
                else:
                    seen_ids.append(idi)

                lm_tvecs = tvecs[ids==idi].T
                lm_bff2d = np.block([[lm_tvecs[2,:]],[-lm_tvecs[0,:]]])
                lm_bff2d = np.mean(lm_bff2d, axis=1).reshape(-1,1)

                lm_measurement = Measurements.MarkerMeasurement(lm_bff2d, idi)
                measurements.append(lm_measurement)

        # Draw markers on image copy
        img_marked = img.copy()
        cv2.aruco.drawDetectedMarkers(img_marked, corners, ids)

        return measurements, img_marked
