import numpy as np
import cv2
import slam.Measurements as Measurements
import sys

sys.path.append("..")
from utils import load_yaml

params = load_yaml("config.yml")
PINK = params["YOLO"]["DRAWING"]["colors"][0]


class aruco_detector:
    def __init__(self, camera_matrix, camera_dist, marker_length=0.1):
        self.camera_matrix = camera_matrix
        self.distortion_params = camera_dist

        self.marker_length = marker_length
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        # THESE ARE ALL DEFAULT PARAMS
        self.aruco_params.adaptiveThreshWinSizeMin = 3
        self.aruco_params.adaptiveThreshWinSizeMax = 23
        self.aruco_params.adaptiveThreshWinSizeStep = 10
        self.aruco_params.adaptiveThreshConstant = 7
        self.aruco_params.minMarkerPerimeterRate = 0.03
        self.aruco_params.maxMarkerPerimeterRate = 4.0
        self.aruco_params.polygonalApproxAccuracyRate = 0.03
        self.aruco_params.minCornerDistanceRate = 0.05
        self.aruco_params.minDistanceToBorder = 3
        self.aruco_params.minMarkerDistanceRate = 0.05
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
        self.aruco_params.cornerRefinementWinSize = 3
        self.aruco_params.cornerRefinementMaxIterations = 30
        self.aruco_params.cornerRefinementMinAccuracy = 0.1
        self.aruco_params.markerBorderBits = 1
        self.aruco_params.perspectiveRemovePixelPerCell = 4
        self.aruco_params.perspectiveRemoveIgnoredMarginPerCell = 0.13
        self.aruco_params.maxErroneousBitsInBorderRate = 0.35
        self.aruco_params.minOtsuStdDev = 5.0
        self.aruco_params.errorCorrectionRate = 0.6

        # CUSTOM PARAMS HERE
        # DO NOT USE CORNER_REFINE_CONTOUR
        self.aruco_params.minMarkerDistanceRate = 0.05 * 2
        self.aruco_params.minDistanceToBorder = 0
        self.aruco_params.adaptiveThreshWinSizeMax = 1000
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.aruco_params.cornerRefinementMaxIterations = 100
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)

    def detect_marker_positions(self, img):
        # Perform detection
        corners, ids, rejected = cv2.aruco.detectMarkers(
            img,
            self.aruco_dict,
            parameters=self.aruco_params,
            cameraMatrix=self.camera_matrix,
            distCoeff=self.distortion_params,
        )

        if ids is None:
            return [], img

        # Marker length should be scaled to pixel size
        _, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, self.marker_length, self.camera_matrix, self.distortion_params
        )

        id_flag = []
        MAX_DISTANCE = params["ARUCO"]["max_tvecs_norm"]

        if tvecs is not None:
            for i in range(tvecs.shape[0]):
                total = np.linalg.norm(tvecs[i][0])
                if total > MAX_DISTANCE:
                    id_flag.append(ids[i][0])

        # Compute the marker positions
        measurements = []
        seen_ids = []
        for i in range(len(ids)):
            # Some markers appear multiple times but should only be handled once.
            idi = ids[i, 0]

            if np.any(id_flag == idi):
                continue

            if idi in seen_ids:
                continue
            else:
                seen_ids.append(idi)

            lm_tvecs = tvecs[ids == idi].T
            lm_bff2d = np.block([[lm_tvecs[2, :]], [-lm_tvecs[0, :]]])
            lm_bff2d = np.mean(lm_bff2d, axis=1).reshape(-1, 1)
            # Covariance not specified
            lm_measurement = Measurements.MarkerMeasurement(lm_bff2d, idi)
            measurements.append(lm_measurement)

        # img_marked = img.copy()
        # Draw markers on original
        cv2.aruco.drawDetectedMarkers(img, corners, ids, PINK)
        # Draw markers on copy
        # cv2.aruco.drawDetectedMarkers(img_marked, corners, ids)

        return measurements, img
