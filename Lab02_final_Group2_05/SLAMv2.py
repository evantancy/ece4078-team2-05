import numpy as np
import cv2
from PenguinPiC import PenguinPi


class Extractor:
    def __init__(self):
        self.orb = cv2.ORB_create(100)
        self.bf = cv2.BFMatcher()
        self.last = None

    def extract(self, img, mask=None):
        # detection
        features = None
        # TODO: Feed bounding boxes / aruco image segments
        features = cv2.goodFeaturesToTrack(
            np.mean(img, axis=2).astype(np.uint8),
            3000,
            qualityLevel=0.01,
            minDistance=3,
        )

        # extraction
        kps = [cv2.KeyPoint(x=feat[0][0], y=feat[0][1], _size=20) for feat in features]
        kps, des = self.orb.compute(img, kps)

        # matching
        matches = None
        if self.last is not None:
            matches = self.bf.match(des, self.last["des"])

        # store last frame info
        if kps is not None and des is not None:
            self.last = {"kps": kps, "des": des}

        return kps, des, matches

    def process_frame(self, img, mask=None):
        kp, des, matches = self.extract(img, mask)
        if kp is not None:
            for p in kp:
                u, v = map(lambda x: int(round(x)), p.pt)
                cv2.circle(img, (u, v), color=(0, 255, 0), radius=3)
        if matches is not None:
            for m in matches:
                print(m)