import cv2
import numpy as np


def match_descriptors(a, b, ratio=0.75):
    """Hamming KNN Lowe ratio plus mutual nearest-neighbor consistency."""
    if a is None or b is None or len(a) < 2 or len(b) < 2:
        return np.empty((0, 2), dtype=int), []
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
    forward = matcher.knnMatch(a, b, k=2)
    reverse = {m.queryIdx: m.trainIdx for m in matcher.match(b, a)}
    accepted = [pair[0] for pair in forward if len(pair) == 2
                and pair[0].distance < ratio * pair[1].distance
                and reverse.get(pair[0].trainIdx) == pair[0].queryIdx]
    return np.asarray([(m.queryIdx, m.trainIdx) for m in accepted], dtype=int).reshape(-1, 2), accepted
