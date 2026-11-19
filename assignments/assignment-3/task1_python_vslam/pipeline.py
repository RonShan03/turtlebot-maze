"""ORB + essential initialization + local landmark PnP tracking.

No ground truth is accepted by this module. It cannot supply online scale.
Failed frames retain a held output pose and are explicitly marked lost.
"""
from dataclasses import dataclass
import time
import cv2
import numpy as np
from .features import match_descriptors
from .geometry import inverse, relative_pose, triangulate, pnp_pose


@dataclass
class Config:
    features: int = 3500
    ratio: float = 0.75
    ransac_px: float = 1.5
    min_inliers: int = 20
    reprojection_px: float = 2.5
    parallax_deg: float = 0.5
    keyframe_interval: int = 5
    local_keyframes: int = 5
    seed: int = 7


@dataclass
class Frame:
    index: int
    image: np.ndarray
    keypoints: list
    descriptors: object
    pixels: np.ndarray
    landmarks: np.ndarray
    pose: np.ndarray


class VisualOdometry:
    def __init__(self, K, config=None):
        self.K, self.cfg = np.asarray(K, float), config or Config()
        cv2.setRNGSeed(self.cfg.seed)
        cv2.setNumThreads(1)
        self.orb = cv2.ORB_create(nfeatures=self.cfg.features, fastThreshold=12)
        self.previous = None
        self.keyframes = []
        self.last_step = 1.0
        self.last_visualization = None

    def process(self, index, timestamp, image):
        start = time.perf_counter()
        kp, desc = self.orb.detectAndCompute(image, None)
        pixels = np.asarray([k.pt for k in kp], float).reshape(-1, 2)
        frame = Frame(index, image, kp, desc, pixels, np.full((len(kp), 3), np.nan),
                      np.eye(4) if self.previous is None else self.previous.pose.copy())
        row = dict(frame=index, timestamp=timestamp, features=len(kp), matches=0, inliers=0,
                   inlier_ratio=0.0, new_landmarks=0, status="lost", reason="", keyframe=False)
        if self.previous is None:
            if len(kp) >= self.cfg.min_inliers:
                row.update(status="anchor", keyframe=True)
                self.previous = frame
                self.keyframes.append(frame)
            else:
                row["reason"] = "insufficient features to establish anchor"
        else:
            references = [self.previous] + [k for k in reversed(self.keyframes) if k.index != self.previous.index]
            pairs, matches = match_descriptors(self.previous.descriptors, desc, self.cfg.ratio)
            row["matches"] = len(pairs)
            accepted = None
            # Map tracking is preferred to arbitrary pairwise translation scaling.
            for ref in references:
                ids, ms = (pairs, matches) if ref is self.previous else match_descriptors(ref.descriptors, desc, self.cfg.ratio)
                known = np.isfinite(ref.landmarks[ids[:, 0]]).all(axis=1)
                selected = ids[known]
                try:
                    pose, inliers = pnp_pose(ref.landmarks[selected[:, 0]], pixels[selected[:, 1]], self.K,
                                             self.cfg.reprojection_px, self.cfg.min_inliers)
                    accepted = (ref, ids, ms, pose, selected[inliers], "pnp")
                    break
                except (ValueError, cv2.error):
                    continue
            if accepted is None:
                try:
                    relative, valid = relative_pose(self.previous.pixels[pairs[:, 0]], pixels[pairs[:, 1]], self.K,
                                                    self.cfg.ransac_px, self.cfg.min_inliers)
                    relative[:3, 3] *= self.last_step
                    pose = self.previous.pose @ inverse(relative)
                    ip = pairs[valid]
                    _, good, _ = triangulate(self.K, self.previous.pose, pose, self.previous.pixels[ip[:, 0]],
                                             pixels[ip[:, 1]], self.cfg.reprojection_px, self.cfg.parallax_deg)
                    if np.count_nonzero(good) < self.cfg.min_inliers:
                        raise ValueError("low parallax / unreliable triangulation")
                    mode = "essential_init" if not any(np.isfinite(k.landmarks).any() for k in self.keyframes) else "essential_fallback"
                    accepted = (self.previous, pairs, matches, pose, ip[good], mode)
                except (ValueError, cv2.error) as exc:
                    row["reason"] = str(exc)
            if accepted is not None:
                ref, ids, ms, pose, inlier_pairs, mode = accepted
                frame.pose = pose
                row.update(status=mode, matches=len(ids), inliers=len(inlier_pairs),
                           inlier_ratio=len(inlier_pairs) / max(1, len(ids)))
                # Only geometrically accepted tracks carry existing map points.
                inherited = np.isfinite(ref.landmarks[inlier_pairs[:, 0]]).all(axis=1)
                frame.landmarks[inlier_pairs[inherited, 1]] = ref.landmarks[inlier_pairs[inherited, 0]]
                # Triangulate against the latest keyframe to allow useful baseline.
                mapping_ref = self.keyframes[-1]
                mid, _ = match_descriptors(mapping_ref.descriptors, desc, self.cfg.ratio)
                X, good, _ = triangulate(self.K, mapping_ref.pose, pose, mapping_ref.pixels[mid[:, 0]], pixels[mid[:, 1]],
                                         self.cfg.reprojection_px, self.cfg.parallax_deg)
                new = good & ~np.isfinite(mapping_ref.landmarks[mid[:, 0]]).all(axis=1)
                mapping_ref.landmarks[mid[new, 0]] = X[new]
                frame.landmarks[mid[new, 1]] = X[new]
                # Existing keyframe points still need a current-frame reprojection gate.
                from .geometry import project
                known = np.isfinite(mapping_ref.landmarks[mid[:, 0]]).all(axis=1)
                uv, depth = project(self.K, pose, mapping_ref.landmarks[mid[:, 0]])
                reusable = known & (depth > 0) & (np.linalg.norm(uv - pixels[mid[:, 1]], axis=1) < self.cfg.reprojection_px)
                frame.landmarks[mid[reusable, 1]] = mapping_ref.landmarks[mid[reusable, 0]]
                row["new_landmarks"] = int(new.sum())
                if index - mapping_ref.index >= self.cfg.keyframe_interval:
                    self.keyframes.append(frame)
                    self.keyframes = self.keyframes[-self.cfg.local_keyframes:]
                    row["keyframe"] = True
                step = np.linalg.norm(pose[:3, 3] - self.previous.pose[:3, 3])
                if step > 1e-5:
                    self.last_step = step
                self.previous = frame
                accepted_set = set(map(tuple, inlier_pairs.tolist()))
                mask = [int((m.queryIdx, m.trainIdx) in accepted_set) for m in ms]
                self.last_visualization = cv2.drawMatches(ref.image, ref.keypoints, image, kp, ms[:120], None,
                    matchesMask=mask[:120], flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
        row["seconds"] = time.perf_counter() - start
        return frame.pose, row, frame.landmarks[np.isfinite(frame.landmarks).all(axis=1)]
