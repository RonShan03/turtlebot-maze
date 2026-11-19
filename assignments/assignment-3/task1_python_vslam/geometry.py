"""Calibrated two-view geometry and SE(3) helpers. Optical axes: right, down, forward."""
import cv2
import numpy as np


def transform(R=None, t=None):
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = R
    if t is not None:
        T[:3, 3] = np.asarray(t).reshape(3)
    return T


def inverse(T):
    return transform(T[:3, :3].T, -T[:3, :3].T @ T[:3, 3])


def project(K, T_wc, points):
    T_cw = inverse(T_wc)
    camera = np.asarray(points) @ T_cw[:3, :3].T + T_cw[:3, 3]
    pixels = camera @ K.T
    with np.errstate(divide="ignore", invalid="ignore"):
        return pixels[:, :2] / pixels[:, 2:3], camera[:, 2]


def relative_pose(p1, p2, K, threshold=1.5, min_inliers=20):
    """Return T_camera2_camera1 and cheirality inliers, with unknown translation scale."""
    if len(p1) < max(8, min_inliers):
        raise ValueError("insufficient matches")
    q1 = cv2.undistortPoints(np.asarray(p1, np.float64).reshape(-1, 1, 2), K, None).reshape(-1, 2)
    q2 = cv2.undistortPoints(np.asarray(p2, np.float64).reshape(-1, 1, 2), K, None).reshape(-1, 2)
    E, mask = cv2.findEssentialMat(q1, q2, np.eye(3), method=cv2.RANSAC,
                                  prob=0.999, threshold=threshold / np.mean([K[0, 0], K[1, 1]]))
    if E is None or mask is None:
        raise ValueError("essential matrix estimation failed")
    candidates = []
    for e in E.reshape(-1, 3, 3):
        count, R, t, valid = cv2.recoverPose(e, q1, q2, np.eye(3), mask=mask.copy())
        candidates.append((count, R, t, valid.ravel() != 0))
    count, R, t, valid = max(candidates, key=lambda x: x[0])
    if count < min_inliers:
        raise ValueError("insufficient cheirality inliers / degenerate motion")
    return transform(R, t), valid


def triangulate(K, T_w1, T_w2, p1, p2, max_error=2.5, min_parallax=0.5):
    """Triangulate in world coordinates; return points, validity, max pixel error."""
    if not len(p1):
        return np.empty((0, 3)), np.zeros(0, bool), np.empty(0)
    P1, P2 = K @ inverse(T_w1)[:3], K @ inverse(T_w2)[:3]
    h = cv2.triangulatePoints(P1, P2, np.asarray(p1, float).T, np.asarray(p2, float).T).T
    nonzero = np.abs(h[:, 3]) > 1e-10
    X = np.full((len(h), 3), np.nan)
    X[nonzero] = h[nonzero, :3] / h[nonzero, 3:4]
    uv1, z1 = project(K, T_w1, X)
    uv2, z2 = project(K, T_w2, X)
    error = np.maximum(np.linalg.norm(uv1 - p1, axis=1), np.linalg.norm(uv2 - p2, axis=1))
    ray1, ray2 = X - T_w1[:3, 3], X - T_w2[:3, 3]
    denominator = np.linalg.norm(ray1, axis=1) * np.linalg.norm(ray2, axis=1)
    with np.errstate(divide="ignore", invalid="ignore"):
        angle = np.degrees(np.arccos(np.clip(np.sum(ray1 * ray2, axis=1) / denominator, -1, 1)))
    valid = (nonzero & np.isfinite(X).all(axis=1) & (z1 > 0) & (z2 > 0)
             & (error <= max_error) & (angle >= min_parallax))
    return X, valid, error


def pnp_pose(points3d, pixels, K, threshold=2.5, min_inliers=12):
    if len(points3d) < min_inliers:
        raise ValueError("insufficient 2D-3D correspondences")
    ok, rvec, tvec, inliers = cv2.solvePnPRansac(
        np.asarray(points3d, np.float64), np.asarray(pixels, np.float64), K, None,
        iterationsCount=200, reprojectionError=threshold, confidence=0.999, flags=cv2.SOLVEPNP_EPNP)
    if not ok or inliers is None or len(inliers) < min_inliers:
        raise ValueError("PnP RANSAC failed")
    ids = inliers.ravel()
    rvec, tvec = cv2.solvePnPRefineLM(np.asarray(points3d, float)[ids], np.asarray(pixels, float)[ids],
                                    K, None, rvec, tvec)
    T_wc = inverse(transform(cv2.Rodrigues(rvec)[0], tvec))
    uv, z = project(K, T_wc, points3d)
    valid = (z > 0) & (np.linalg.norm(uv - pixels, axis=1) <= threshold)
    if np.count_nonzero(valid) < min_inliers:
        raise ValueError("PnP refinement rejected")
    return T_wc, valid
