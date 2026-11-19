"""Strict KITTI odometry readers (rectified image_0, calibration P0, T_world_camera)."""
from pathlib import Path
import cv2
import numpy as np


def load_poses(path):
    rows = np.loadtxt(path, ndmin=2)
    if rows.shape[1] != 12 or not np.isfinite(rows).all():
        raise ValueError("KITTI poses must contain twelve finite values per row")
    poses = np.repeat(np.eye(4)[None], len(rows), axis=0)
    poses[:, :3] = rows.reshape(-1, 3, 4)
    if not np.allclose(poses[:, :3, :3] @ poses[:, :3, :3].transpose(0, 2, 1), np.eye(3), atol=1e-4):
        raise ValueError("pose rotations are not orthogonal")
    if not np.allclose(np.linalg.det(poses[:, :3, :3]), 1, atol=1e-4):
        raise ValueError("pose rotations must have determinant +1")
    return poses


def save_poses(path, poses):
    np.savetxt(path, np.asarray(poses)[:, :3].reshape(-1, 12), fmt="%.10f")


class KittiSequence:
    def __init__(self, root, sequence="00", max_frames=None):
        root = Path(root)
        self.folder = root / "sequences" / sequence
        calibration = {}
        for line in (self.folder / "calib.txt").read_text().splitlines():
            if ":" in line:
                key, values = line.split(":", 1)
                calibration[key] = np.fromstring(values, sep=" ")
        if "P0" not in calibration or calibration["P0"].size != 12:
            raise ValueError("calib.txt requires rectified P0 with twelve values")
        self.K = calibration["P0"].reshape(3, 4)[:, :3]
        if self.K[0, 0] <= 0 or self.K[1, 1] <= 0 or not np.isfinite(self.K).all():
            raise ValueError("invalid camera calibration")
        files = sorted((self.folder / "image_0").glob("*.png"))
        if not files:
            raise ValueError(f"no PNG images in {self.folder / 'image_0'}")
        ids = [int(p.stem) for p in files]
        if ids != list(range(len(files))):
            raise ValueError("images must be contiguous from 000000.png; extract a prefix")
        if max_frames is not None and max_frames < 2:
            raise ValueError("max_frames must be at least two")
        self.images = files[:max_frames]
        times = np.loadtxt(self.folder / "times.txt", ndmin=1)
        if len(times) < len(self.images) or not np.isfinite(times).all() or np.any(np.diff(times) <= 0):
            raise ValueError("timestamps must be finite, increasing and cover all images")
        self.times = times[:len(self.images)]
        pose_path = root / "poses" / f"{sequence}.txt"
        self.ground_truth = load_poses(pose_path)[:len(self.images)] if pose_path.exists() else None
        if self.ground_truth is not None and len(self.ground_truth) != len(self.images):
            raise ValueError("ground truth does not cover the selected frames")

    def __iter__(self):
        shape = None
        for index, (path, timestamp) in enumerate(zip(self.images, self.times)):
            image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
            if image is None:
                raise ValueError(f"unreadable image: {path}")
            if shape is not None and image.shape != shape:
                raise ValueError("image dimensions changed within sequence")
            shape = image.shape
            yield index, float(timestamp), image
