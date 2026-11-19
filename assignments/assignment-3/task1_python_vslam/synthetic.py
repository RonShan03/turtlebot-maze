"""Deterministic rendered images with random textured 3D billboards; NOT KITTI."""
import json
from pathlib import Path
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from .dataset import save_poses
from .geometry import project, transform


def generate(root, frames=45, seed=11):
    if frames < 3:
        raise ValueError("synthetic sequence requires at least three frames")
    root = Path(root)
    folder = root / "sequences" / "00"
    (folder / "image_0").mkdir(parents=True, exist_ok=True)
    (root / "poses").mkdir(parents=True, exist_ok=True)
    if list((folder / "image_0").glob("*.png")):
        raise ValueError("use a new empty output directory to avoid stale synthetic frames")
    rng = np.random.default_rng(seed)
    K = np.array([[520., 0, 320], [0, 520., 240], [0, 0, 1]])
    centers = rng.uniform([-5, -3.5, 5], [6, 3.5, 18], (420, 3))
    textures = []
    for _ in centers:
        patch = rng.integers(20, 240, (7, 7), np.uint8)
        patch = cv2.resize(patch, (42, 42), interpolation=cv2.INTER_NEAREST)
        cv2.rectangle(patch, (0, 0), (41, 41), 250, 2)
        textures.append(patch)
    poses = []
    for i in range(frames):
        fraction = i / max(frames - 1, 1)
        pose = transform(Rotation.from_euler("y", 0.045 * np.sin(fraction * 2)).as_matrix(),
                         [1.5 * fraction, 0.08 * np.sin(4 * fraction), 0.8 * fraction])
        poses.append(pose)
        image = np.full((480, 640), 30, np.uint8)
        # Painter's algorithm gives real occlusion; each patch has a fixed world plane.
        for idx in np.argsort(centers[:, 2])[::-1]:
            x, y, z = centers[idx]
            size = 0.30
            corners = np.array([[x-size,y-size,z],[x+size,y-size,z],[x+size,y+size,z],[x-size,y+size,z]])
            uv, depth = project(K, pose, corners)
            if np.any(depth <= 0) or np.max(uv[:, 0]) < 0 or np.min(uv[:, 0]) >= 640 or np.max(uv[:, 1]) < 0 or np.min(uv[:, 1]) >= 480:
                continue
            H = cv2.getPerspectiveTransform(np.array([[0,0],[41,0],[41,41],[0,41]],np.float32), uv.astype(np.float32))
            warped = cv2.warpPerspective(textures[idx], H, (640,480))
            mask = cv2.warpPerspective(np.full((42,42),255,np.uint8), H, (640,480), flags=cv2.INTER_NEAREST)
            image[mask > 0] = warped[mask > 0]
        cv2.imwrite(str(folder / "image_0" / f"{i:06d}.png"), image)
    P = np.column_stack([K, np.zeros(3)])
    (folder / "calib.txt").write_text("P0: " + " ".join(map(str, P.ravel())) + "\n")
    np.savetxt(folder / "times.txt", np.arange(frames) / 10)
    save_poses(root / "poses" / "00.txt", poses)
    (root / "synthetic.json").write_text(json.dumps(dict(kind="synthetic rendered billboards, NOT KITTI", seed=seed, frames=frames), indent=2))
    return root
