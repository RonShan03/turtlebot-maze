"""Camera synchronization, back-projection, and map transform utilities."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Iterable, Optional, Sequence, Tuple

import numpy as np

CAMERA_INTRINSICS = {
    "fx": 277.13,
    "fy": 277.13,
    "cx": 160.0,
    "cy": 120.0,
    "width": 320,
    "height": 240,
}

# Compatibility alias for the public package API used by assignment imports.
CAMERA_INSTRINSICS = CAMERA_INTRINSICS

R_OPTICAL_TO_BODY = np.array(
    [
        [0.0, 0.0, 1.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
    ],
    dtype=np.float64,
)

T_BODY_TO_CAMERA = np.array(
    [
        [1.0, 0.0, 0.0, 0.064],
        [0.0, 1.0, 0.0, -0.065],
        [0.0, 0.0, 1.0, 0.094],
        [0.0, 0.0, 0.0, 1.0],
    ],
    dtype=np.float64,
)


@dataclass
class SynchronizedFrame:
    color_ts_ns: int
    depth_ts_ns: int
    odom_ts_ns: int
    color_msg: Any
    depth_msg: Any
    odom_msg: Any


def _timestamp_ns(msg: Any) -> Optional[int]:
    header = getattr(msg, "header", None)
    if header is None:
        return None
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return None
    sec = getattr(stamp, "sec", 0)
    nanosec = getattr(stamp, "nanosec", 0)
    return int(sec) * 1_000_000_000 + int(nanosec)


def synchronize_color_depth_odometry(color_msg: Any, depth_msg: Any, odom_msg: Any, max_delta_ns: int = 200_000_000) -> SynchronizedFrame:
    """Synchronize frames by closest depth/odom timestamp while enforcing 200 ms tolerance."""
    color_ts = _timestamp_ns(color_msg)
    depth_ts = _timestamp_ns(depth_msg)
    odom_ts = _timestamp_ns(odom_msg)

    if color_ts is None or depth_ts is None or odom_ts is None:
        raise ValueError("All messages must include valid ROS header timestamps.")

    if abs(depth_ts - color_ts) > max_delta_ns:
        raise ValueError("Depth image is not synchronized within the 200 ms tolerance.")
    if abs(odom_ts - color_ts) > max_delta_ns:
        raise ValueError("Odometry message is not synchronized within the 200 ms tolerance.")

    return SynchronizedFrame(
        color_ts_ns=color_ts,
        depth_ts_ns=depth_ts,
        odom_ts_ns=odom_ts,
        color_msg=color_msg,
        depth_msg=depth_msg,
        odom_msg=odom_msg,
    )


def _internal_bbox_region(bbox: Sequence[float], margin_frac: float = 0.25) -> Tuple[int, int, int, int]:
    x1, y1, x2, y2 = [float(v) for v in bbox]
    width = max(0.0, x2 - x1)
    height = max(0.0, y2 - y1)
    ix1 = int(max(0, x1 + width * margin_frac))
    iy1 = int(max(0, y1 + height * margin_frac))
    ix2 = int(min(CAMERA_INTRINSICS["width"], x2 - width * margin_frac))
    iy2 = int(min(CAMERA_INTRINSICS["height"], y2 - height * margin_frac))
    return ix1, iy1, ix2, iy2


def median_depth_in_bbox(depth_image: np.ndarray, bbox: Sequence[float], min_valid: int = 5) -> Optional[float]:
    """Return the median of valid positive finite depth samples in the central interior region."""
    if depth_image is None or depth_image.size == 0:
        return None

    x1, y1, x2, y2 = _internal_bbox_region(bbox)
    region = depth_image[y1:y2, x1:x2]
    if region.size == 0:
        return None

    valid = region[np.isfinite(region) & (region > 0.0)]
    if valid.size < min_valid:
        return None
    return float(np.median(valid))


def pose_to_transform(x: float, y: float, yaw: float) -> np.ndarray:
    """Construct a 4x4 homogeneous transform from (x, y, yaw) in the map frame."""
    c = math.cos(yaw)
    s = math.sin(yaw)
    return np.array(
        [
            [c, -s, 0.0, x],
            [s, c, 0.0, y],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def resolve_map_pose(odom_pose: Sequence[float], frame_id: str, map_to_odom: Optional[Sequence[float]] = None) -> Tuple[float, float, float]:
    """Return the robot pose in the map frame. If odometry is not already in the map frame, apply the map-to-odom transform."""
    x, y, yaw = float(odom_pose[0]), float(odom_pose[1]), float(odom_pose[2])
    if frame_id in {"map", "world", "/map", "/world"}:
        return x, y, yaw

    if frame_id.lstrip('/') != 'odom':
        raise ValueError(f'Unsupported pose frame: {frame_id}')
    if map_to_odom is None:
        raise ValueError('An odom pose requires T_map_odom; refusing to assume identity.')
    tx, ty, theta = map_to_odom
    c, s = math.cos(theta), math.sin(theta)
    return tx + c*x - s*y, ty + s*x + c*y, math.atan2(math.sin(theta+yaw), math.cos(theta+yaw))


def backproject_pixel_to_map(
    u: float,
    v: float,
    depth_m: float,
    robot_pose: Sequence[float],
    camera_translation: Sequence[float] = (0.064, -0.065, 0.094),
    frame_id: str = "map",
    map_to_odom: Optional[Sequence[float]] = None,
) -> np.ndarray:
    """Back-project a depth pixel to the map frame using the assignment's transform chain.

    p_map = T_mb * T_bc * R_co * p_optical
    """
    if not np.isfinite(depth_m) or depth_m <= 0.0:
        raise ValueError("Depth must be positive.")

    fx = CAMERA_INTRINSICS["fx"]
    fy = CAMERA_INTRINSICS["fy"]
    cx = CAMERA_INTRINSICS["cx"]
    cy = CAMERA_INTRINSICS["cy"]

    x_optical = ((u - cx) * depth_m) / fx
    y_optical = ((v - cy) * depth_m) / fy
    z_optical = depth_m

    p_optical = np.array([x_optical, y_optical, z_optical, 1.0], dtype=np.float64)
    t_bc = np.array(camera_translation, dtype=np.float64)
    T_bc = np.eye(4, dtype=np.float64)
    T_bc[:3, 3] = t_bc
    # Apply optical-to-body rotation without a separate camera frame.
    R_co = np.eye(4, dtype=np.float64)
    R_co[:3, :3] = R_OPTICAL_TO_BODY

    map_x, map_y, map_yaw = resolve_map_pose(robot_pose, frame_id, map_to_odom)
    T_mb = pose_to_transform(map_x, map_y, map_yaw)
    map_point = T_mb @ T_bc @ R_co @ p_optical
    return map_point[:3]
