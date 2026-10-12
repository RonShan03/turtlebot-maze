"""Assignment-specific object detection and semantic localization modules."""

from .detector_pipeline import (
    DetectionRecord,
    accept_keyframe,
    wrapped_angle_difference,
    process_keyframe,
)
from .geometry import (
    CAMERA_INSTRINSICS,
    CAMERA_INTRINSICS,
    R_OPTICAL_TO_BODY,
    T_BODY_TO_CAMERA,
    backproject_pixel_to_map,
    median_depth_in_bbox,
    synchronize_color_depth_odometry,
)
from .database import DatabaseStore, init_database
from .deduplication import DetectionCluster, ObjectDeduplicator, merge_detection_into_objects
from .queries import (
    class_histogram_query,
    nearest_object_query,
    object_observations_query,
    region_query,
)

__all__ = [
    "DetectionRecord",
    "accept_keyframe",
    "wrapped_angle_difference",
    "process_keyframe",
    "CAMERA_INTRINSICS",
    "R_OPTICAL_TO_BODY",
    "T_BODY_TO_CAMERA",
    "backproject_pixel_to_map",
    "median_depth_in_bbox",
    "synchronize_color_depth_odometry",
    "DatabaseStore",
    "init_database",
    "DetectionCluster",
    "ObjectDeduplicator",
    "merge_detection_into_objects",
    "nearest_object_query",
    "class_histogram_query",
    "region_query",
    "object_observations_query",
]
