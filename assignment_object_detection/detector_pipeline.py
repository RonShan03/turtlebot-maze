"""YOLOv8 + CLIP detection pipeline for the assignment."""

from __future__ import annotations

import base64
import math
from dataclasses import dataclass
from typing import Any, Iterable, List, Optional, Sequence, Tuple

import numpy as np
import torch
from PIL import Image as PILImage


@dataclass
class DetectionRecord:
    class_name: str
    confidence: float
    bbox: List[float]
    embedding: Optional[np.ndarray] = None

    def as_dict(self) -> dict:
        payload: dict[str, Any] = {
            "class": self.class_name,
            "confidence": round(float(self.confidence), 3),
            "bbox": [float(v) for v in self.bbox],
        }
        if self.embedding is not None:
            emb = np.asarray(self.embedding, dtype=np.float32)
            payload["embedding"] = base64.b64encode(emb.tobytes()).decode("ascii")
            payload["embedding_dim"] = int(emb.shape[0])
        return payload


def wrapped_angle_difference(a: float, b: float) -> float:
    """Return the wrapped angular difference in radians."""
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def accept_keyframe(
    current_pose: Optional[Tuple[float, float, float]],
    previous_pose: Optional[Tuple[float, float, float]],
    translation_threshold: float = 0.5,
    rotation_threshold_deg: float = 15.0,
) -> bool:
    """Accept a frame as a new keyframe when the robot has moved enough."""
    if current_pose is None:
        return False
    if previous_pose is None:
        return True

    old_x, old_y, old_yaw = previous_pose
    x, y, yaw = current_pose
    translation = math.hypot(x - old_x, y - old_y)
    rotation = wrapped_angle_difference(yaw, old_yaw)
    return translation >= translation_threshold or rotation >= math.radians(rotation_threshold_deg)


def _normalize_embedding(vec: np.ndarray) -> np.ndarray:
    emb = np.asarray(vec, dtype=np.float32)
    norm = np.linalg.norm(emb)
    if norm <= 0.0:
        return emb
    return emb / norm


def _extract_valid_crop(frame: np.ndarray, bbox: Sequence[float]) -> Optional[np.ndarray]:
    x1, y1, x2, y2 = [int(round(v)) for v in bbox]
    x1 = max(0, x1)
    y1 = max(0, y1)
    x2 = min(frame.shape[1], x2)
    y2 = min(frame.shape[0], y2)
    if x2 <= x1 or y2 <= y1:
        return None
    crop = frame[y1:y2, x1:x2]
    if crop.size == 0:
        return None
    return crop


def _clip_batch_embeddings(crops: Sequence[np.ndarray], model: Any, preprocess: Any, device: torch.device) -> List[np.ndarray]:
    if not crops:
        return []

    transformed = [preprocess(PILImage.fromarray(np.asarray(crop[:, :, ::-1], dtype=np.uint8))) for crop in crops]
    batch = torch.stack(transformed).to(device)
    with torch.no_grad():
        encoded = model.encode_image(batch)
    encoded = encoded.cpu().numpy().astype(np.float32)
    normalized = np.asarray([_normalize_embedding(vec) for vec in encoded], dtype=np.float32)
    return [emb for emb in normalized]


def process_keyframe(
    frame: np.ndarray,
    model: Any,
    confidence_threshold: float = 0.3,
    clip_model: Optional[Any] = None,
    clip_preprocess: Optional[Any] = None,
    device: Optional[torch.device] = None,
    class_names: Optional[Sequence[str]] = None,
) -> List[DetectionRecord]:
    """Process a single RGB frame and return per-detection objects."""
    if model is None:
        raise ValueError("A YOLO model instance is required.")

    results = model.predict(frame, conf=confidence_threshold, verbose=False)
    detections: List[DetectionRecord] = []
    crop_list: List[np.ndarray] = []

    for result in results:
        for box in result.boxes:
            cls_id = int(box.cls[0])
            raw_bbox = [float(v) for v in box.xyxy[0].tolist()]
            class_name = class_names[cls_id] if class_names is not None and cls_id < len(class_names) else str(cls_id)
            detection = DetectionRecord(class_name=class_name, confidence=float(box.conf[0]), bbox=raw_bbox)
            crop = _extract_valid_crop(frame, raw_bbox)
            if crop is not None:
                crop_list.append(crop)
            else:
                crop_list.append(None)
            detections.append(detection)

    if clip_model is not None and clip_preprocess is not None:
        valid_crops = [crop for crop in crop_list if crop is not None]
        if valid_crops:
            emb_device = device or torch.device("cuda" if torch.cuda.is_available() else "cpu")
            embeddings = _clip_batch_embeddings(valid_crops, clip_model, clip_preprocess, emb_device)
            idx = 0
            for i, det in enumerate(detections):
                if crop_list[i] is not None:
                    det.embedding = embeddings[idx]
                    idx += 1

    return detections
