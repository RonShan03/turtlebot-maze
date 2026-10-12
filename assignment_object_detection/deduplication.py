"""Same-class object association and deduplication."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Iterable, List, Optional, Sequence, Tuple

import numpy as np


@dataclass
class DetectionCluster:
    class_name: str
    map_position: np.ndarray
    embedding: np.ndarray
    observation_count: int = 1
    first_seen_ns: int = 0
    last_seen_ns: int = 0

    def normalized_embedding(self) -> np.ndarray:
        emb = np.asarray(self.embedding, dtype=np.float32)
        norm = np.linalg.norm(emb)
        if norm <= 0.0:
            return emb
        return emb / norm


class ObjectDeduplicator:
    def __init__(self, association_radius: float = 0.5):
        self.association_radius = float(association_radius)

    def candidate_objects(
        self,
        class_name: str,
        map_position: Sequence[float],
        objects: Sequence[DetectionCluster],
    ) -> List[DetectionCluster]:
        position = np.asarray(map_position, dtype=np.float32)
        candidates: List[DetectionCluster] = []
        for obj in objects:
            if obj.class_name != class_name:
                continue
            dist = float(np.linalg.norm(obj.map_position - position))
            if dist <= self.association_radius:
                candidates.append(obj)
        return candidates

    def select_best_candidate(
        self,
        class_name: str,
        map_position: Sequence[float],
        objects: Sequence[DetectionCluster],
        embedding: Sequence[float],
    ) -> Optional[DetectionCluster]:
        candidates = self.candidate_objects(class_name, map_position, objects)
        if not candidates:
            return None
        if len(candidates) == 1:
            return candidates[0]

        vec = np.asarray(embedding, dtype=np.float32)
        vec = vec / (np.linalg.norm(vec) + 1e-12)
        best = candidates[0]
        best_score = float(np.dot(best.normalized_embedding(), vec))
        for candidate in candidates[1:]:
            score = float(np.dot(candidate.normalized_embedding(), vec))
            if score > best_score:
                best = candidate
                best_score = score
        return best

    def merge_detection(
        self,
        class_name: str,
        map_position: Sequence[float],
        embedding: Sequence[float],
        seen_at_ns: int,
        objects: List[DetectionCluster],
    ) -> Tuple[Optional[DetectionCluster], bool]:
        position = np.asarray(map_position, dtype=np.float32)
        vec = np.asarray(embedding, dtype=np.float32)
        vec = vec / (np.linalg.norm(vec) + 1e-12)

        candidate = self.select_best_candidate(class_name, position, objects, vec)
        if candidate is None:
            new_obj = DetectionCluster(
                class_name=class_name,
                map_position=np.asarray(position, dtype=np.float32),
                embedding=vec.copy(),
                observation_count=1,
                first_seen_ns=seen_at_ns,
                last_seen_ns=seen_at_ns,
            )
            return new_obj, True

        n = candidate.observation_count
        merged_position = (n * candidate.map_position + position) / (n + 1)
        merged_embedding = (n * candidate.normalized_embedding() + vec) / (n + 1)
        merged_embedding = merged_embedding / (np.linalg.norm(merged_embedding) + 1e-12)
        candidate.map_position = merged_position
        candidate.embedding = merged_embedding
        candidate.observation_count = n + 1
        candidate.last_seen_ns = max(candidate.last_seen_ns, seen_at_ns)
        candidate.first_seen_ns = min(candidate.first_seen_ns, seen_at_ns)
        return candidate, False


def merge_detection_into_objects(
    objects: List[DetectionCluster],
    class_name: str,
    map_position: Sequence[float],
    embedding: Sequence[float],
    seen_at_ns: int,
    association_radius: float = 0.5,
) -> Tuple[List[DetectionCluster], DetectionCluster, bool]:
    deduplicator = ObjectDeduplicator(association_radius=association_radius)
    merged, created_new = deduplicator.merge_detection(class_name, map_position, embedding, seen_at_ns, objects)
    if created_new:
        objects.append(merged)
        return objects, merged, True
    return objects, merged, False
