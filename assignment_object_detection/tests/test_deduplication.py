import numpy as np

from assignment_object_detection.deduplication import DetectionCluster, ObjectDeduplicator


def test_new_object_created_for_empty_candidate_set():
    dedup = ObjectDeduplicator(association_radius=0.5)
    obj = DetectionCluster(class_name="chair", map_position=np.array([0.0, 0.0], dtype=np.float32), embedding=np.ones(512, dtype=np.float32))
    merged, created = dedup.merge_detection("chair", [3.0, 3.0], np.ones(512, dtype=np.float32), 100, [obj])
    assert created is True
    assert merged.class_name == "chair"


def test_repeated_observation_updates_position_and_count():
    dedup = ObjectDeduplicator(association_radius=1.0)
    obj = DetectionCluster(class_name="chair", map_position=np.array([0.0, 0.0], dtype=np.float32), embedding=np.ones(512, dtype=np.float32), observation_count=1, first_seen_ns=0, last_seen_ns=10)
    merged, created = dedup.merge_detection("chair", [0.1, 0.1], np.ones(512, dtype=np.float32), 20, [obj])
    assert created is False
    assert merged.observation_count == 2
    assert merged.last_seen_ns == 20


def test_candidate_selection_prefers_high_similarity():
    dedup = ObjectDeduplicator(association_radius=1.0)
    vec_a = np.array([1.0] + [0.0] * 511, dtype=np.float32)
    vec_b = np.array([0.0, 1.0] + [0.0] * 510, dtype=np.float32)
    obj_a = DetectionCluster(class_name="chair", map_position=np.array([0.0, 0.0], dtype=np.float32), embedding=vec_a.copy(), observation_count=1, first_seen_ns=0, last_seen_ns=1)
    obj_b = DetectionCluster(class_name="chair", map_position=np.array([0.2, 0.0], dtype=np.float32), embedding=vec_b.copy(), observation_count=1, first_seen_ns=0, last_seen_ns=1)
    merged, created = dedup.merge_detection("chair", [0.1, 0.0], vec_a.copy(), 2, [obj_a, obj_b])
    assert created is False
    assert merged is obj_a
