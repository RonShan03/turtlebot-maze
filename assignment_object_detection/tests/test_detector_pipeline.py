import numpy as np
import pytest

from assignment_object_detection.detector_pipeline import DetectionRecord, accept_keyframe, wrapped_angle_difference


def test_wrapped_angle_difference():
    assert wrapped_angle_difference(0.1, 6.2) == pytest.approx(0.18318530717958595)
    assert wrapped_angle_difference(3.0, -3.0) == pytest.approx(0.28318530717958645)


def test_accept_keyframe_initial_frame():
    assert accept_keyframe((0.0, 0.0, 0.0), None) is True


def test_accept_keyframe_translation_threshold():
    prev = (0.0, 0.0, 0.0)
    current = (0.6, 0.0, 0.0)
    assert accept_keyframe(current, prev, translation_threshold=0.5, rotation_threshold_deg=15.0) is True


def test_accept_keyframe_rotation_threshold():
    prev = (0.0, 0.0, 0.0)
    current = (0.0, 0.0, 0.3)
    assert accept_keyframe(current, prev, translation_threshold=0.5, rotation_threshold_deg=0.1) is True


def test_detection_record_serializes_embedding():
    det = DetectionRecord(class_name="chair", confidence=0.82, bbox=[1.0, 2.0, 3.0, 4.0], embedding=np.ones(3))
    payload = det.as_dict()
    assert payload["class"] == "chair"
    assert payload["confidence"] == 0.82
    assert "embedding" in payload

def test_inference_uses_one_clip_batch_and_normalizes_embeddings():
    import torch
    from types import SimpleNamespace
    from assignment_object_detection.detector_pipeline import process_keyframe
    calls=[]
    class Detector:
        def predict(self,frame,conf,verbose):
            assert conf==.3
            return [SimpleNamespace(boxes=[SimpleNamespace(cls=[0],conf=[.9],
                xyxy=torch.tensor([[0.,0.,5.,5.]])),SimpleNamespace(cls=[1],conf=[.8],
                xyxy=torch.tensor([[5.,5.,10.,10.]]))])]
    class Clip:
        def encode_image(self,batch):
            calls.append(tuple(batch.shape))
            return torch.full((batch.shape[0],512),2.)
    records=process_keyframe(np.zeros((10,10,3),dtype=np.uint8),Detector(),clip_model=Clip(),
        clip_preprocess=lambda image:torch.zeros(3,4,4),device=torch.device('cpu'),class_names=['chair','bed'])
    assert calls==[(2,3,4,4)]
    assert [r.class_name for r in records]==['chair','bed']
    for record in records:
        assert record.embedding.shape==(512,)
        assert np.linalg.norm(record.embedding)==pytest.approx(1)

def test_stationary_or_missing_odometry_rejects_keyframe():
    assert not accept_keyframe(None,None)
    assert not accept_keyframe((1,2,.1),(1,2,.1))
