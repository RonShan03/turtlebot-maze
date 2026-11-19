import numpy as np
import pytest
from task1_python_vslam.dataset import KittiSequence, load_poses, save_poses
from task1_python_vslam.pipeline import VisualOdometry
from task1_python_vslam.features import match_descriptors
from task1_python_vslam.synthetic import generate


def test_loader_roundtrip_and_timestamps(tmp_path):
    generate(tmp_path, frames=3)
    seq = KittiSequence(tmp_path, max_frames=2)
    assert len(list(seq)) == 2
    assert seq.ground_truth.shape == (2,4,4)
    save_poses(tmp_path / "copy.txt", seq.ground_truth)
    np.testing.assert_allclose(load_poses(tmp_path / "copy.txt"), seq.ground_truth, atol=1e-9)
    (tmp_path / "sequences/00/times.txt").write_text("0\n0\n0\n")
    with pytest.raises(ValueError, match="timestamps"):
        KittiSequence(tmp_path)


def test_blank_frames_fail_without_crashing():
    vo = VisualOdometry(np.array([[500.,0,320],[0,500,240],[0,0,1]]))
    for i in range(3):
        pose, row, points = vo.process(i, i*.1, np.zeros((480,640),np.uint8))
        assert row["status"] == "lost"
        np.testing.assert_array_equal(pose, np.eye(4))
    assert match_descriptors(None, None)[0].shape == (0,2)


def test_rendered_images_exercise_orb_essential_and_pnp(tmp_path):
    generate(tmp_path, frames=12)
    seq = KittiSequence(tmp_path)
    vo = VisualOdometry(seq.K)
    records = [vo.process(i,t,im)[1] for i,t,im in seq]
    assert any(r["status"] == "essential_init" for r in records)
    assert sum(r["status"] == "pnp" for r in records) >= 6
    assert sum(r["new_landmarks"] for r in records) > 30
    # Tracking loss keeps the last valid map and can recover to the same gauge.
    pose, row, _ = vo.process(12,1.2,np.zeros((480,640),np.uint8))
    assert row["status"] == "lost"
    np.testing.assert_allclose(pose, vo.previous.pose)
    _, _, last_image = list(seq)[-1]
    _, recovered, _ = vo.process(13,1.3,last_image)
    assert recovered["status"] == "pnp"
