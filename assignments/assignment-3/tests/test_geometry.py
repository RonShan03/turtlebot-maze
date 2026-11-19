import cv2
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from task1_python_vslam.geometry import transform, inverse, project, relative_pose, triangulate, pnp_pose
from task1_python_vslam.evaluation import align_poses, metrics

K = np.array([[500.,0,320],[0,500,240],[0,0,1]])


def scene():
    rng = np.random.default_rng(5)
    X = rng.uniform([-3,-2,5],[3,2,14], (200,3))
    T = transform(Rotation.from_euler("y", .04).as_matrix(), [1., .03, .2])
    p1, _ = project(K, np.eye(4), X)
    p2, _ = project(K, T, X)
    return X, T, p1, p2


def test_chain_is_camera_to_world():
    _, T, p1, p2 = scene()
    relative, valid = relative_pose(p1, p2, K)
    relative[:3,3] *= np.linalg.norm(T[:3,3])
    assert valid.sum() > 180
    np.testing.assert_allclose(inverse(relative), T, atol=1e-4)
    np.testing.assert_allclose(T @ inverse(T), np.eye(4), atol=1e-12)


def test_triangulation_and_reprojection_rejection():
    X, T, p1, p2 = scene()
    recovered, valid, _ = triangulate(K, np.eye(4), T, p1, p2)
    assert valid.all()
    np.testing.assert_allclose(recovered, X, atol=1e-7)
    wrong = p2.copy()
    wrong[:20,1] += 50
    _, valid, _ = triangulate(K, np.eye(4), T, p1, wrong)
    assert not valid[:20].any()


def test_zero_baseline_and_points_behind_camera_rejected():
    X, T, p1, p2 = scene()
    _, valid, _ = triangulate(K, np.eye(4), np.eye(4), p1, p1)
    assert not valid.any()
    behind = X.copy()
    behind[:,2] *= -1
    a, _ = project(K, np.eye(4), behind)
    b, _ = project(K, T, behind)
    _, valid, _ = triangulate(K, np.eye(4), T, a, b)
    assert not valid.any()


def test_pnp_rejects_outliers_and_recovers_metric_map_pose():
    X, T, _, p2 = scene()
    p2[:30] = np.random.default_rng(6).uniform([0,0],[640,480],(30,2))
    pose, valid = pnp_pose(X, p2, K)
    np.testing.assert_allclose(pose, T, atol=1e-5)
    assert not valid[:30].any()


def test_essential_ransac_outliers():
    _, T, p1, p2 = scene()
    p2[:40] = np.random.default_rng(7).uniform([0,0],[640,480],(40,2))
    pose, valid = relative_pose(p1, p2, K)
    np.testing.assert_allclose(pose[:3,:3], T[:3,:3].T, atol=.02)
    assert valid[:40].sum() < 5


def test_sim3_alignment_and_rpe():
    rng = np.random.default_rng(9)
    gt = np.repeat(np.eye(4)[None], 25, axis=0)
    gt[:,:3,3] = rng.normal(size=(25,3))
    R = Rotation.from_euler("xyz", [.1,.2,.3]).as_matrix()
    est = gt.copy()
    est[:,:3,:3] = R.T
    est[:,:3,3] = (gt[:,:3,3] - [3,2,1]) @ R / 4
    aligned, model = align_poses(est, gt)
    assert model["scale"] == pytest.approx(4)
    np.testing.assert_allclose(aligned, gt, atol=1e-10)
    scores = metrics(aligned, gt)
    assert scores["ate_rmse_m"] < 1e-10
    assert scores["rpe_translation_rmse_m"] < 1e-10
    assert scores["rpe_rotation_rmse_deg"] < 1e-5


def test_se3_preserves_metric_scale():
    poses = np.repeat(np.eye(4)[None], 5, axis=0)
    poses[:,0,3] = np.arange(5)
    truth = poses.copy()
    truth[:,:3,3] += [2,3,4]
    aligned, model = align_poses(poses, truth, with_scale=False)
    assert model["scale"] == 1
    np.testing.assert_allclose(aligned, truth, atol=1e-10)


def test_rpe_does_not_bridge_tracking_loss():
    poses = np.repeat(np.eye(4)[None], 6, axis=0)
    poses[:,0,3] = np.arange(6)
    scores = metrics(poses, poses, delta=2, valid=[True,True,False,True,True,True])
    assert scores["rpe_pairs"] == 1
    with pytest.raises(ValueError):
        align_poses(np.repeat(np.eye(4)[None],3,axis=0), np.repeat(np.eye(4)[None],3,axis=0))
