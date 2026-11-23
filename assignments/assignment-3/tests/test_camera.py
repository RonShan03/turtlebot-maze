import json
import cv2
import numpy as np
import pytest
from task2_stella_vslam.camera import stella_config, inspect_video, calibrate


def fixture_calibration():
    return dict(K=[[100.,0,80],[0,100.,60],[0,0,1]],distortion=[0.]*5,image_size=[160,120])


def test_yaml_uses_perspective_and_measured_values():
    config=stella_config(fixture_calibration(),20.)
    assert config['Camera']['model']=='perspective'
    assert config['Camera']['fx']==100.
    with pytest.raises(ValueError): stella_config(fixture_calibration(),0.)


def test_video_validation_resizing_and_derived_calibration(tmp_path):
    source=tmp_path/'test.avi'
    writer=cv2.VideoWriter(str(source),cv2.VideoWriter_fourcc(*'MJPG'),20.,(160,120))
    for i in range(8): writer.write(np.random.default_rng(i).integers(0,255,(120,160,3),dtype=np.uint8))
    writer.release()
    output=tmp_path/'small.avi'
    report=inspect_video(source,calibration=fixture_calibration(),output_video=output,scale=.5)
    assert report['decoded_frames']==8
    adjusted=json.loads((tmp_path/'small.avi.calibration.json').read_text())
    assert adjusted['K'][0][0]==50.
    assert adjusted['image_size']==[80,60]
    invalid=fixture_calibration(); invalid['image_size']=[320,240]
    with pytest.raises(ValueError,match='resolution'): inspect_video(source,calibration=invalid)


def test_calibration_refuses_unavailable_chessboards(tmp_path):
    cv2.imwrite(str(tmp_path/'blank.png'),np.zeros((480,640),np.uint8))
    with pytest.raises(ValueError,match='usable chessboards'): calibrate(tmp_path,(9,6),.025,tmp_path/'calibration.json')


@pytest.mark.parametrize("column_shape", [False, True])
def test_per_view_rms_compares_corresponding_corners(tmp_path, monkeypatch, column_shape):
    cv2.imwrite(str(tmp_path / "board.png"), np.zeros((120, 160), np.uint8))
    intrinsic = np.array(fixture_calibration()["K"])
    distortion = np.zeros(5)
    rotation = np.zeros(3)
    translation = np.array([0., 0., 1.])
    points = np.zeros((9, 3), dtype=np.float32)
    points[:, :2] = np.mgrid[:3, :3].T.reshape(-1, 2) * .025
    projected, _ = cv2.projectPoints(points, rotation, translation, intrinsic, distortion)
    corners = (projected.reshape(-1, 2) + [3, 4]).astype(np.float32)
    if column_shape:
        corners = corners.reshape(-1, 1, 2)
    monkeypatch.setattr(cv2, "findChessboardCornersSB", lambda *args, **kwargs: (True, corners))
    monkeypatch.setattr(cv2, "calibrateCamera", lambda *args: (
        5., intrinsic, distortion, [rotation], [translation],
    ))

    report = calibrate(tmp_path, (3, 3), .025, tmp_path / "calibration.json", min_views=1)

    assert report["per_view_rms_px"] == pytest.approx([5.])
