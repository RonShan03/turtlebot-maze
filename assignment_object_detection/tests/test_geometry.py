import numpy as np
import math
import pytest

from assignment_object_detection.geometry import backproject_pixel_to_map, median_depth_in_bbox, synchronize_color_depth_odometry


class MockHeader:
    def __init__(self, sec, nanosec):
        self.stamp = MockStamp(sec, nanosec)


class MockStamp:
    def __init__(self, sec, nanosec):
        self.sec = sec
        self.nanosec = nanosec


class MockMsg:
    def __init__(self, sec=0, nanosec=0):
        self.header = MockHeader(sec, nanosec)


def test_sync_accepts_tightly_aligned_messages():
    color = MockMsg(sec=10, nanosec=0)
    depth = MockMsg(sec=10, nanosec=50_000_000)
    odom = MockMsg(sec=10, nanosec=100_000_000)
    synced = synchronize_color_depth_odometry(color, depth, odom, max_delta_ns=200_000_000)
    assert synced.color_ts_ns == 10_000_000_000
    assert synced.depth_ts_ns == 10_050_000_000
    assert synced.odom_ts_ns == 10_100_000_000


def test_median_depth_in_bbox_uses_valid_depth_region():
    depth = np.zeros((240, 320), dtype=np.float32)
    depth[100:140, 100:200] = 1.0
    depth[110:130, 130:170] = 2.0
    depth[115:120, 140:150] = 0.0
    depth[120:130, 145:155] = 10.0
    bbox = [90.0, 90.0, 210.0, 180.0]
    value = median_depth_in_bbox(depth, bbox, min_valid=5)
    assert value is not None
    assert value == 1.0


def test_backproject_pixel_to_map_known_result():
    u = 150.0
    v = 90.0
    depth = 2.0
    robot_pose = (1.0, 2.0, 0.0)
    point = backproject_pixel_to_map(u, v, depth, robot_pose)
    # Optical=(-20/277.13,-60/277.13,2); body=(2,20/277.13,60/277.13).
    # Add camera translation, then robot translation (1,2,0).
    expected = [3.064, 2.007168296467362, 0.31050488940208565]
    np.testing.assert_allclose(point, expected, atol=1e-9, rtol=0)

def test_rotated_map_to_odom_chain():
    point = backproject_pixel_to_map(150,90,2,(1,0,0),frame_id='odom',
                                    map_to_odom=(10,20,math.pi/2))
    np.testing.assert_allclose(point,[9.992831703532638,23.064,.31050488940208565],atol=1e-9,rtol=0)

def test_odom_without_transform_is_rejected():
    with pytest.raises(ValueError,match='requires'):
        backproject_pixel_to_map(150,90,2,(1,0,0),frame_id='odom')
