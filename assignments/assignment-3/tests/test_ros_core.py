import numpy as np
from scipy.spatial.transform import Rotation
from ds685_vslam_nav.core import LocalizationBridge, OccupancyMapper, pose_matrix, StampedPairs


def test_exact_pairs_are_bounded_and_never_reuse_stale_images():
    pairs=StampedPairs(limit=3)
    assert pairs.push(0,1,'rgb') is None
    assert pairs.push(1,2,'unmatched depth') is None
    assert pairs.push(1,1,'depth')==('rgb','depth')
    for t in range(3,10): pairs.push(0,t,t)
    assert len(pairs.queues[0])==3
    assert pairs.push(1,9,'latest')==(9,'latest')
    assert all(not q for q in pairs.queues)


def test_bridge_composes_camera_extrinsic_and_odometry_without_double_rotation():
    bridge=LocalizationBridge()
    mount=pose_matrix([.07,0,.16],[0,0,0,1])
    odom=np.eye(4)
    # Wrapper camera pose has ROS body axes, not optical axes.
    correction,base=bridge.update(np.eye(4),mount,odom)
    np.testing.assert_allclose(correction,np.eye(4),atol=1e-10)
    visual=pose_matrix([1.,.2,0],Rotation.from_euler('z',.1).as_quat())
    odom=pose_matrix([.9,.1,0],[0,0,0,1])
    correction,base=bridge.update(visual,mount,odom)
    np.testing.assert_allclose(correction@odom,base,atol=1e-10)
    assert base[0,3]>.9
    assert base[2,3]==0.


def test_occupancy_rays_distinguish_free_hit_unknown_and_infinity():
    mapper=OccupancyMapper(resolution=.1,size=10)
    mapper.insert(np.eye(4),[2.],0.,.1,.1,4.)
    grid=mapper.grid(); x,y=mapper.cell(2,0); xf,yf=mapper.cell(1,0)
    assert grid[y,x]>50 and grid[yf,xf]<50
    assert grid[0,0]==-1
    mapper.insert(np.eye(4),[float('inf')],np.pi/2,.1,.1,4.)
    x,y=mapper.cell(0,4)
    assert mapper.grid()[y,x]<50


def test_occupancy_uses_transformed_sensor_origin_and_drops_nan():
    mapper=OccupancyMapper(resolution=.1,size=10)
    T=pose_matrix([1,1,0],Rotation.from_euler('z',np.pi/2).as_quat())
    mapper.insert(T,[2.,float('nan'),-.1],0.,.1,.1,4.)
    x,y=mapper.cell(1,3)
    assert mapper.grid()[y,x]>50
