import numpy as np
import pytest
from assignment_object_detection.database import DatabaseStore
from assignment_object_detection.messages import (Header,Time,Image,Odometry,Vector3,Quaternion,
    Pose,PoseWithCovariance,Twist,TwistWithCovariance)
from assignment_object_detection.synchronization import TimestampBuffer,image_array
from pathlib import Path

def test_decode_independent_ros_serialization_fixture():
    msg=Odometry.deserialize((Path(__file__).parent/'fixtures/ros_odometry.cdr').read_bytes())
    assert msg.header.stamp.sec==4 and msg.header.frame_id=='odom'
    assert msg.pose.pose.position.z==3
    assert list(msg.pose.covariance)==list(range(36))
    assert msg.twist.twist.linear.x==4 and msg.twist.twist.angular.z==.75
    assert list(msg.twist.covariance)==list(range(36,72))

def test_nearest_timestamp_out_of_order_and_reject_stale():
    b = TimestampBuffer()
    for ns in [100_000_000,30_000_000,200_000_000]:
        b.add(Image(Header(Time(1,ns),'camera'),1,1,'rgb8',0,3,[1,2,3]))
    assert b.nearest(1_040_000_000).header.stamp.nanosec == 30_000_000
    assert b.nearest(2_000_000_000) is None

def test_cdr_covariance_fixed_array_and_image_stride():
    v=Vector3(1.,2.,3.)
    msg=Odometry(Header(Time(4,5),'odom'),'base_link',
        PoseWithCovariance(Pose(v,Quaternion(0.,0.,0.,1.)),[0.]*36),
        TwistWithCovariance(Twist(v,v),[0.]*36))
    decoded=Odometry.deserialize(msg.serialize())
    assert decoded.twist.twist.angular.z == 3
    # 40-byte aligned header/frame strings + 13 doubles + 72 covariance doubles + CDR header.
    assert len(msg.serialize()) == 724
    image=Image(Header(Time(0,0),'camera'),1,1,'rgb8',0,4,[10,20,30,99])
    np.testing.assert_array_equal(image_array(image),[[[30,20,10]]])

def test_atomic_association_and_observation_history(conn):
    store=DatabaseStore(conn)
    v=np.eye(1,512,0,dtype=np.float32)[0]
    args=dict(class_name='chair',embedding=v,robot_pose=(0,0,0),camera_pose=(.064,-.065,.094,0,0,0,1),
              bbox=[0,0,10,10],confidence=.9,bearing_rad=0,range_m=1,radius=.5)
    a,_=store.ingest(position=[1,0,0],seen_at_ns=10,**args)
    b,_=store.ingest(position=[1.2,0,0],seen_at_ns=20,**args)
    assert a==b
    with conn.cursor() as cur:
        cur.execute('SELECT map_x,observation_count FROM semantic_objects WHERE object_id=%s',(a,))
        x,n=cur.fetchone()
        assert x==pytest.approx(1.1) and n==2
        cur.execute('SELECT count(*) FROM object_observations WHERE object_id=%s',(a,))
        assert cur.fetchone()[0]==2
    with pytest.raises(TypeError):
        store.ingest(position=[1.1,0,0],seen_at_ns=30,**{**args,'confidence':None})
    with conn.cursor() as cur:
        cur.execute('SELECT observation_count FROM semantic_objects WHERE object_id=%s',(a,))
        assert cur.fetchone()[0]==2

def test_database_multiple_candidates_uses_clip_similarity(conn):
    store=DatabaseStore(conn)
    a=np.eye(1,512,0,dtype=np.float32)[0];b=np.eye(1,512,1,dtype=np.float32)[0]
    store.create_object('chair',0,0,0,a,1,1)
    expected=store.create_object('chair',1,0,0,b,1,1)
    chosen,_=store.ingest('chair',[.5,0,0],b,2,(0,0,0),(.064,-.065,.094,0,0,0,1),
                          [0,0,10,10],.9,0,1,radius=.6)
    assert chosen==expected
