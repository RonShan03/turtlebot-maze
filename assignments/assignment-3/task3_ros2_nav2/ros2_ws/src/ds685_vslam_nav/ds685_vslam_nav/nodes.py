import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from .core import pose_matrix, LocalizationBridge, OccupancyMapper


def matrix_tf(msg):
    t,q=msg.transform.translation,msg.transform.rotation
    return pose_matrix([t.x,t.y,t.z],[q.x,q.y,q.z,q.w])


class PoseBridge(Node):
    def __init__(self):
        super().__init__('vslam_pose_bridge')
        self.buffer=Buffer(); self.listener=TransformListener(self.buffer,self)
        self.broadcaster=TransformBroadcaster(self); self.bridge=LocalizationBridge()
        self.pose=self.create_publisher(Odometry,'/vslam/base_pose',10)
        self.last_stamp=None
        self.subscription=self.create_subscription(Odometry,'/run_slam/camera_pose',self.receive,10)
        self.declare_parameter('log_path','/work/assignments/assignment-3/results/ros2/live/poses.jsonl')
        self.declare_parameter('jump_limit_m',2.0)
        self.last_base=None

    def receive(self,msg):
        try:
            stamp=rclpy.time.Time.from_msg(msg.header.stamp)
            if self.last_stamp is not None and stamp.nanoseconds<=self.last_stamp: return
            T_base_camera=matrix_tf(self.buffer.lookup_transform('base_link',msg.child_frame_id,stamp))
            T_odom_base=matrix_tf(self.buffer.lookup_transform('odom','base_link',stamp))
            p,q=msg.pose.pose.position,msg.pose.pose.orientation
            T_slam_camera=pose_matrix([p.x,p.y,p.z],[q.x,q.y,q.z,q.w])
            T_map_odom,T_map_base=self.bridge.update(T_slam_camera,T_base_camera,T_odom_base)
            jump=self.last_base is not None and np.linalg.norm(T_map_base[:2,3]-self.last_base[:2,3])>self.get_parameter('jump_limit_m').value
            if jump:
                self.get_logger().error('Visual pose jump: stopping TF updates; stop Nav2, clear/rebuild occupancy before resuming')
                return
            tr=TransformStamped(); tr.header=msg.header; tr.header.frame_id='map'; tr.child_frame_id='odom'
            # Small future tolerance covers interpolation between camera updates.
            tr.header.stamp=(stamp+rclpy.duration.Duration(seconds=.2)).to_msg()
            tr.transform.translation.x,tr.transform.translation.y=map(float,T_map_odom[:2,3])
            rot=Rotation.from_matrix(T_map_odom[:3,:3]).as_quat()
            tr.transform.rotation.x,tr.transform.rotation.y,tr.transform.rotation.z,tr.transform.rotation.w=map(float,rot)
            self.broadcaster.sendTransform(tr)
            out=Odometry(); out.header=msg.header; out.header.frame_id='map'; out.child_frame_id='base_link'
            out.pose.pose.position.x,out.pose.pose.position.y=map(float,T_map_base[:2,3])
            qbase=Rotation.from_matrix(T_map_base[:3,:3]).as_quat()
            out.pose.pose.orientation.x,out.pose.pose.orientation.y,out.pose.pose.orientation.z,out.pose.pose.orientation.w=map(float,qbase)
            # Conservative configured uncertainty, not a learned/calibrated covariance.
            out.pose.covariance=[.05 if i in (0,7,35) else (1e3 if i in (14,21,28) else 0.) for i in range(36)]
            self.pose.publish(out); self.last_stamp=stamp.nanoseconds; self.last_base=T_map_base
            path=Path(self.get_parameter('log_path').value); path.parent.mkdir(parents=True,exist_ok=True)
            with path.open('a') as f: f.write(json.dumps(dict(stamp=stamp.nanoseconds/1e9,T_map_base=T_map_base.tolist()))+'\n')
        except Exception as exc:
            self.get_logger().warning(f'Visual pose waiting for valid synchronized TF: {exc}',throttle_duration_sec=3)


class Mapper(Node):
    def __init__(self):
        super().__init__('range_occupancy_mapper')
        self.mapper=OccupancyMapper(); self.buffer=Buffer(); self.listener=TransformListener(self.buffer,self)
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.publisher=self.create_publisher(OccupancyGrid,'/map',qos)
        self.subscription=self.create_subscription(LaserScan,'/scan',self.receive,qos_profile_sensor_data)
        self.timer=self.create_timer(1.,self.publish)
        self.scans=0

    def receive(self,msg):
        try:
            T=matrix_tf(self.buffer.lookup_transform('map',msg.header.frame_id,rclpy.time.Time.from_msg(msg.header.stamp)))
        except Exception:
            return
        self.mapper.insert(T,msg.ranges,msg.angle_min,msg.angle_increment,msg.range_min,min(msg.range_max,10.))
        self.scans+=1

    def publish(self):
        if self.scans==0: return
        grid=OccupancyGrid(); grid.header.stamp=self.get_clock().now().to_msg(); grid.header.frame_id='map'
        grid.info.resolution=self.mapper.resolution; grid.info.width=grid.info.height=self.mapper.width
        grid.info.origin.position.x=grid.info.origin.position.y=self.mapper.origin; grid.info.origin.orientation.w=1.
        grid.data=self.mapper.grid().ravel().tolist(); self.publisher.publish(grid)


def spin(cls):
    rclpy.init(); node=cls()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.try_shutdown()


def pose_bridge_main(): spin(PoseBridge)
def mapper_main(): spin(Mapper)
