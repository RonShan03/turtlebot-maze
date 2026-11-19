"""Bridge sensor QoS to the wrapper, preserve synchronized stamps, derive YAML from CameraInfo."""
from pathlib import Path
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,CameraInfo
from .core import StampedPairs


class Adapter(Node):
    def __init__(self):
        super().__init__('vslam_camera_adapter')
        self.declare_parameter('config_path','/work/assignments/assignment-3/data/sim-camera.yaml')
        self.declare_parameter('fps',15.)
        self.info=None; self.pairs=0
        self.color_pub=self.create_publisher(Image,'/vslam/color/image_raw',10)
        self.depth_pub=self.create_publisher(Image,'/vslam/depth/image_raw',10)
        self.info_sub=self.create_subscription(CameraInfo,'/camera/color/camera_info',self.receive_info,qos_profile_sensor_data)
        self.sync=StampedPairs(); self.raw_counts=[0,0]
        self.color=self.create_subscription(Image,'/camera/color/image_raw',lambda m:self.raw(0,m),qos_profile_sensor_data)
        self.depth=self.create_subscription(Image,'/camera/depth/image_rect_raw',lambda m:self.raw(1,m),qos_profile_sensor_data)
        self.timer=self.create_timer(5.,self.report)

    def raw(self,side,msg):
        self.raw_counts[side]+=1
        stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        pair=self.sync.push(side,stamp,msg)
        if pair is not None: self.receive(*pair)

    def report(self):
        self.get_logger().info(f'RGBD pairs={self.pairs}, raw={self.raw_counts}, calibration={self.info is not None}, queues={[len(q) for q in self.sync.queues]}')

    def receive_info(self,msg):
        if self.info is not None: return
        K=np.array(msg.k).reshape(3,3)
        if msg.distortion_model not in ('plumb_bob','') or K[0,0]<=0: return
        D=list(msg.d)+[0.]*5
        camera=dict(name='Gazebo CameraInfo RGBD',setup='RGBD',model='perspective',cols=msg.width,rows=msg.height,
                    fx=float(K[0,0]),fy=float(K[1,1]),cx=float(K[0,2]),cy=float(K[1,2]),
                    k1=float(D[0]),k2=float(D[1]),p1=float(D[2]),p2=float(D[3]),k3=float(D[4]),
                    fps=self.get_parameter('fps').value,color_order='RGB',focal_x_baseline=float(K[0,0])*.05,depth_threshold=40.)
        config=dict(Camera=camera,Preprocessing=dict(min_size=200,depthmap_factor=1.),
                    Feature=dict(name='ORB',scale_factor=1.2,num_levels=8,ini_fast_threshold=12,min_fast_threshold=5),
                    Mapping=dict(backend='g2o',baseline_dist_thr=.05),Tracking=dict(backend='g2o'),
                    System=dict(map_format='msgpack'))
        path=Path(self.get_parameter('config_path').value); path.parent.mkdir(parents=True,exist_ok=True)
        temporary=path.with_suffix('.tmp'); temporary.write_text(yaml.safe_dump(config)); temporary.replace(path)
        self.info=msg
        self.get_logger().info(f'CameraInfo validated; wrote {path}')

    def receive(self,color,depth):
        if self.info is None: return
        if color.width!=self.info.width or color.height!=self.info.height or (color.width,color.height)!=(depth.width,depth.height):
            self.get_logger().error('RGB-D/calibration resolution mismatch'); return
        if depth.encoding!='32FC1' or color.encoding!='rgb8':
            self.get_logger().error('This simulator adapter requires rgb8 and 32FC1 metres'); return
        # Gz RGB-D images are aligned and carry the same original timestamp.
        self.color_pub.publish(color); self.depth_pub.publish(depth); self.pairs+=1


def main():
    rclpy.init(); node=Adapter()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.try_shutdown()


if __name__=='__main__': main()
