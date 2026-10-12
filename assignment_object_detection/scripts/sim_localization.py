"""Simulation-only localization: T_map_odom = T_map_base_truth * inv(T_odom_base).

Run with the SDF ground-truth plugin and after disabling AMCL tf_broadcast. This uses
Gazebo's world pose as a controlled localization baseline; it is not real SLAM.
"""
import json
import math
from pathlib import Path
import argparse
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def stamp(m): return m.header.stamp.sec*1_000_000_000+m.header.stamp.nanosec

class Localization(Node):
    def __init__(self,output):
        super().__init__('ds685_sim_localization')
        self.odom=[];self.truth=[];self.tf=TransformBroadcaster(self)
        self.output=output.open('w')
        self.create_subscription(Odometry,'/odom',self.on_odom,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/ground_truth/odom',self.on_truth,qos_profile_sensor_data)
    def on_odom(self,msg):
        self.odom.append(msg);self.odom=self.odom[-60:];self.process()
    def on_truth(self,msg):
        self.truth.append(msg);self.truth=self.truth[-60:];self.process()
    def process(self):
        if not self.odom or not self.truth:return
        g=self.truth[-1];o=min(self.odom,key=lambda m:abs(stamp(m)-stamp(g)))
        if abs(stamp(o)-stamp(g))>50_000_000:return
        gp,op=g.pose.pose,o.pose.pose
        theta=yaw(gp.orientation)-yaw(op.orientation)
        c,s=math.cos(theta),math.sin(theta)
        tx=gp.position.x-(c*op.position.x-s*op.position.y)
        ty=gp.position.y-(s*op.position.x+c*op.position.y)
        tf=TransformStamped();tf.header.stamp=g.header.stamp
        tf.header.frame_id='map';tf.child_frame_id='odom'
        tf.transform.translation.x=tx;tf.transform.translation.y=ty
        tf.transform.rotation.z=math.sin(theta/2);tf.transform.rotation.w=math.cos(theta/2)
        self.tf.sendTransform(tf)
        self.output.write(json.dumps(dict(stamp_ns=stamp(g),ground_truth=[gp.position.x,gp.position.y,yaw(gp.orientation)],
                        odom=[op.position.x,op.position.y,yaw(op.orientation)],map_to_odom=[tx,ty,theta]))+'\n')
        self.output.flush()

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();a.output.parent.mkdir(parents=True,exist_ok=True)
    rclpy.init();node=Localization(a.output)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.output.close();node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
