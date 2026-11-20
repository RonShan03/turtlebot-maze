"""Bounded ROS connectivity and TF probe; saves real sensor images, never mock screenshots."""
import argparse
import json
import sys
from pathlib import Path
import time
import cv2
import numpy as np
import rclpy
from rclpy.utilities import remove_ros_args
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo, LaserScan, Imu
from nav_msgs.msg import Odometry, OccupancyGrid
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--output",required=True); parser.add_argument("--seconds",type=float,default=15)
    parser.add_argument("--require-navigation", action="store_true",
                        help="exit nonzero without sustained visual poses, map TF and costmaps")
    args=parser.parse_args(remove_ros_args()[1:])
    output=Path(args.output); output.mkdir(parents=True,exist_ok=True)
    rclpy.init()
    node=rclpy.create_node("ds685_smoke")
    bridge=CvBridge(); buffer=Buffer(); listener=TransformListener(buffer,node)
    counts={}; details={}; subscriptions=[]
    visual_stamps = set()
    def receive(topic,msg):
        counts[topic]=counts.get(topic,0)+1
        if topic == "/run_slam/camera_pose":
            visual_stamps.add((msg.header.stamp.sec, msg.header.stamp.nanosec))
        if topic not in details:
            info=dict(frame=msg.header.frame_id, stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9)
            if isinstance(msg,Image):
                info.update(width=msg.width,height=msg.height,encoding=msg.encoding)
                image=bridge.imgmsg_to_cv2(msg,desired_encoding="passthrough" if "depth" in topic else "bgr8")
                if "depth" in topic:
                    depth=np.nan_to_num(image,nan=0,posinf=0,neginf=0)
                    info.update(finite_fraction=float(np.isfinite(image).mean()), positive_fraction=float((depth>0).mean()),max_depth=float(depth.max()))
                    image=cv2.applyColorMap(np.uint8(np.clip(depth,0,5)*51),cv2.COLORMAP_TURBO)
                cv2.imwrite(str(output/("depth.png" if "depth" in topic else "camera.png")),image)
            elif isinstance(msg,CameraInfo): info.update(K=list(msg.k),D=list(msg.d),width=msg.width,height=msg.height)
            elif isinstance(msg,LaserScan): info.update(rays=len(msg.ranges),range_max=msg.range_max)
            elif isinstance(msg,OccupancyGrid):
                info.update(width=msg.info.width,height=msg.info.height,resolution=msg.info.resolution,known_cells=sum(v>=0 for v in msg.data))
                cells=np.array(msg.data).reshape(msg.info.height,msg.info.width)
                picture=np.full(cells.shape,127,np.uint8); picture[cells==0]=255; picture[cells>50]=0
                cv2.imwrite(str(output/(topic.strip('/').replace('/','_')+'.png')),np.flipud(picture))
            details[topic]=info
    topics=[("/camera/color/image_raw",Image),("/camera/depth/image_rect_raw",Image),
            ("/vslam/color/image_raw",Image),("/vslam/depth/image_raw",Image),
            ("/camera/color/camera_info",CameraInfo),("/scan",LaserScan),("/imu",Imu),("/odom",Odometry),
            ("/run_slam/camera_pose",Odometry),("/map",OccupancyGrid),
            ("/global_costmap/costmap",OccupancyGrid),("/local_costmap/costmap",OccupancyGrid)]
    for topic,kind in topics:
        subscriptions.append(node.create_subscription(kind,topic,lambda msg,t=topic:receive(t,msg),qos_profile_sensor_data))
    start=time.monotonic()
    while time.monotonic()-start < args.seconds:
        rclpy.spin_once(node,timeout_sec=.1)
    transforms={}
    for parent,child in [("odom","base_link"),("base_link","assignment3_camera_optical"),("base_link","camera_depth_frame"),("map","odom"),("map","base_link")]:
        try:
            tr=buffer.lookup_transform(parent,child,rclpy.time.Time())
            t,q=tr.transform.translation,tr.transform.rotation
            transforms[f"{parent}<-{child}"]=dict(translation=[t.x,t.y,t.z],quaternion=[q.x,q.y,q.z,q.w],
                age_seconds=(node.get_clock().now() - rclpy.time.Time.from_msg(
                    tr.header.stamp, clock_type=node.get_clock().clock_type)).nanoseconds * 1e-9)
        except Exception as exc:
            transforms[f"{parent}<-{child}"]=dict(error=str(exc))
    report=dict(duration_seconds=time.monotonic()-start,counts=counts,first_messages=details,transforms=transforms,
                topics=dict(node.get_topic_names_and_types()),nodes=node.get_node_names(),
                distinct_visual_stamps=len(visual_stamps),
                note="Sensor snapshots captured from this live headless run; no Gazebo/RViz GUI screenshot claimed")
    if args.require_navigation:
        required_topics = (
            "/camera/color/image_raw", "/camera/depth/image_rect_raw", "/camera/color/camera_info",
            "/scan", "/odom", "/map", "/global_costmap/costmap", "/local_costmap/costmap",
        )
        failures = [f"No messages on {topic}" for topic in required_topics if not counts.get(topic)]
        if len(visual_stamps) < 2:
            failures.append("Expected at least two distinct visual pose timestamps")
        for name in ("map<-odom", "map<-base_link"):
            age = transforms[name].get("age_seconds")
            if age is None or not -0.25 <= age <= 2:
                failures.append(f"{name} missing or outside the allowed simulation-time window")
        if "error" in transforms["base_link<-assignment3_camera_optical"]:
            failures.append("Camera extrinsic unavailable")
        if not details.get("/map", {}).get("known_cells"):
            failures.append("Occupancy grid contains no observed cells")
        report["navigation_failures"] = failures
    (output/"smoke.json").write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))
    node.destroy_node(); rclpy.shutdown()
    return int(bool(report.get("navigation_failures")))


if __name__ == "__main__": sys.exit(main())
