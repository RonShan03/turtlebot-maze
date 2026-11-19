"""Bounded raw header timestamp diagnostic, independent of message_filters."""
import json
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
rclpy.init(); node=rclpy.create_node('rgbd_timestamp_probe')
stamps=[[],[]]; subs=[]
def receive(i,msg): stamps[i].append(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec)
for i,topic in enumerate(['/camera/color/image_raw','/camera/depth/image_rect_raw']):
    subs.append(node.create_subscription(Image,topic,lambda m,index=i:receive(index,m),qos_profile_sensor_data))
start=time.monotonic()
while time.monotonic()-start<8: rclpy.spin_once(node,timeout_sec=.1)
nearest=[min(abs(a-b) for b in stamps[1]) for a in stamps[0]] if all(stamps) else []
report=dict(counts=list(map(len,stamps)),common_stamps=len(set(stamps[0])&set(stamps[1])),
            nearest_delta_ns=np.percentile(nearest,[0,25,50,75,100]).tolist() if nearest else [],
            first_stamps=[x[:4] for x in stamps])
path=Path('/work/assignments/assignment-3/results/ros2/sync-diagnostic.json'); path.write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2)); node.destroy_node(); rclpy.shutdown()
