"""Record actual simulated RGB frames with measured visual-pose annotations, bounded in wall time."""
import json
from pathlib import Path
import time
import cv2
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge

rclpy.init(); node=rclpy.create_node('ds685_camera_recording'); bridge=CvBridge()
path=Path('/work/assignments/assignment-3/data/navigation02.avi')
state={'writer':None,'last':0.,'frames':0,'pose':None}; subscriptions=[]
def pose(msg):
    p=msg.pose.pose.position
    state['pose']=[p.x,p.y,msg.header.stamp.sec+msg.header.stamp.nanosec/1e9]
def image(msg):
    now=time.monotonic()
    if now-state['last']<.2:return
    frame=bridge.imgmsg_to_cv2(msg,'bgr8')
    if state['writer'] is None:
        state['writer']=cv2.VideoWriter(str(path),cv2.VideoWriter_fourcc(*'MJPG'),5.,frame.shape[1::-1])
        if not state['writer'].isOpened(): raise RuntimeError('video writer failed')
    stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
    cv2.putText(frame,f'Actual Gazebo RGB | sim t={stamp:.2f}s',(10,24),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,255,255),1)
    if state['pose']:
        x,y,t=state['pose']
        cv2.putText(frame,f'Visual map pose x={x:.3f}, y={y:.3f}, pose t={t:.2f}s',(10,47),cv2.FONT_HERSHEY_SIMPLEX,.45,(0,255,255),1)
    state['writer'].write(frame); state.update(last=now,frames=state['frames']+1)
subscriptions.append(node.create_subscription(Image,'/camera/color/image_raw',image,qos_profile_sensor_data))
subscriptions.append(node.create_subscription(Odometry,'/vslam/base_pose',pose,10))
start=time.monotonic()
try:
    while time.monotonic()-start<55:rclpy.spin_once(node,timeout_sec=.1)
finally:
    if state['writer']:state['writer'].release()
    report=dict(kind='actual Gazebo RGB camera recording with measured visual pose',frames=state['frames'],
                wall_duration_s=time.monotonic()-start,encoded_fps=5.,video=str(path),
                note='Frames sampled at most 5 Hz wall time; no Gazebo/RViz GUI recording claimed')
    Path('/work/assignments/assignment-3/results/ros2/navigation-recording.json').write_text(json.dumps(report,indent=2))
    node.destroy_node();rclpy.try_shutdown()
