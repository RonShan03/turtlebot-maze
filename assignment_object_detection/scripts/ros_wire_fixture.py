"""Generate an independent odometry CDR fixture using ROS's own serializer."""
from pathlib import Path
from nav_msgs.msg import Odometry
from rclpy.serialization import serialize_message

msg=Odometry()
msg.header.stamp.sec=4;msg.header.stamp.nanosec=5;msg.header.frame_id='odom'
msg.child_frame_id='base_link'
msg.pose.pose.position.x=1.;msg.pose.pose.position.y=2.;msg.pose.pose.position.z=3.
msg.pose.pose.orientation.w=1.
msg.pose.covariance=list(map(float,range(36)))
msg.twist.twist.linear.x=4.;msg.twist.twist.angular.z=.75
msg.twist.covariance=list(map(float,range(36,72)))
path=Path('/assignment/tests/fixtures/ros_odometry.cdr')
path.parent.mkdir(parents=True,exist_ok=True)
path.write_bytes(serialize_message(msg))
print(f'Wrote independent ROS fixture: {path}')
