"""Run inside the ROS container: initialize AMCL, follow Nav2 room waypoints."""
import argparse
import json
import math
from pathlib import Path
import time
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--route',required=True,type=Path)
    p.add_argument('--output',required=True,type=Path)
    p.add_argument('--timeout',type=float,default=180)
    p.add_argument('--initialize',action='store_true',help='Only for a fresh robot spawned at (0,0,0)')
    args=p.parse_args()
    rclpy.init()
    nav=BasicNavigator()
    def pose(x,y,angle):
        msg=PoseStamped()
        msg.header.frame_id='map'
        msg.header.stamp=nav.get_clock().now().to_msg()
        msg.pose.position.x=float(x); msg.pose.position.y=float(y)
        msg.pose.orientation.z=math.sin(angle/2); msg.pose.orientation.w=math.cos(angle/2)
        return msg
    initial=pose(0,0,0)
    if args.initialize:
        nav.setInitialPose(initial)
    nav.waitUntilNav2Active()
    args.output.parent.mkdir(parents=True,exist_ok=True)
    try:
        with args.output.open('w') as out:
            for waypoint in json.loads(args.route.read_text()):
                print(f'GOAL {waypoint}',flush=True)
                nav.goToPose(pose(waypoint['x'],waypoint['y'],waypoint['yaw']))
                start=time.monotonic()
                while not nav.isTaskComplete():
                    if time.monotonic()-start > args.timeout:
                        nav.cancelTask()
                    time.sleep(.1)
                result=nav.getResult()
                out.write(json.dumps(dict(**waypoint,result=str(result),elapsed_s=time.monotonic()-start))+'\n')
                out.flush()
                print(f'RESULT {result}',flush=True)
                if result != TaskResult.SUCCEEDED:
                    print('Stopping route after failed goal; inspect obstacle/localization.',flush=True)
                    break
    finally:
        nav.cancelTask()
        nav.destroy_node()
        rclpy.shutdown()

if __name__=='__main__': main()
