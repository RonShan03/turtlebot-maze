import argparse
import json
from pathlib import Path
import time
import math
import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data


def main():
    p=argparse.ArgumentParser(description='Bounded real Nav2 goal or scan-guarded initial motion; logs actual outcome')
    p.add_argument('--x',type=float,default=.5); p.add_argument('--y',type=float,default=0.); p.add_argument('--yaw',type=float,default=0.)
    p.add_argument('--timeout',type=float,default=90); p.add_argument('--output',required=True)
    p.add_argument('--initialize-seconds',type=float,default=0.,help='slow forward motion only; use before starting navigation goals')
    args=p.parse_args(); rclpy.init(); node=rclpy.create_node('ds685_navigation_probe')
    report={'mode':'Nav2 goal','goal':[args.x,args.y,args.yaw],'accepted':False,'succeeded':False}
    start=time.monotonic()
    try:
        if args.initialize_seconds>0:
            report['mode']='scan-guarded initial motion'; report['commanded_seconds']=args.initialize_seconds
            latest={'scan':None,'time':0.}
            def scan(msg): latest.update(scan=msg,time=time.monotonic())
            sub=node.create_subscription(LaserScan,'/scan',scan,qos_profile_sensor_data)
            pub=node.create_publisher(TwistStamped,'/cmd_vel',10)
            while time.monotonic()-start<args.initialize_seconds:
                rclpy.spin_once(node,timeout_sec=.05)
                msg=latest['scan']; safe=False
                if msg is not None and time.monotonic()-latest['time']<2:
                    front=[r for i,r in enumerate(msg.ranges) if abs(math.atan2(math.sin(msg.angle_min+i*msg.angle_increment),math.cos(msg.angle_min+i*msg.angle_increment)))<.3]
                    safe=bool(front) and min(front)>.65
                command=TwistStamped(); command.header.stamp=node.get_clock().now().to_msg()
                command.twist.linear.x=.06 if safe else 0.; pub.publish(command)
            command=TwistStamped(); command.header.stamp=node.get_clock().now().to_msg(); pub.publish(command)
            report['finished']=True
        else:
            client=ActionClient(node,NavigateToPose,'navigate_to_pose')
            if not client.wait_for_server(timeout_sec=min(args.timeout,30.)):
                raise RuntimeError('Nav2 action server unavailable')
            goal=NavigateToPose.Goal(); goal.pose.header.frame_id='map'; goal.pose.header.stamp=node.get_clock().now().to_msg()
            goal.pose.pose.position.x=args.x; goal.pose.pose.position.y=args.y
            goal.pose.pose.orientation.z=math.sin(args.yaw/2); goal.pose.pose.orientation.w=math.cos(args.yaw/2)
            future=client.send_goal_async(goal)
            rclpy.spin_until_future_complete(node,future,timeout_sec=10.)
            if not future.done(): raise RuntimeError('goal acceptance timed out')
            handle=future.result(); report['accepted']=handle.accepted
            if not handle.accepted: raise RuntimeError('goal rejected')
            result=handle.get_result_async()
            while not result.done() and time.monotonic()-start<args.timeout: rclpy.spin_once(node,timeout_sec=.2)
            if not result.done():
                cancel=handle.cancel_goal_async(); rclpy.spin_until_future_complete(node,cancel,timeout_sec=5.)
                report['error']='goal timed out; cancellation requested'
            else:
                report['action_status']=result.result().status; report['succeeded']=result.result().status==4
                report['error_code']=int(getattr(result.result().result,'error_code',0))
    except Exception as exc: report['error']=str(exc)
    finally:
        report['elapsed_seconds']=time.monotonic()-start
        path=Path(args.output); path.parent.mkdir(parents=True,exist_ok=True); path.write_text(json.dumps(report,indent=2))
        print(json.dumps(report,indent=2)); node.destroy_node(); rclpy.try_shutdown()


if __name__=='__main__':main()
