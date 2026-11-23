"""Record the simulator camera, live occupancy, visual path and Nav2 commands."""
import argparse
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image


def stamp(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def caption(frame, text, pos, size=.62, color=(233, 242, 246)):
    cv2.putText(frame, text, pos, cv2.FONT_HERSHEY_SIMPLEX, size, color, 1, cv2.LINE_AA)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--seconds", type=float, default=105)
    args = parser.parse_args()
    folder = args.output
    folder.mkdir(parents=True, exist_ok=True)
    fps = 10
    goals = [(0.5, 0.0), (1.25, 0.0)]
    map_width, map_height = 508, 504
    pixels_per_metre = 96
    camera = None
    grid = None
    camera_stamp = 0.0
    position_stamps = []
    path = []
    positions_in_video = []
    commands = []
    drawn_frames = 0
    result_frames = 0
    first_camera_stamp = None
    bridge = CvBridge()

    def on_image(msg):
        nonlocal camera, camera_stamp
        camera = bridge.imgmsg_to_cv2(msg, "bgr8")
        camera_stamp = stamp(msg.header)

    def on_pose(msg):
        p = msg.pose.pose.position
        t = stamp(msg.header)
        if not position_stamps or t > position_stamps[-1]:
            path.append((p.x, p.y))
            position_stamps.append(t)

    def on_map(msg):
        nonlocal grid
        values = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        raster = np.full(values.shape, 187, dtype=np.uint8)
        raster[values == 0] = 248
        raster[values > 0] = 37
        origin = msg.info.origin.position
        resolution = msg.info.resolution
        transform = np.float32([
            [pixels_per_metre * resolution, 0, map_width / 2 + (origin.x - 0.7) * pixels_per_metre],
            [0, -pixels_per_metre * resolution, map_height / 2 - origin.y * pixels_per_metre],
        ])
        preview = cv2.warpAffine(raster, transform, (map_width, map_height),
                                 flags=cv2.INTER_NEAREST, borderValue=187)
        grid = cv2.cvtColor(preview, cv2.COLOR_GRAY2BGR)

    def on_command(msg):
        commands.append((stamp(msg.header), msg.twist.linear.x, msg.twist.angular.z))

    def map_pixel(point):
        x, y = point
        return (round(map_width / 2 + (x - .7) * pixels_per_metre),
                round(map_height / 2 - y * pixels_per_metre))

    rclpy.init()
    node = rclpy.create_node("ds685_navigation_recording")
    subscriptions = [
        node.create_subscription(Image, "/camera/color/image_raw", on_image, qos_profile_sensor_data),
        node.create_subscription(Odometry, "/vslam/base_pose", on_pose, 10),
        node.create_subscription(OccupancyGrid, "/map", on_map,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        node.create_subscription(TwistStamped, "/cmd_vel", on_command, qos_profile_sensor_data),
    ]
    writer = cv2.VideoWriter(str(folder / "navigation-live.avi"), cv2.VideoWriter_fourcc(*"MJPG"),
                             fps, (1280, 720))
    start = time.monotonic()
    next_frame = start
    previous_stamp = None
    frame = None
    try:
        if not writer.isOpened():
            raise RuntimeError("Could not open navigation video writer")
        while rclpy.ok() and time.monotonic() - start < args.seconds:
            if (folder / "recording.stop").exists():
                break
            rclpy.spin_once(node, timeout_sec=.03)
            now = time.monotonic()
            if camera is None or grid is None or not path or camera_stamp == previous_stamp or now < next_frame:
                continue
            previous_stamp = camera_stamp
            next_frame = max(next_frame + 1 / fps, now)
            if first_camera_stamp is None:
                first_camera_stamp = camera_stamp
            frame = np.full((720, 1280, 3), (31, 46, 60), dtype=np.uint8)
            caption(frame, "DS685 Task 3  |  StellaVSLAM + Nav2 in Gazebo", (34, 41), .99)
            caption(frame, "Live robot camera", (34, 77), .72)
            caption(frame, "Online occupancy map and visual path", (738, 77), .72)
            frame[94:598, 34:706] = cv2.resize(camera, (672, 504))
            panel = grid.copy()
            pixels = np.asarray([map_pixel(p) for p in path], dtype=np.int32)
            cv2.polylines(panel, [pixels], False, (185, 91, 15), 3, cv2.LINE_AA)
            for index, goal in enumerate(goals, 1):
                pixel = map_pixel(goal)
                cv2.drawMarker(panel, pixel, (0, 126, 199), cv2.MARKER_CROSS, 19, 2)
                caption(panel, f"G{index}", (pixel[0] - 12, pixel[1] + 28), .51, (19, 54, 89))
            cv2.circle(panel, map_pixel(path[-1]), 7, (185, 91, 15), -1, cv2.LINE_AA)
            frame[94:598, 738:1246] = panel
            x, y = path[-1]
            pose_age = camera_stamp - position_stamps[-1]
            caption(frame, f"Sim time {camera_stamp:.1f} s  |  Visual map pose x {x:.2f} m, y {y:.2f} m  |  Pose offset {pose_age:+.2f} s",
                    (34, 638))
            caption(frame, "Blue: visual path  |  +: Nav2 goals  |  Map built from live LiDAR and visual localization. No prior map.",
                    (34, 683), .61)
            writer.write(frame)
            drawn_frames += 1
            positions_in_video.append((x, y))
            if drawn_frames == fps:
                (folder / "recording.ready").touch()

        results = []
        for number in (1, 2):
            goal_file = folder / f"goal-{number}.json"
            if goal_file.exists():
                results.append(json.loads(goal_file.read_text()))
        if frame is not None and results:
            summary = frame.copy()
            cv2.rectangle(summary, (38, 185), (715, 417), (31, 46, 60), -1)
            caption(summary, "Measured Nav2 action results", (64, 235), .91)
            for index, result in enumerate(results, 1):
                x, y, _ = result["goal"]
                outcome = "succeeded" if result["succeeded"] else "not reached"
                caption(summary, f"Goal {index} ({x:.2f}, {y:.2f}) m: {outcome}",
                        (64, 235 + index * 45), .70)
            caption(summary, "AI assistance: Codex helped automate the test and recording.", (64, 389), .46)
            for _ in range(fps * 4):
                writer.write(summary)
                result_frames += 1
    finally:
        writer.release()
        displacement = (math.dist(positions_in_video[0], positions_in_video[-1])
                        if len(positions_in_video) >= 2 else 0.0)
        report = dict(
            camera_frames=drawn_frames, end_card_frames=result_frames, encoded_fps=fps,
            first_camera_stamp=first_camera_stamp, last_camera_stamp=previous_stamp,
            visual_pose_count=len(path), visual_displacement_m=displacement,
            command_count=len(commands),
            nonzero_commands=sum(abs(v) > 1e-3 or abs(w) > 1e-3 for _, v, w in commands),
            poses=[dict(stamp=t, x=x, y=y) for t, (x, y) in zip(position_stamps, path)],
        )
        (folder / "recording.json").write_text(json.dumps(report, indent=2) + "\n")
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
