#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../../../.."
docker info >/dev/null
docker build --target slam-cache -f assignments/assignment-3/task3_ros2_nav2/Dockerfile.bootstrap -t ds685-slam-cache .
docker build --target course -f assignments/assignment-3/task3_ros2_nav2/Dockerfile.bootstrap -t ds685-course .
docker build -f assignments/assignment-3/task2_stella_vslam/Dockerfile \
  --build-arg SLAM_CACHE=ds685-slam-cache --build-arg COURSE_IMAGE=ds685-course -t ds685-vslam:latest .
echo 'Fresh-host images built. Generate local assets, then start the integration Compose profile.'
