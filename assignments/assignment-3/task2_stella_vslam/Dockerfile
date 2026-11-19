# Reuse the workstation's compiled g2o/FBoW and course ROS/Gazebo stack.
# Build from the repository root; neither earlier image is modified.
ARG SLAM_CACHE=turtlebot-maze-demo-slam:latest
ARG COURSE_IMAGE=turtlebot_behavior:overlay
FROM ${SLAM_CACHE} AS cached
FROM ${COURSE_IMAGE} AS examples
USER root
SHELL ["/bin/bash", "-c"]
COPY --from=cached /usr/local/ /usr/local/
COPY --from=cached /slam/vocab/orb_vocab.fbow /opt/vslam/orb_vocab.fbow
ENV LD_LIBRARY_PATH=/usr/local/lib:/usr/local/lib/fbow
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git curl ca-certificates ffmpeg \
    libeigen3-dev libspdlog-dev libyaml-cpp-dev libsqlite3-dev libsuitesparse-dev \
    libgoogle-glog-dev libgflags-dev libatlas-base-dev libopencv-dev \
    ros-jazzy-tf2-eigen ros-jazzy-cv-bridge ros-jazzy-image-transport \
    ros-jazzy-message-filters ros-jazzy-rosbag2-cpp python3-scipy python3-yaml python3-msgpack \
    && rm -rf /var/lib/apt/lists/* && ldconfig
ARG STELLA_REF=e445b5452535e781fdf6777ec33a06f5f8f5e416
ARG EXAMPLES_REF=defc69eecc36e51cdda22885bb86954f08ad6887
ARG ROS_REF=186f623dd1c24ee83678f8e1da593801f450bbb3
ARG BUILD_JOBS=2
RUN git clone https://github.com/stella-cv/stella_vslam.git /opt/vslam/core \
    && cd /opt/vslam/core && git checkout ${STELLA_REF} && git submodule update --init --recursive \
    && if test -f /usr/local/lib/fbow/fbowTargets.cmake; then sed -i 's|/build/stella_vslam/3rd/FBoW/include|/usr/local/include|g; s|/build/stella_vslam/build/3rd/FBoW/|/usr/local/lib/|g' /usr/local/lib/fbow/fbowTargets.cmake; fi \
    && cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_WITH_MARCH_NATIVE=OFF \
    && cmake --build build -j${BUILD_JOBS} && cmake --install build && ldconfig
RUN git clone https://github.com/stella-cv/stella_vslam_examples.git /opt/vslam/examples \
    && cd /opt/vslam/examples && git checkout ${EXAMPLES_REF} && git submodule update --init --recursive \
    && cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build build -j${BUILD_JOBS} --target run_video_slam run_camera_slam
FROM examples AS integration
COPY assignments/assignment-3/task2_stella_vslam/patch_wrapper.py /opt/vslam/patch_wrapper.py
RUN mkdir -p /opt/vslam/ros_ws/src \
    && git clone -b ros2 https://github.com/stella-cv/stella_vslam_ros.git /opt/vslam/ros_ws/src/stella_vslam_ros \
    && cd /opt/vslam/ros_ws/src/stella_vslam_ros && git checkout ${ROS_REF} && git submodule update --init --recursive \
    && python3 /opt/vslam/patch_wrapper.py /opt/vslam/ros_ws/src/stella_vslam_ros/CMakeLists.txt \
    && source /opt/ros/jazzy/setup.bash && cd /opt/vslam/ros_ws \
    && MAKEFLAGS=-j${BUILD_JOBS} colcon build --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
COPY assignments/assignment-3/task3_ros2_nav2/ros2_ws/src/ds685_vslam_nav /opt/vslam/nav_ws/src/ds685_vslam_nav
RUN source /opt/ros/jazzy/setup.bash && cd /opt/vslam/nav_ws && colcon build --symlink-install
COPY assignments/assignment-3/task3_ros2_nav2/scripts/entrypoint.sh /opt/vslam/entrypoint.sh
RUN chmod +x /opt/vslam/entrypoint.sh
WORKDIR /work
ENTRYPOINT ["/opt/vslam/entrypoint.sh"]
CMD ["bash"]
