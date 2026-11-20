# RGB-D StellaVSLAM with Nav2 in an unknown TurtleBot maze

Use the repository root for the commands in this tutorial. No existing assignment or running user container needs to be replaced. The Compose project is named `ds685-visual-slam`; do not run its baseline and integration profiles simultaneously. Both use ROS domain 68, with ROS local to the container and no published ports.

## 1. Prepare and preserve the runtime

```powershell
docker info
docker images
python assignments/assignment-3/task3_ros2_nav2/scripts/prepare_sim.py
docker build -f assignments/assignment-3/task2_stella_vslam/Dockerfile -t ds685-vslam:latest .
docker compose -f assignments/assignment-3/compose.yaml config --quiet
```

`prepare_sim.py` requires PyYAML. Run it with the Python environment used for the assignment. The SDF and Nav2 files are already checked in; regenerate after changing the earlier robot/config source. It makes a separate camera asset, removing the inherited stereo projection offset, fixing the sensor pose, assigning an explicit optical frame, and increasing the nominal sensor rate/resolution. The original model remains untouched.

The default build expects the existing `turtlebot_behavior:overlay` and `turtlebot-maze-demo-slam:latest` images. Record their IDs with `docker image inspect IMAGE`. For another Linux Docker host, either transfer images with `docker save`/`docker load` through private storage, or run:

```bash
bash assignments/assignment-3/task3_ros2_nav2/scripts/bootstrap_linux.sh
# On a managed cloud host whose Git traffic is proxied, pass its trusted CA bundle.
DS685_BUILD_CA=/etc/ssl/certs/ca-certificates.crt bash assignments/assignment-3/task3_ros2_nav2/scripts/bootstrap_linux.sh
```

The fresh-host route builds pinned g2o/FBoW plus a minimal ROS Jazzy/Gazebo course layer. It requires network access, several GB of disk and a suitable CPU/RAM budget. Source/package pins are in the Dockerfiles; apt/base tags are documented rather than byte-for-byte locked. The cached workstation route has the direct runtime evidence. Validate fresh-host builds separately; do not infer successful Gazebo execution from a successful image build.

## 2. Sensor baseline

```powershell
docker compose -f assignments/assignment-3/compose.yaml --profile baseline up -d baseline
docker compose -f assignments/assignment-3/compose.yaml exec baseline bash /work/assignments/assignment-3/task3_ros2_nav2/scripts/inspect.sh
docker compose -f assignments/assignment-3/compose.yaml exec baseline bash -lc 'source /opt/ros/jazzy/setup.bash; python3 /work/assignments/assignment-3/task3_ros2_nav2/ros2_ws/src/ds685_vslam_nav/ds685_vslam_nav/smoke.py --seconds 15 --output /work/assignments/assignment-3/results/ros2/baseline-new'
docker compose -f assignments/assignment-3/compose.yaml stop baseline
```

The unextended baseline tests wheel/IMU/range/RGB-D connectivity. The smoke subscriber also lists some requested topics itself, so **a topic name alone is not evidence of publication**; require positive message counts. The baseline does not yet start visual localization, camera-info bridging or the new static camera TF. Saved failures for those checks are expected at this stage.

Measured baseline: in 12 wall-clock seconds, the new 640×480 sensor delivered 32 RGB and 54 depth messages, with 318 odometry, 2,324 IMU and 57 LiDAR messages. The first depth image was 98.80% finite/positive and reached 3.489 m. RGB and depth sampling counts differ because the diagnostic subscriber receives each stream independently; this does not prove RGB-D pair synchronization. The wrapper checks pairs in the integration stage.

The cached image's launch attempted a GUI and crashed on OpenGL. The assignment mounts the existing corrected `tb_world.launch.py`, which invokes `gz sim -s --headless-rendering`. Headless software-rendered RGB/depth images worked under Docker Desktop; this is useful evidence for remote Linux feasibility, not a guarantee for every VM's EGL/CPU configuration.

## 3. Start the full visual-localization stack

```powershell
docker compose -f assignments/assignment-3/compose.yaml --profile integration up -d --no-build integration
docker compose -f assignments/assignment-3/compose.yaml logs -f integration
```

The launch starts the existing enhanced maze with textured walls, the assignment camera, static camera extrinsics, a CameraInfo bridge, the adapter, the **official** ROS wrapper, the visual-pose bridge, the occupancy inserter and Nav2's navigation-only launch. It does not load an existing map. If your image predates a package-file addition, rebuild it before launching; Python/config/launch edits use the assignment package mounted over the installed source.

| Data | Actual source / consumer |
|---|---|
| `/camera/color/image_raw`, `rgb8` | Gazebo RGB-D sensor → existing bridge → StellaVSLAM |
| `/camera/depth/image_rect_raw`, `32FC1` metres | Same aligned sensor → StellaVSLAM |
| `/camera/color/camera_info` | Extra bridge from `/camera/realsense_d435i/camera_info` |
| `/vslam/color/image_raw`, `/vslam/depth/image_raw` | Optional relay outputs when launched with `relay_images:=true` |
| `/tmp/ds685-camera-*.yaml` | Derived from CameraInfo in the current launch |
| `/run_slam/camera_pose`, `nav_msgs/Odometry` | Official wrapper, map coordinates and ROS camera-body axes |
| `/vslam/base_pose` | Bridge-adjusted planar robot pose; conservative configured covariance |
| `/odom`, `/tf` | Gazebo differential-drive odometry and existing robot-state publisher |
| `/map` | LiDAR log-odds insertion using the visual/odometry TF chain |
| `/scan` | LiDAR → occupancy inserter and Nav2 obstacle layers |
| `/cmd_vel`, `TwistStamped` | Nav2 → existing Gazebo bridge |

The CameraInfo node writes a fresh calibration file and the wrapper waits up to 180 seconds for it. The pinned wrapper is patched to use sensor QoS for its RGB-D subscriptions. It takes images directly from the Gazebo bridge and pairs their original timestamps exactly. Gazebo provides aligned `rgb8` and `32FC1` images, with depth in metres and `Preprocessing.depthmap_factor=1.0`.

Rebuild the integration image before using this launch with a cached image. The prior logs show raw delivery to the Python relay freezing even as a separate subscriber still received frames. To compare the two paths, restart from a clean simulation with `docker compose -f assignments/assignment-3/compose.yaml --profile integration run --rm integration ros2 launch ds685_vslam_nav system.launch.py relay_images:=true`. Use a separate result directory for each smoke window and preserve the launch log. The relay checks encoding and resolution and publishes only exact pairs; in direct mode check these properties in the smoke report.

TF authority is deliberately unique:

```text
map --visual pose bridge--> odom --Gazebo diff drive--> base_footprint --robot_state_publisher--> base_link
base_link --assignment mount--> assignment3_camera --optical rotation--> assignment3_camera_optical
base_link --existing robot description--> base_scan / imu_link
```

The assigned camera mount is 0.07 m forward and 0.16 m high relative to `base_link`. The optical rotation is roll −π/2 and yaw −π/2. The SDF sensor uses the same physical placement. The wrapper already conjugates camera pose into ROS body axes; the bridge must not rotate that message as if it were still an optical-basis pose. `publish_tf:=false` prevents the wrapper from competing for `map -> odom`.

The camera setup is **RGBD**, so the localization translation has depth-derived metric scale. The bridge aligns the first visual gauge with wheel odometry, then applies visual corrections while wheel odometry carries local propagation. A monocular alternative would require a separately validated metric-scale estimator; merely scaling TF by an arbitrary constant would be insufficient.

## 4. Validate tracking, TF and costmaps

```powershell
docker compose -f assignments/assignment-3/compose.yaml exec integration /opt/vslam/entrypoint.sh ros2 run ds685_vslam_nav smoke_test --require-navigation --seconds 20 --output /work/assignments/assignment-3/results/ros2/integration-new --ros-args -p use_sim_time:=true
docker compose -f assignments/assignment-3/compose.yaml exec integration /opt/vslam/entrypoint.sh ros2 run tf2_ros tf2_echo map base_link
docker compose -f assignments/assignment-3/compose.yaml exec integration /opt/vslam/entrypoint.sh ros2 lifecycle get /controller_server
```

Inspect `smoke.json`: require actual RGB/depth/info/visual-pose/map messages and valid `map <- odom`, `map <- base_link`, and `base_link <- assignment3_camera_optical`. Inspect saved CameraInfo values and camera/depth images. Require occupied/free grid cells and published Nav2 costmaps. Nav2 servers must reach `active`. Repeat the strict smoke check with a new output directory after waiting at least 60 seconds, and again after the route. A nonzero status preserves the JSON report and flags missing streams, repeated or missing visual timestamps, stale map TF, or an empty occupancy grid.

The baseline's original blank gray walls provide very few visual corners, so the full launch uses the repository's existing enhanced textured world. This is a legitimate feature-rich test scene. If textures fail to render or initialization is poor, inspect the actual RGB snapshot and material paths before tuning feature thresholds. Do not substitute Gazebo ground truth, an identity map transform or LiDAR localization.

For cautious initial movement before requesting a navigation goal:

```powershell
docker compose -f assignments/assignment-3/compose.yaml exec integration /opt/vslam/entrypoint.sh ros2 run ds685_vslam_nav navigate --initialize-seconds 8 --output /work/assignments/assignment-3/results/ros2/initial-motion.json --ros-args -p use_sim_time:=true
```

This publishes 0.06 m/s forward only when recent front-sector LiDAR clearance exceeds 0.65 m, and stops at the end. It is a diagnostic motion in simulation, not an autonomous exploration policy. Use it while Nav2 is idle; never send competing manual and navigation velocities.

The mapper performs **ray insertion only**, without scan matching. It publishes free=0, occupied=100 and unknown=−1 cells after log-odds updates. This matters because Nav2's trinary StaticLayer otherwise treats sub-100 occupied probabilities as free. Nav2's local range layer supplies immediate obstacles; accumulated `/map` supplies persistent geometry. A 40×40 m grid at 0.05 m resolution is adequate for this scene, not an unbounded world.

## 5. Execute and record a real goal

```powershell
docker compose -f assignments/assignment-3/compose.yaml exec integration /opt/vslam/entrypoint.sh ros2 run ds685_vslam_nav navigate --x 0.5 --y 0.0 --timeout 120 --output /work/assignments/assignment-3/results/ros2/goal-new.json --ros-args -p use_sim_time:=true
```

The probe records goal acceptance, terminal action status, any error code and elapsed time; status 4 means success. A successful short goal establishes basic integration, not completion of the maze. Add successive reachable goals around the actual walls, collect a rosbag and a narrated screen recording, and preserve failures. Goals in unknown space are allowed by the global planner, but the robot must discover obstacles and replan; no frontier-exploration policy is implemented.

For a genuine recording, start `ros2 bag record` from a second shell for `/clock /tf /tf_static /camera/color/image_raw /camera/depth/image_rect_raw /camera/color/camera_info /scan /odom /run_slam/camera_pose /vslam/base_pose /map /cmd_vel`. Use the package entrypoint as above and an output under `data/`. Camera topics are high-volume; use a bounded recording and keep bags out of Git. A rosbag is not a substitute for the requested narrated demonstration video.

Stop the project gracefully so the wrapper writes its map and TUM trajectories:

```powershell
docker compose -f assignments/assignment-3/compose.yaml stop -t 60 integration
docker compose -f assignments/assignment-3/compose.yaml logs --no-color integration
```

Save logs to a fresh result folder before starting another run. The wrapper may need longer than the timeout for a large optimization; never report a saved map unless the file was actually written.

## Failure diagnosis and limits

| Observation | Diagnostic / next action |
|---|---|
| No `/clock` or sensor counts | Inspect Gazebo server log and EGL/software-rendering support; topic listings alone are insufficient. |
| RGB-D pairs absent | Check CameraInfo bridge, frame dimensions, `rgb8`/`32FC1` encodings, synchronized stamps and QoS. |
| Wrapper runs but no visual poses | Inspect texture/features, CameraInfo-derived YAML, depth in metres and wrapper tracking log. |
| Visual pose exists, no map TF | Check exact-time `odom -> base_link`, the camera-body extrinsic, use_sim_time and bridge warning log. |
| Map exists, Nav2 not active | Inspect lifecycle logs, plugin availability and global/local costmap TF. |
| Map ghosts after loop correction | Stop navigation, clear/rebuild occupancy and reinitialize. A robust keyframe-correctable dense mapper remains future work. |
| Lost tracking | The bridge stops producing new transforms; stale TF should block motion. Do not keep publishing a held visual pose as if it were newly localized. |
| New cloud host | Run Python CI first, build/import images, then repeat sensor/tracking/TF/costmap/goal stages. Cloud-provider creation and GPU allocation are not automated. |

## Measured runtime outcome at travel handoff

`../results/ros2/integration02/smoke.json` validates all required sensor streams, 157 visual-pose messages, map TF and both costmaps. Controller and navigator were active. `goal01.json` records the successful (0.5, 0.0) goal: accepted, terminal status 4, error code 0, 5.345 s. This run used Python TimeSynchronizer and the upstream wrapper's approximate synchronization; commit `9789edb` preserves that source version.

The stream later stalled: adapter pair counts stopped increasing while independent probes still saw raw color/depth. `sync-diagnostic.json` found 106 equal-stamp pairs across 113 RGB/111 depth messages, confirming that the raw sensor can supply matching stamps. `goal02.json` records a 2 m goal abort; `goal03.json` records rejection on a fresh run without visual map TF. The latest directly testable pairing buffer is algorithm-tested but did not resolve the runtime stall. Investigate per-subscription DDS/QoS delivery and buffer/loan lifetime next; the root cause is not established. Do not claim reliable full-maze navigation.

`data/navigation-bag/` is a real 54.4 s, 21,971-message recording; its small inventory is committed as `navigation-bag-info.txt`. It began after goal01 finished, and contains zero velocity commands, so it records localization/geometry and the later stall rather than the completed short motion. `data/navigation02.avi` records actual camera imagery during a subsequent failed run; metadata describe its scope. These raw artifacts are ignored by Git and need an owner-controlled backup if they must survive loss of the workstation.

The isolated simulation was stopped at handoff. Restart with the commands above, then repeat every stage. No personal camera reconstruction, full-maze run, GUI recording or published video is claimed.
