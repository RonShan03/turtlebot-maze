# Deliverable coverage

The table records which assignment deliverables have supporting evidence.

| Requirement | Artifact / evidence | Status / remaining action |
|---|---|---|
| Official requirements and linked docs inspected | README references; pinned upstream sources | Done |
| Assignment directory, dedicated branch, preserved earlier work | `assignments/assignment-3`, `codex/visual-slam` | Done; no merge to main |
| ORB/Hamming matching and robust essential geometry | `features.py`, `geometry.py`, geometry tests | Implemented and tested |
| Cheirality, triangulation, depth/parallax/reprojection rejection | `geometry.py`, `test_geometry.py` | Implemented and tested |
| Camera trajectory, keyframes, 2D–3D PnP, loss handling | `pipeline.py`, `test_pipeline.py` | Implemented and tested; no global BA/loop closure |
| KITTI images/calibration/times/poses | `dataset.py`, `acquire_kitti.py`, acquisition hashes | 150 official sequence-00 frames acquired/evaluated |
| Ground-truth trajectory comparison, ATE/RPE and diagnostics | `results/kitti-00/` | Executed; significant measured scale drift documented |
| Synthetic fallback and reproducibility | `synthetic.py`, `results/synthetic/`, CI | Executed locally; Linux CI evidence assessed separately |
| Stella official installation and compatible CLI | Task-2 Dockerfile, guarded Jazzy patches, build logs | Core, examples, official ROS wrapper and custom ROS package built successfully |
| AIST dataset, vocabulary, official camera YAML | `acquire_reference.py`, pinned image assets | Acquired; reference experiment results recorded separately |
| AIST feature/map screenshots and database | `results/stella/aist/` | Executed: 225 keyframes, 7,215 landmarks, genuine tracker frames; large map local/ignored |
| Physical calibration and quality report | README calibration summary | 23 phone checkerboard views, 0.577 px RMS |
| Perspective YAML and video validation | Camera `yaml` and `video` commands | Measured YAML generated in the cloud workspace. The personal clip decoded all 2,512 frames. |
| Personal room reconstruction | [Task 2 demo](https://youtu.be/vi-8dxjenc0) | 543 saved camera poses, 65 keyframes, and 1,531 landmarks |
| YouTube explanation and link | [Task 2 demo](https://youtu.be/vi-8dxjenc0) | Published and linked in the README |
| ROS/Gazebo/TurtleBot reuse and live connectivity | Baseline inventory, smoke JSON, RGB/depth PNGs | Executed headlessly |
| Wrapper RGB-D topics, metric scale and TF | ROS package, CameraInfo adapter, pose bridge | Implemented; runtime validation reported by stage |
| Occupancy/obstacle geometry without LiDAR localization substitution | Log-odds ray inserter, Nav2 parameters and core tests | Implemented and algorithm-tested; visual TF required |
| Unknown-maze Nav2 navigation | [Hosted run](https://github.com/RonShan03/turtlebot-maze/actions/runs/45123583442), [demo video](../results/ros2/demo/task3-demo.mp4) | Direct RGB-D path sustained tracking without a prior map. First action reported success. Second action timed out. |
| Classmate tutorial and exact reproduction commands | `task3_ros2_nav2/TUTORIAL.md` | Written; runtime findings preserved |
| Headless remote operation | Compose and Linux bootstrap script | Verified on GitHub-hosted Ubuntu 24.04 |
| Automated checks, dependency documentation, logs | `tests/`, lock snapshot, build/test logs, GitHub workflow | Local tests executed; no physical-camera claim |
| Private repository and TA access | [Submission guide](https://aegean.ai/aiml-common/resources/environment/assignment-submission) | Private course repository and grader access pending |
| Assignment directory URL | `assignments/assignment-3` on `main` | Submit the private course repository directory URL in Canvas or Brightspace |
