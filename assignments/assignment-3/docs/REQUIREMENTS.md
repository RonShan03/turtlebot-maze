# Deliverable coverage

This is the user-requested submission checklist, expressed as a coverage table. Statuses distinguish real execution from source preparation. It is not an assertion that pending experiments passed.

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
| Stella official installation and compatible CLI | Task-2 Dockerfile, guarded Jazzy patches, build logs | Core/examples built; wrapper runtime status in tutorial/results |
| AIST dataset, vocabulary, official camera YAML | `acquire_reference.py`, pinned image assets | Acquired; reference experiment results recorded separately |
| AIST feature/map screenshots and database | `Dockerfile.evidence`, `run_video.sh`, `plot_results.py` | Require completed actual reference-run artifacts |
| Physical calibration and quality report | `camera.py`, before-travel guide | Tools tested; real images, square measurement and camera required |
| Perspective YAML and video validation | Camera `yaml` / `video` commands and tests | Implemented; actual personal calibration still required |
| Personal room reconstruction | Exact run commands in README | Pending actual footage/calibration |
| YouTube explanation and link | `DEMONSTRATION.md` | Script prepared; owner must record/publish actual video |
| ROS/Gazebo/TurtleBot reuse and live connectivity | Baseline inventory, smoke JSON, RGB/depth PNGs | Executed headlessly |
| Wrapper RGB-D topics, metric scale and TF | ROS package, CameraInfo adapter, pose bridge | Implemented; runtime validation reported by stage |
| Occupancy/obstacle geometry without LiDAR localization substitution | Log-odds ray inserter, Nav2 parameters and core tests | Implemented and algorithm-tested; visual TF required |
| Unknown-maze Nav2 navigation | System launch and bounded action probe | Require actual accepted/successful goal and full route evidence |
| Classmate tutorial and exact reproduction commands | `task3_ros2_nav2/TUTORIAL.md` | Written; runtime findings preserved |
| Headless remote operation and cloud fallback | Compose, bootstrap Dockerfile/script, remote-access guide | Local headless baseline proven; new-host bootstrap untested separately |
| Automated checks, dependency documentation, logs | `tests/`, lock snapshot, build/test logs, GitHub workflow | Local tests executed; no physical-camera claim |
| Frequent commit/push | Branch history at authorized remote | Milestones pushed; final handoff includes exact hash |
| Private repository and TA access | Owner GitHub settings | User authorized existing public remote; course guide requests private visibility and TA access |
| Final directory URL on submitted main | `https://github.com/RonShan03/turtlebot-maze/tree/main/assignments/assignment-3` | Only valid after owner-approved merge within deadline; not submitted automatically |
