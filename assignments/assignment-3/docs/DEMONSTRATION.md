# Narrated demonstration and visual evidence

No YouTube upload or link is claimed. Publish only after you have the actual recording and are comfortable showing the interior of your space.

Suggested 4–6 minute sequence:

1. **20 s — project overview.** “I implemented calibrated monocular visual odometry, evaluated an official KITTI prefix, and integrated metric RGB-D StellaVSLAM localization with range-based occupancy mapping for Nav2.” Show the architecture and clearly label the measured state of each component.
2. **45 s — Task 1.** Run the CLI on the official sequence prefix. Show actual ORB matching, triangulated landmarks and the estimated/ground-truth plot. State the measured ATE/RPE and the scale-drift limitation. Explain why one global Sim(3) alignment is appropriate for monocular output. Do not present rendered test images as KITTI.
3. **45 s — reference StellaVSLAM.** Show the `aist_living_lab_1` panoramic video, real feature-tracking snapshots, saved reconstruction and terminal outcome. Explain that its camera model is equirectangular and its map scale is arbitrary.
4. **60–90 s — your room.** Show a few original calibration-board images and measured reprojection-error report, then the perspective YAML and a short portion of the recorded room route. Show live/saved feature tracking, a revisit/loop if achieved, and the actual reconstructed map. Describe failures honestly. Keep identifying documents, addresses and people out of the footage.
5. **60–90 s — navigation.** Start a new maze run without loading a map. Show real RGB-D input, the valid TF chain, the growing occupancy grid and Nav2 costmaps, goal acceptance and the robot reaching the goal. If only a short goal has been validated, state that scope. Show the saved failure if the full maze is incomplete.
6. **20 s — interpretation.** Explain that visual/depth measurements supply metric localization, wheel odometry supplies local motion, and LiDAR supplies occupancy/obstacles. State what remains incomplete and link the exact assignment directory.

Evidence required for a complete course submission:

| Evidence | Where to obtain it |
|---|---|
| KITTI plot overlaid on real ground truth | `results/kitti-00/trajectory_xz.png` |
| ORB matches and triangulation | `results/kitti-00/matches_*.jpg`, `triangulation.png` |
| Reference tracker and map | Evidence-layer output `tracking_*.jpg`, saved `reconstruction.png`, actual run log |
| Personal calibration quality | Real `calibration.json`, board views and corner coverage |
| Personal indoor reconstruction | Real perspective footage, tracker view, saved map/trajectory |
| TF and navigation | Runtime `smoke.json`, occupancy/costmap captures, actual action-result JSON and recording |
| GUI screenshots if used | Capture the actual running Gazebo/RViz/viewer windows with the OS recording tool |
| Narrated demonstration link | Actual YouTube URL, added after owner publication |

Raw bags/videos and large maps belong in private storage outside Git. Commit small selected evidence and enough metadata to reproduce it. Run `git status` and push the branch before shutting down the workstation. Verify the backup from the travel machine.
