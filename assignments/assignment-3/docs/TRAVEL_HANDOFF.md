# Travel handoff — November 19, 2026

The code, reports, test results, KITTI plots, AIST tracking/reconstruction images and ROS evidence are committed on `codex/visual-slam` in the authorized GitHub repository. `main` and earlier assignments were not modified. The isolated project simulation is stopped.

**Before leaving:** capture the personal calibration-board views and room video using [BEFORE_TRAVEL.md](BEFORE_TRAVEL.md), and back them up to storage available from your travel machine. Without those inputs, personal reconstruction cannot be completed remotely. The concise commands are:

```powershell
# From assignments/assignment-3, using a Python environment with the project installed:
python -m task2_stella_vslam.camera capture --output data/personal/calibration --width 640 --height 480
# Press S for each varied board view; Q finishes.
python -m task2_stella_vslam.camera capture --output data/personal/recording --width 640 --height 480 --fps 30 --record --seconds 150
```

If remote workstation access is needed, review and manually perform [REMOTE_ACCESS.md](REMOTE_ACCESS.md), then test from the travel laptop over a hotspot. Tailscale/OpenSSH were not found in the inspected PATH/services, and Windows Remote Desktop service was stopped. No remote-access installation, firewall rule, public-port exposure or unattended-access configuration was performed. A manual setup and external connectivity test remain necessary.

Raw artifacts deliberately excluded from Git:

| Path | Contents / backup priority |
|---|---|
| `data/personal/` | Owner's future calibration images/footage; highest priority |
| `results/stella/aist/map.msg` | Actual 41.4 MB reference map; can also be reproduced from the official video |
| `data/aist/` | Official 79.7 MB archive and extracted video; reproducible |
| `data/kitti/` | Official 150-frame sequence prefix and hashes; reproducible with range acquisition |
| `data/navigation-bag/` | Actual localization/map recording; inventory committed |
| `data/navigation02.avi` | Actual camera recording of a later failed integration run; not a recording of goal01's successful motion |
| Local Docker images | Cached course stack and built `ds685-vslam:latest` / `:evidence`; export only if needed and disk/time permit |

Proven results: 18 local tests pass; official KITTI prefix executes with ATE 7.512 m; official AIST run saves 225 keyframes/7,215 landmarks; headless ROS sensors, visual TF, occupancy and costmaps work during the validated window; a 0.5 m target goal succeeds in 5.345 s without a prior map.

Remaining runtime issue: image delivery to the adapter intermittently stalls, although independent subscribers see the raw images. Longer goals subsequently abort or are rejected because visual TF stops. Commit `9789edb` preserves the source at the successful short-goal milestone; the latest source includes additional exact-pair buffer diagnostics/tests but is not proven reliable. Preserve failures when resuming. A suitable remote Linux Docker machine can build/import images and run the same staged tutorial; the fresh-host bootstrap is provided but not yet independently built. Python-only CI cannot establish simulation success.

Remaining submission work: resolve streaming reliability, demonstrate an actual longer maze route, complete measured personal calibration/reconstruction, capture the narrated demonstration, publish its real YouTube link, verify TA access/repository visibility, and merge the completed directory to `main` within the applicable deadline. No course-portal submission or video publication was performed.
