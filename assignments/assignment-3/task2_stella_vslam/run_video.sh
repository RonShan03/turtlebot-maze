#!/usr/bin/env bash
set -euo pipefail
if [[ $# -ne 3 ]]; then echo 'Usage: run_video.sh VIDEO CAMERA_YAML OUTPUT_DIR'; exit 2; fi
video=$(realpath "$1"); config=$(realpath "$2"); output=$3
test -f "$video" && test -f "$config"
if [[ -f "$output/run.log" ]]; then echo 'Use a fresh output directory'; exit 2; fi
mkdir -p "$output"
output=$(realpath "$output")
exe=/opt/vslam/examples/build/run_video_slam
"$exe" --help > "$output/executable-help.txt" 2>&1 || true
"$exe" -v /opt/vslam/orb_vocab.fbow -m "$video" -c "$config" \
  --frame-skip 3 --no-sleep --auto-term --eval-log-dir "$output" --map-db-out "$output/map.msg" \
  2>&1 | tee "$output/run.log"
test -s "$output/map.msg"
