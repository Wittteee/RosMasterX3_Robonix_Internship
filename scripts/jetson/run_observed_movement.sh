#!/usr/bin/env bash
set -eo pipefail
root="$HOME/rosmaster_x3_deploy"
camera_pid=$(pgrep -f '^python3 /home/jetson/rosmaster_x3_deploy/scripts/watch_camera_qr.py$' || true)
if [ -n "$camera_pid" ]; then kill "$camera_pid"; sleep 1; fi
nohup python3 "$root/scripts/record_movement.py" >"$root/logs/movement_video.log" 2>&1 </dev/null &
video_pid=$!
trap 'kill "$video_pid" 2>/dev/null || true' EXIT
bash "$root/scripts/retry_next_demo.sh"
