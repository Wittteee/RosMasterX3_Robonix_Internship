#!/usr/bin/env bash
set -eo pipefail
root="$HOME/rosmaster_x3_deploy"
if fuser /dev/ttyUSB1 2>/dev/null; then echo 'Serial occupied; no motion'; exit 2; fi
old=$(pgrep -f '^python3 /home/jetson/rosmaster_x3_deploy/scripts/watch_camera_qr.py$' || true)
if [ -n "$old" ]; then kill "$old"; sleep 1; fi
nohup python3 "$root/scripts/watch_camera_qr.py" >"$root/logs/camera_qr.log" 2>&1 </dev/null &
camera_pid=$!
echo "$camera_pid" >"$root/live_test/camera_qr.pid"
trap 'kill "$camera_pid" 2>/dev/null || true' EXIT
sleep 2
python3 "$root/scripts/qr_auto_move.py"
