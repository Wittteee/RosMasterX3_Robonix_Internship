#!/usr/bin/env bash
set -eo pipefail
root="$HOME/rosmaster_x3_deploy"
if ! pgrep -f '^python3 /home/jetson/rosmaster_x3_deploy/scripts/watch_camera_qr.py$' >/dev/null; then
 nohup python3 "$root/scripts/watch_camera_qr.py" >"$root/logs/camera_qr.log" 2>&1 </dev/null &
 echo "$!" >"$root/live_test/camera_qr.pid"
fi
sleep 3
cat "$root/logs/camera_qr.json"
