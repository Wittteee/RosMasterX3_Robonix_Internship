#!/usr/bin/env bash
set -eo pipefail
root="$HOME/rosmaster_x3_deploy"
bash "$root/scripts/finish_demo_check.sh"
mkdir -p "$root/demo_project"
tar -xf "$root/project_45e3216.tar" -C "$root/demo_project"
nohup python3 "$root/scripts/watch_camera_qr.py" >"$root/logs/camera_qr.log" 2>&1 </dev/null &
echo "$!" >"$root/live_test/camera_qr.pid"
sleep 3
cat "$root/logs/camera_qr.json"
