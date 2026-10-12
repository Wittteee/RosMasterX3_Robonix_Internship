#!/usr/bin/env bash
set -eo pipefail
task_pid="$(cat "$HOME/rosmaster_x3_deploy/live_test/sensors.pid")"
task_command="$(ps -p "$task_pid" -o args= || true)"
case "$task_command" in
    *roslaunch*'/home/jetson/rosmaster_x3_deploy/live_test/sensors.launch'*) kill -INT "$task_pid" ;;
    *) echo 'Task sensor launch is not running; skipping shutdown.' ;;
esac
sleep 3
python3 "$HOME/rosmaster_x3_deploy/scripts/probe_raw_reply.py"
lsusb -v -d 0483:5750 2>/dev/null | grep -E 'iProduct|iManufacturer|bInterfaceClass' || true
