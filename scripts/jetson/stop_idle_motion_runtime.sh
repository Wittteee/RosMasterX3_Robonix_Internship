#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1
rosnode kill /demo_motion_guard || true
root="$HOME/rosmaster_x3_deploy/live_test"
if [ -f "$root/sensors.pid" ]; then
 pid=$(cat "$root/sensors.pid")
 if ps -p "$pid" -o args= | grep -q "$root/sensors.launch"; then kill -INT "$pid"; sleep 4; fi
fi
rosnode list
echo 'Idle chassis and lidar stopped after read-only validation; SSH and ROS bridge retained.'
