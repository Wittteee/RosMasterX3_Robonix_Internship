#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1
root="$HOME/rosmaster_x3_deploy/live_test"
timeout 5 rostopic echo -n 1 /odom/twist/twist || true
pid=$(cat "$root/navigation.pid")
if ps -p "$pid" -o args= | grep -q "$root/demo_navigation.launch"; then
 kill -INT "$pid"
 sleep 3
fi
rosnode list
echo 'Navigation stopped; sensors and ROS bridge retained; MakerControl paused.'
