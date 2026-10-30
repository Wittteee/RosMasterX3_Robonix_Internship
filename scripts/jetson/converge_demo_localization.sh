#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1
for n in $(seq 1 20); do
 timeout 3 rosservice call /request_nomotion_update '{}' >/dev/null
 sleep 0.5
done
timeout 5 rostopic echo -n 1 /amcl_pose
export X3_MOVE_OPTIONS_MIN_RADIUS_M=0.1 X3_MOVE_OPTIONS_MAX_RADIUS_M=0.1
"$HOME/rosmaster_x3_deploy/scripts/move_base" --dry-run
