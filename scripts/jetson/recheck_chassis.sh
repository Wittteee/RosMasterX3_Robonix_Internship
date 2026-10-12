#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1
python "$HOME/rosmaster_x3_deploy/scripts/check_chassis_health.py"
