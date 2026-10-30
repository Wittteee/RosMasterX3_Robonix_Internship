#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1
rosnode kill /demo_motion_guard || true
bash "$HOME/rosmaster_x3_deploy/scripts/setup_next_demo.sh"
bash "$HOME/rosmaster_x3_deploy/scripts/finish_demo_check.sh"
bash "$HOME/rosmaster_x3_deploy/scripts/run_next_demo.sh"
