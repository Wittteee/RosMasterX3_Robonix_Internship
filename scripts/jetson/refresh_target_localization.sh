#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1
rosrun dynamic_reconfigure dynparam set /amcl '{"update_min_d":0.02,"update_min_a":0.03}'
for n in $(seq 1 20); do rosservice call /request_nomotion_update '{}' >/dev/null; sleep .3; done
timeout 5 rostopic echo -n 1 /amcl_pose
timeout 5 rostopic echo -n 1 /odom/pose/pose
