#!/usr/bin/env bash
source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1
tail -90 "$HOME/rosmaster_x3_deploy/live_test/navigation.log"
rosparam get /move_base/DWAPlannerROS
cat "$HOME/yahboomcar_ws/src/yahboomcar_nav/param/common/move_base_params.yaml"
