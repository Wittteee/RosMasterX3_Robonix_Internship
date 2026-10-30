#!/usr/bin/env bash
set -eo pipefail
echo 'VENDOR DRIVER'
grep -n -A 18 'def cmd_vel_callback' "$HOME/yahboomcar_ws/src/yahboomcar_bringup/scripts/Mcnamu_driver.py"
echo 'EKF SENSOR CONFIG'
cat "$HOME/yahboomcar_ws/src/yahboomcar_bringup/param/robot_localization.yaml"
echo 'ORIGINAL DWA CONFIG'
cat "$HOME/yahboomcar_ws/src/yahboomcar_nav/param/common/dwa_local_planner_params_X3.yaml"
echo 'LOCAL COSTMAP CONFIG'
cat "$HOME/yahboomcar_ws/src/yahboomcar_nav/param/common/local_costmap_params.yaml"
echo 'CURRENT SENSOR STATUS'
source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1
timeout 4 rostopic echo -n 1 /imu/imu_data
timeout 4 rostopic echo -n 1 /odom
