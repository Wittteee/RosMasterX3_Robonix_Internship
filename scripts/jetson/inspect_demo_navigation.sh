#!/usr/bin/env bash
source /opt/ros/melodic/setup.bash
source "$HOME/yahboomcar_ws/devel/setup.bash"
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1
rosnode list
rostopic list
find "$HOME/yahboomcar_ws/src/yahboomcar_nav/launch" "$HOME/yahboomcar_ws/src/yahboomcar_slam/launch" -maxdepth 1 -type f
tail -50 "$HOME/rosmaster_x3_deploy/live_test/sensors.log"
sed -n '1,180p' "$HOME/yahboomcar_ws/src/yahboomcar_bringup/scripts/Mcnamu_driver.py"
