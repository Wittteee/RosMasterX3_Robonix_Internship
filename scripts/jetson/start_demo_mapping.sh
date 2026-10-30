#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/melodic/setup.bash
source "$HOME/software/library_ws/devel/setup.bash"
source "$HOME/yahboomcar_ws/devel/setup.bash"
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1
root="$HOME/rosmaster_x3_deploy/live_test"
cat "$HOME/yahboomcar_ws/src/yahboomcar_nav/launch/library/gmapping.launch"
cat "$HOME/yahboomcar_ws/src/yahboomcar_nav/launch/library/move_base.launch"
cat "$HOME/yahboomcar_ws/src/yahboomcar_nav/launch/library/amcl.launch"
nohup roslaunch yahboomcar_nav gmapping.launch >"$root/mapping.log" 2>&1 </dev/null &
echo "$!" >"$root/mapping.pid"
sleep 12
timeout 8 rostopic echo -n 1 /map/info
mkdir -p "$root/maps"
timeout 15 rosrun map_server map_saver -f "$root/maps/scene_20261028"
cat "$root/maps/scene_20261028.yaml"
