#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/melodic/setup.bash
source "$HOME/software/library_ws/devel/setup.bash"
source "$HOME/yahboomcar_ws/devel/setup.bash"
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1 ROBOT_TYPE=X3
root="$HOME/rosmaster_x3_deploy/live_test"
python3 "$HOME/rosmaster_x3_deploy/scripts/prepare_demo_nav.py"
pid=$(cat "$root/mapping.pid")
if ps -p "$pid" -o args= | grep -q 'roslaunch yahboomcar_nav gmapping.launch'; then
 kill -INT "$pid"
 sleep 3
fi
if rosnode list | grep -q '^/slam_gmapping$'; then echo 'SLAM still running; abort'; exit 2; fi
nohup roslaunch "$root/demo_navigation.launch" >"$root/navigation.log" 2>&1 </dev/null &
echo "$!" >"$root/navigation.pid"
sleep 15
rosnode list
timeout 8 rostopic echo -n 1 /amcl_pose
rosservice list | grep make_plan
