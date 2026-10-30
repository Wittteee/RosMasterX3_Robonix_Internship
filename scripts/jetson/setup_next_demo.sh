#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1
root="$HOME/rosmaster_x3_deploy"
python3 -c 'import rospy'
nohup python3 "$root/scripts/demo_motion_guard.py" __name:=demo_guard_isolated /move_base/goal:=/guard_test/goal /move_base/cancel:=/guard_test/cancel /odom:=/guard_test/odom /vel_raw:=/guard_test/wheel >"$root/logs/guard_test.log" 2>&1 </dev/null &
guard_pid=$!
trap 'kill "$guard_pid" 2>/dev/null || true' EXIT
python3 "$root/scripts/test_demo_guard.py"
