#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1
python "$HOME/rosmaster_x3_deploy/scripts/check_chassis_health.py"
test "$(rosparam get /move_base/DWAPlannerROS/max_vel_x)" = '0.05'
test "$(rosparam get /move_base/recovery_behavior_enabled)" = 'false'
pkg="$HOME/rosmaster_x3_deploy/primitives/rosmaster_x3_bridge"
export PYTHONPATH="$pkg:$pkg/rosmaster_x3_bridge:$pkg/rbnx-build/codegen/robonix_mcp_types:$pkg/rbnx-build/codegen/proto_gen:$HOME/x3_rosbridge_adapter/scripts"
"$HOME/miniforge3/envs/robonix/bin/python" "$HOME/rosmaster_x3_deploy/scripts/execute_demo_step.py"
