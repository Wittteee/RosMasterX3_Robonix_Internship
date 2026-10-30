#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1
root="$HOME/rosmaster_x3_deploy"
python3 "$root/scripts/nav_calibration_policy.py" "$root/scripts/chassis_nav_calibration.json"
python "$root/scripts/check_chassis_health.py"
rosnode kill /demo_motion_guard || true
nohup python3 "$root/scripts/demo_motion_guard.py" >"$root/logs/target_guard.log" 2>&1 </dev/null &
echo "$!" >"$root/live_test/guard.pid"
sleep 2
pkg="$root/primitives/rosmaster_x3_bridge"
export PYTHONPATH="$pkg:$pkg/rosmaster_x3_bridge:$pkg/rbnx-build/codegen/robonix_mcp_types:$pkg/rbnx-build/codegen/proto_gen:$HOME/x3_rosbridge_adapter/scripts"
"$HOME/miniforge3/envs/robonix/bin/python" "$root/scripts/execute_target_parking.py" "$@"
