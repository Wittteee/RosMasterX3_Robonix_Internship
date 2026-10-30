#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1
root="$HOME/rosmaster_x3_deploy"
bash "$root/scripts/start_demo_nav.sh"
for n in $(seq 1 20); do rosservice call /request_nomotion_update '{}' >/dev/null; sleep 0.5; done
python "$root/scripts/check_chassis_health.py"
nohup python3 "$root/scripts/demo_motion_guard.py" >"$root/logs/guard_live.log" 2>&1 </dev/null &
echo "$!" >"$root/live_test/guard.pid"
sleep 2
pkg="$root/primitives/rosmaster_x3_bridge"
export PYTHONPATH="$pkg:$pkg/rosmaster_x3_bridge:$pkg/rbnx-build/codegen/robonix_mcp_types:$pkg/rbnx-build/codegen/proto_gen:$HOME/x3_rosbridge_adapter/scripts"
"$HOME/miniforge3/envs/robonix/bin/python" "$root/scripts/execute_demo_30cm.py"
