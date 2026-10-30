#!/usr/bin/env bash
set -eo pipefail
root="$HOME/rosmaster_x3_deploy"
bash "$root/scripts/start_demo_nav.sh"
source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1
for n in $(seq 1 20); do rosservice call /request_nomotion_update '{}' >/dev/null; sleep 0.5; done
pkg="$root/primitives/rosmaster_x3_bridge"
export PYTHONPATH="$pkg:$pkg/rosmaster_x3_bridge:$pkg/rbnx-build/codegen/robonix_mcp_types:$pkg/rbnx-build/codegen/proto_gen:$HOME/x3_rosbridge_adapter/scripts"
"$HOME/miniforge3/envs/robonix/bin/python" "$root/demo_project/scripts/run_visual_nav.py" --observation-file "$root/logs/camera_qr.json" --provider-dir "$pkg" --max-radius 0.3 --output-dir "$root/logs/real_vision"
