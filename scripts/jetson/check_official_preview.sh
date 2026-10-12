#!/usr/bin/env bash
set -euo pipefail
export LC_ALL=C.UTF-8
source "$HOME/.cargo/env"
task_pkg="$HOME/rosmaster_x3_deploy/primitives/rosmaster_x3_bridge"
export PYTHONPATH="$task_pkg:$task_pkg/rbnx-build/codegen/robonix_mcp_types:$task_pkg/rbnx-build/codegen/proto_gen:$HOME/x3_rosbridge_adapter/scripts"
"$HOME/miniforge3/envs/robonix/bin/python" "$HOME/rosmaster_x3_deploy/scripts/audit_official_preview.py"
