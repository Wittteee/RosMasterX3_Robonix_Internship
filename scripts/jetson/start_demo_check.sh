#!/usr/bin/env bash
set -eo pipefail
bash "$HOME/rosmaster_x3_deploy/scripts/start_ros_readonly.sh"
bash "$HOME/rosmaster_x3_deploy/scripts/start_local_mapping.sh"
