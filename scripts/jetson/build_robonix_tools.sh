#!/usr/bin/env bash
set -euo pipefail
export LC_ALL=C.UTF-8
source "$HOME/.cargo/env"
export CARGO_BUILD_JOBS=1
cd "$HOME/refs/robonix"
if [[ ! -f capabilities/lib/common_interfaces/geometry_msgs/msg/Pose.msg ]]; then
    git submodule update --init --recursive -- capabilities/lib/common_interfaces capabilities/lib/rcl_interfaces
    if git ls-files --stage capabilities/lib/unique_identifier_msgs | grep -q '^160000'; then
        git submodule update --init --recursive -- capabilities/lib/unique_identifier_msgs
    fi
fi
cargo build --locked -p robonix-codegen -p robonix-cli
install -m 755 target/debug/robonix-codegen "$HOME/.cargo/bin/robonix-codegen"
install -m 755 target/debug/rbnx "$HOME/.cargo/bin/rbnx"
rbnx setup "$HOME/refs/robonix"
export PATH="$HOME/miniforge3/envs/robonix/bin:$PATH"
export RBNX_CODEGEN_PYTHON="$HOME/miniforge3/envs/robonix/bin/python"
task_pkg="$HOME/rosmaster_x3_deploy/primitives/rosmaster_x3_bridge"
RBNX_PACKAGE_ROOT="$task_pkg" bash "$task_pkg/scripts/build.sh"
bash "$HOME/refs/robonix-rosmaster-x3-safe-move/jetson/scripts/install_move_base_command.sh"
"$HOME/rosmaster_x3_deploy/scripts/move_base" --help
