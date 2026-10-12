#!/usr/bin/env bash
set -euo pipefail
export LC_ALL=C.UTF-8
"$HOME/miniforge3/envs/robonix/bin/python" --version
"$HOME/.cargo/bin/rbnx" --version
echo 'Robonix source revision:'
git -C "$HOME/refs/robonix" rev-parse HEAD
echo 'Safe-move source revision:'
git -C "$HOME/refs/robonix-rosmaster-x3-safe-move" rev-parse HEAD
echo 'Provider source hashes:'
sha256sum "$HOME/refs/robonix-rosmaster-x3-safe-move/jetson/main.py" "$HOME/rosmaster_x3_deploy/primitives/rosmaster_x3_bridge/rosmaster_x3_bridge/main.py"
echo 'Bridge source hashes:'
sha256sum "$HOME/refs/robonix-rosmaster-x3-safe-move/jetson/x3_bridge.py" "$HOME/x3_rosbridge_adapter/scripts/x3_bridge.py"
echo 'CLI entry point:'
readlink -f "$HOME/miniforge3/envs/robonix/bin/move_base"
echo 'ROS communication listeners:'
ss -lnt | grep -E '9090|11311'
