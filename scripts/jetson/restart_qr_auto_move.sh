#!/usr/bin/env bash
set -eo pipefail
root="$HOME/rosmaster_x3_deploy"
old=$(pgrep -f '^python3 /home/jetson/rosmaster_x3_deploy/scripts/watch_camera_qr.py$' || true)
if [ -n "$old" ]; then kill "$old"; sleep 1; fi
bash "$root/scripts/qr_auto_move.sh"
