#!/usr/bin/env bash
set -eo pipefail
if fuser /dev/ttyUSB1 2>/dev/null; then echo 'Serial occupied; no movement sent'; exit 2; fi
python3 "$HOME/rosmaster_x3_deploy/scripts/direct_auto_move.py"
