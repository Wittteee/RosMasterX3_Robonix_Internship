#!/usr/bin/env bash
set -eo pipefail
mkdir -p "$HOME/rosmaster_x3_deploy/logs"
pgrep -f '^python3 /home/jetson/Rosmaster/rosmaster/rosmaster_main.py$' | xargs -r kill -TERM
sleep 2
if fuser /dev/ttyUSB1 /dev/ttyUSB2 2>/dev/null; then
 echo 'Serial still occupied; aborting queries.'
 exit 2
fi
python3 "$HOME/rosmaster_x3_deploy/scripts/probe_raw_reply.py"
echo 'No motor command sent; MakerControl remains paused.'
