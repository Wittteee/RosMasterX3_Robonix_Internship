#!/usr/bin/env bash
set -eo pipefail
root="$HOME/rosmaster_x3_deploy"
source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1
pid=$(cat "$root/live_test/sensors.pid")
if ps -p "$pid" -o args= | grep -q "$root/live_test/sensors.launch"; then
 kill -INT "$pid"
 sleep 4
fi
if fuser /dev/ttyUSB1 2>/dev/null; then echo 'Serial occupied; abort'; exit 2; fi
python3 "$root/scripts/motor_pulse_diagnostic.py"
echo 'Pulse finished; motors commanded zero, ROS chassis remains stopped.'
