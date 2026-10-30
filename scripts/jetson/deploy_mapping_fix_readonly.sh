#!/usr/bin/env bash
set -eo pipefail
root="$HOME/rosmaster_x3_deploy"
source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1
if rosnode list | grep -q '^/move_base$'; then echo 'Navigation active; refuse driver replacement'; exit 2; fi
python3 -m py_compile "$root/scripts/pinned_chassis_driver.py" "$root/scripts/velocity_mapping.py" "$root/scripts/nav_calibration_policy.py" "$root/scripts/record_chassis_feedback.py"
python3 - <<'PY'
import json,os,sys
sys.path.insert(0,os.path.expanduser('~/rosmaster_x3_deploy/scripts'))
from velocity_mapping import map_velocity,command_expired
from nav_calibration_policy import require_calibration
assert map_velocity(.09,0,.15)==(.09,0,.15)
assert map_velocity(0,0,0)==(0,0,0)
assert command_expired(1,1.5)
try: require_calibration(json.load(open(os.path.expanduser('~/rosmaster_x3_deploy/scripts/chassis_nav_calibration.json'))))
except ValueError: print('PASS: navigation blocked pending physical calibration')
else: raise RuntimeError('Unexpected unmeasured calibration accepted')
print('PASS: metric velocity mapping and command timeout policy')
PY
PYTHONPATH="$root/scripts" python -c 'from velocity_mapping import map_velocity,monotonic; assert map_velocity(.09,0,.15)==(.09,0,.15); assert monotonic()>0; print("PASS: vendor ROS interpreter compatibility")'
if [ -f "$root/live_test/sensors.pid" ]; then
 old=$(cat "$root/live_test/sensors.pid")
 if ps -p "$old" -o args= | grep -q "$root/live_test/sensors.launch"; then kill -INT "$old"; sleep 4; fi
fi
bash "$root/scripts/start_local_mapping.sh"
rostopic info /driver_node/applied_cmd_vel
python3 "$root/scripts/record_chassis_feedback.py"
