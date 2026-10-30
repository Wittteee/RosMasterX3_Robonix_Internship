#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1
rosnode list
rostopic info /vel_raw
timeout 4 rostopic echo -n 1 /vel_raw
python3 - <<'PY'
import inspect
from Rosmaster_Lib import Rosmaster
for name in ('get_motor_encoder','get_motion_data','set_car_type','set_car_motion'):
 print(inspect.getsource(getattr(Rosmaster,name)))
PY
