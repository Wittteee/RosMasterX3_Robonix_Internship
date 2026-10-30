#!/usr/bin/env bash
set -eo pipefail
root="$HOME/rosmaster_x3_deploy"
chmod +x "$root/scripts/pinned_chassis_driver.py"
source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1
if rosnode list | grep -q '^/driver_node$'; then echo 'Existing driver requires review; no duplicate launch'; exit 2; fi
if [ -f "$root/live_test/sensors.pid" ]; then
 old=$(cat "$root/live_test/sensors.pid")
 if ps -p "$old" -o args= | grep -q "$root/live_test/sensors.launch"; then kill -INT "$old"; sleep 4; fi
fi
bash "$root/scripts/start_local_mapping.sh"
bash "$root/scripts/start_demo_mapping.sh"
bash "$root/scripts/start_demo_nav.sh"
for n in $(seq 1 25); do rosservice call /request_nomotion_update '{}' >/dev/null; sleep .5; done
rosparam get /move_base/DWAPlannerROS/max_vel_x
timeout 5 rostopic echo -n 1 /amcl_pose
