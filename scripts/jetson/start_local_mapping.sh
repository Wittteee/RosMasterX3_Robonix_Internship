#!/usr/bin/env bash
set -eo pipefail
export LC_ALL=C.UTF-8
source /opt/ros/melodic/setup.bash
source "$HOME/software/library_ws/devel/setup.bash"
source "$HOME/yahboomcar_ws/devel/setup.bash"
unset ROS_HOSTNAME
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1 ROBOT_TYPE=X3 RPLIDAR_TYPE=a1
task_root="$HOME/rosmaster_x3_deploy/live_test"
mkdir -p "$task_root"
python3 "$HOME/rosmaster_x3_deploy/scripts/prepare_local_map.py"
# Transfer the serial device from the vendor APP to the ROS driver.
app_pids=$(pgrep -f '^python3 /home/jetson/Rosmaster/rosmaster/rosmaster_main.py$' || true)
if [ -n "$app_pids" ]; then
    printf '%s\n' "$app_pids" | xargs -r kill -TERM
fi
sleep 2
nohup roslaunch "$task_root/sensors.launch" >"$task_root/sensors.log" 2>&1 </dev/null &
echo "$!" >"$task_root/sensors.pid"
sleep 5
rosnode list
python "$HOME/rosmaster_x3_deploy/scripts/check_chassis_health.py"
timeout 8 rostopic echo -n 1 /odom/pose/pose
timeout 8 rostopic echo -n 1 /scan/header
