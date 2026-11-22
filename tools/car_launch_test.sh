set -e
export LC_ALL=C.UTF-8
export ROBOT_TYPE=X3
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
source /opt/ros/melodic/setup.bash
source /home/jetson/yahboomcar_ws/devel/setup.bash
source /home/jetson/software/library_ws/devel/setup.bash
TASK_DIR=/home/jetson/rosmaster_x3_runtime/obstacle1_20261121_5cm
python3 - <<'PY'
import os, signal, time
for name in os.listdir('/proc'):
    if not name.isdigit(): continue
    try:
        args = open('/proc/%s/cmdline' % name, 'rb').read().split(b'\0')
    except IOError:
        continue
    if b'/home/jetson/Rosmaster/rosmaster/rosmaster_main.py' in args:
        print('Closing original camera/base service PID', name)
        os.kill(int(name), signal.SIGINT)
        time.sleep(2)
PY
python /home/jetson/yahboomcar_ws/src/x3_target_navigation/scripts/check_exclusive_devices.py true true
nohup roslaunch x3_target_navigation target_navigation.launch start_bringup:=true start_astra:=true start_navigation:=true map:=final_site_20261107 obstacle_stop_approach:=true execute:=true > "$TASK_DIR/runtime.log" 2>&1 < /dev/null &
echo "$!" > "$TASK_DIR/runtime.pid"
printf 'Launch started PID '
cat "$TASK_DIR/runtime.pid"
