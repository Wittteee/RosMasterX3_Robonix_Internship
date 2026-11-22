set -e
export LC_ALL=C.UTF-8
export ROBOT_TYPE=X3
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
source /opt/ros/melodic/setup.bash
source /home/jetson/yahboomcar_ws/devel/setup.bash
source /home/jetson/software/library_ws/devel/setup.bash
TASK_DIR=/home/jetson/rosmaster_x3_runtime/obstacle1_20261121_5cm
# Shut down the known launch gracefully. The base shutdown callback sends zero.
python3 - <<'PY'
import os,signal,time
path='/home/jetson/rosmaster_x3_runtime/obstacle1_20261121_5cm/runtime.pid'
pid=int(open(path).read())
args=open('/proc/%d/cmdline'%pid,'rb').read().split(b'\0')
if b'target_navigation.launch' not in args:
    raise RuntimeError('Refusing to signal unrelated PID')
os.kill(pid,signal.SIGINT)
for _ in range(30):
    if not os.path.exists('/proc/%d'%pid): break
    time.sleep(.5)
print('Stopped test launch',pid)
PY
python /home/jetson/yahboomcar_ws/src/x3_target_navigation/scripts/check_exclusive_devices.py true true
nohup roslaunch x3_target_navigation target_navigation.launch start_bringup:=true start_astra:=true start_navigation:=true map:=final_site_20261107 obstacle_stop_approach:=true execute:=false > "$TASK_DIR/preview_runtime.log" 2>&1 < /dev/null &
echo "$!" > "$TASK_DIR/preview_runtime.pid"
printf 'Preview-only launch PID '
cat "$TASK_DIR/preview_runtime.pid"
