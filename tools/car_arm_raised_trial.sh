set -e
export LC_ALL=C.UTF-8
export ROBOT_TYPE=X3
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
source /opt/ros/melodic/setup.bash
source /home/jetson/yahboomcar_ws/devel/setup.bash
source /home/jetson/software/library_ws/devel/setup.bash
TRIAL=/home/jetson/rosmaster_x3_runtime/obstacle1_20261121_raised_trial
python3 - <<'PY'
import os,signal,time
file='/home/jetson/rosmaster_x3_runtime/obstacle1_20261121_5cm/preview_runtime.pid'
pid=int(open(file).read())
args=open('/proc/%d/cmdline'%pid,'rb').read().split(b'\0')
if b'target_navigation.launch' not in args or b'execute:=false' not in args:
    raise RuntimeError('Unexpected launch PID')
os.kill(pid,signal.SIGINT)
for i in range(30):
    if not os.path.exists('/proc/%d'%pid):break
    time.sleep(.5)
PY
python /home/jetson/yahboomcar_ws/src/x3_target_navigation/scripts/check_exclusive_devices.py true true
nohup roslaunch x3_target_navigation target_navigation.launch start_bringup:=true start_astra:=true start_navigation:=true map:=final_site_20261107 obstacle_stop_approach:=true execute:=true > "$TRIAL/runtime.log" 2>&1 < /dev/null &
echo "$!" > "$TRIAL/runtime.pid"
echo "Trial stack PID $!; waiting for separate confirm"
