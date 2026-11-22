set -e
export LC_ALL=C.UTF-8
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
source /opt/ros/melodic/setup.bash
source /home/jetson/rosmaster_x3_runtime/obstacle2_20261121/build_ws/devel/setup.bash
timeout 4 rostopic echo -n 1 /target_navigation/state
timeout 4 rostopic echo -n 1 /target_navigation/motion_permission
timeout 4 rostopic echo -n 1 /odom/twist/twist
python3 - <<'PY'
import os,signal,time
root='/home/jetson/rosmaster_x3_runtime/obstacle2_20261121/hardware_5cm_01'
pid=int(open(root+'/runtime.pid').read())
args=open('/proc/%d/cmdline'%pid,'rb').read().split(b'\0')
assert (root+'/trial.launch').encode() in args,args
os.kill(pid,signal.SIGINT)
for i in range(40):
 try:
  if open('/proc/%d/stat'%pid).read().split()[2]=='Z':break
 except IOError:break
 time.sleep(.25)
else:raise RuntimeError('Trial launch did not exit')
print('5CM_TRIAL_STACK_STOPPED',pid)
PY
