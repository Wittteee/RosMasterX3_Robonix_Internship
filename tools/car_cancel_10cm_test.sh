set -e
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
source /opt/ros/melodic/setup.bash
source /home/jetson/rosmaster_x3_runtime/obstacle2_20261121/build_ws/devel/setup.bash
rosservice call /target_navigation/cancel || true
timeout 4 rostopic echo -n 1 /target_navigation/state || true
timeout 4 rostopic echo -n 1 /odom/twist/twist || true
python3 - <<'PY'
import os,signal,time
root='/home/jetson/rosmaster_x3_runtime/obstacle2_20261121/hardware_10cm_01'
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
print('10CM_TRIAL_STACK_STOPPED',pid)
PY
