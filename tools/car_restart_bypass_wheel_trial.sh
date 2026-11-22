set -e
export LC_ALL=C.UTF-8
export ROBOT_TYPE=X3
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
TASK=/home/jetson/rosmaster_x3_runtime/obstacle2_20261121
TRIAL="$TASK/hardware_left_01"
source /opt/ros/melodic/setup.bash
source "$TASK/build_ws/devel/setup.bash"
python - <<'PY'
import os,signal,time,json,rospy,math
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
root='/home/jetson/rosmaster_x3_runtime/obstacle2_20261121/hardware_left_01'
rospy.init_node('bypass_restart_check',anonymous=True)
v=rospy.wait_for_message('/odom',Odometry,timeout=5).twist.twist
assert math.hypot(v.linear.x,v.linear.y)<.003 and abs(v.angular.z)<.01
assert rospy.wait_for_message('/target_navigation/motion_permission',Header,timeout=5).frame_id=='STOP'
pid=int(open(root+'/runtime.pid').read())
args=open('/proc/%d/cmdline'%pid,'rb').read().split(b'\0')
assert (root+'/trial.launch').encode() in args,args
os.kill(pid,signal.SIGINT)
for i in range(60):
 try:
  if open('/proc/%d/stat'%pid).read().split()[2]=='Z':break
 except IOError:break
 time.sleep(.5)
else:raise RuntimeError('Launch failed to stop')
os.rename(root+'/runtime.log',root+'/runtime_before_restart_%d.log'%int(time.time()))
# The physical car has remained stationary; use the heading recorded before
# the old gyro drift accumulated. AMCL will still have to fit the live scan.
initial=json.load(open(root+'/initial_pose.json'));initial['yaw']=1.122
launch='''<launch>
 <include file="$(find x3_target_navigation)/launch/bottle_bypass.launch"><arg name="execute" value="true"/></include>
 <param name="/amcl/initial_pose_x" value="{x}"/>
 <param name="/amcl/initial_pose_y" value="{y}"/>
 <param name="/amcl/initial_pose_a" value="{yaw}"/>
 <param name="/amcl/initial_cov_xx" value="0.0025"/>
 <param name="/amcl/initial_cov_yy" value="0.0025"/>
 <param name="/amcl/initial_cov_aa" value="0.01"/>
 <!-- Trial-only workaround: raw wheel odometry is stable; the IMU gyro is biased.
      Keep x/y/yaw-rate wheel measurements; exclude drifting IMU orientation/rate. -->
 <rosparam param="/ekf_localization/imu0_config">[false,false,false,false,false,false,false,false,false,false,false,false,false,false,false]</rosparam>
</launch>'''.format(**initial)
open(root+'/trial.launch','w').write(launch)
json.dump({'trial_only_wheel_heading':True,'imu_bias_unresolved':True,'initial_pose':initial},open(root+'/trial_configuration.json','w'),indent=2)
PY
python "$TASK/build_ws/src/x3_target_navigation/scripts/check_exclusive_devices.py" true true
nohup roslaunch "$TRIAL/trial.launch" > "$TRIAL/runtime.log" 2>&1 < /dev/null &
echo "$!" > "$TRIAL/runtime.pid"
echo 'Wheel-heading trial ready; waiting for target/route checks and separate confirm'
