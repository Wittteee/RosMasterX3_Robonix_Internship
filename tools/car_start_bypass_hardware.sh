set -e
export LC_ALL=C.UTF-8
export ROBOT_TYPE=X3
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
TASK=/home/jetson/rosmaster_x3_runtime/obstacle2_20261121
TRIAL="$TASK/hardware_left_01"
mkdir -p "$TRIAL"
source /opt/ros/melodic/setup.bash
source /home/jetson/yahboomcar_ws/devel/setup.bash
source /home/jetson/software/library_ws/devel/setup.bash
python - <<'PY'
import rospy,math,json,os,signal,time
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
root='/home/jetson/rosmaster_x3_runtime/obstacle2_20261121/hardware_left_01'
rospy.init_node('bypass_save_initial_pose',anonymous=True)
state=rospy.wait_for_message('/target_navigation/state',String,timeout=5)
odom=rospy.wait_for_message('/odom',Odometry,timeout=5)
v=odom.twist.twist
assert math.hypot(v.linear.x,v.linear.y)<.003 and abs(v.angular.z)<.01,'Car must be stopped'
assert state.data.startswith('FAILED: BLOCKED_NO_SAFE_PATH'),state.data
p=rospy.wait_for_message('/amcl_pose',PoseWithCovarianceStamped,timeout=5)
q=p.pose.pose.orientation
yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
initial={'x':p.pose.pose.position.x,'y':p.pose.pose.position.y,'yaw':yaw,'stamp':p.header.stamp.to_sec()}
json.dump(initial,open(root+'/initial_pose.json','w'),indent=2)
launch='''<launch>
  <include file="$(find x3_target_navigation)/launch/bottle_bypass.launch">
    <arg name="execute" value="true"/>
  </include>
  <param name="/amcl/initial_pose_x" value="{x}"/>
  <param name="/amcl/initial_pose_y" value="{y}"/>
  <param name="/amcl/initial_pose_a" value="{yaw}"/>
  <param name="/amcl/initial_cov_xx" value="0.0004"/>
  <param name="/amcl/initial_cov_yy" value="0.0004"/>
  <param name="/amcl/initial_cov_aa" value="0.0025"/>
</launch>'''.format(**initial)
open(root+'/trial.launch','w').write(launch)
pid=18185
args=open('/proc/%d/cmdline'%pid,'rb').read().split(b'\0')
assert b'target_navigation.launch' in args and b'execute:=true' in args,args
print('Preserved pose',initial,'stopping verified original launch',pid)
os.kill(pid,signal.SIGINT)
for i in range(60):
    try:
        status=open('/proc/%d/stat'%pid).read().split()[2]
        if status=='Z':break
    except IOError:break
    time.sleep(.5)
else:raise RuntimeError('Original launch has not exited')
PY
source "$TASK/build_ws/devel/setup.bash"
sha256sum -c "$TASK/function1_binary.sha256"
python "$TASK/build_ws/src/x3_target_navigation/scripts/check_exclusive_devices.py" true true
nohup roslaunch "$TRIAL/trial.launch" > "$TRIAL/runtime.log" 2>&1 < /dev/null &
echo "$!" > "$TRIAL/runtime.pid"
printf 'Function2 trial stack started, PID %s; no confirm issued\n' "$!"
