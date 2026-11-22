set -e
export LC_ALL=C.UTF-8
export ROBOT_TYPE=X3
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
TASK=/home/jetson/rosmaster_x3_runtime/obstacle2_20261121
TRIAL="$TASK/hardware_5cm_01"
mkdir -p "$TRIAL"
source /opt/ros/melodic/setup.bash
source "$TASK/build_ws/devel/setup.bash"
python - <<'PY'
import rospy,os,signal,time,math,json,xml.etree.ElementTree as ET
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
root='/home/jetson/rosmaster_x3_runtime/obstacle2_20261121'
rospy.init_node('arm_five_cm_trial',anonymous=True)
assert rospy.get_param('/target_navigation/target_task_manager/execute_enabled') is False
assert rospy.get_param('/target_navigation/target_task_manager/bypass_detection_distance_m')==.05
assert rospy.wait_for_message('/target_navigation/motion_permission',Header,timeout=5).frame_id=='STOP'
v=rospy.wait_for_message('/odom',Odometry,timeout=5).twist.twist
assert math.hypot(v.linear.x,v.linear.y)<.003 and abs(v.angular.z)<.01
p=rospy.wait_for_message('/amcl_pose',PoseWithCovarianceStamped,timeout=5);q=p.pose.pose.orientation
seed={'x':p.pose.pose.position.x,'y':p.pose.pose.position.y,'a':math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))}
tree=ET.parse(root+'/hardware_5cm_preview/preview.launch')
tree.getroot().find("include/arg[@name='execute']").set('value','true')
for name,value in seed.items():tree.getroot().find("param[@name='/amcl/initial_pose_%s']"%name).set('value',str(value))
tree.write(root+'/hardware_5cm_01/trial.launch')
json.dump({'initial_pose':seed,'execute_enabled':True,'front_trigger_m':.05,'wheel_heading_trial':True},open(root+'/hardware_5cm_01/configuration.json','w'),indent=2)
pid=int(open(root+'/hardware_5cm_preview/runtime.pid').read())
args=open('/proc/%d/cmdline'%pid,'rb').read().split(b'\0')
assert (root+'/hardware_5cm_preview/preview.launch').encode() in args,args
os.kill(pid,signal.SIGINT)
for i in range(60):
 try:
  if open('/proc/%d/stat'%pid).read().split()[2]=='Z':break
 except IOError:break
 time.sleep(.25)
else:raise RuntimeError('Preview launch did not stop')
PY
python "$TASK/build_ws/src/x3_target_navigation/scripts/check_exclusive_devices.py" true true
nohup roslaunch "$TRIAL/trial.launch" > "$TRIAL/runtime.log" 2>&1 < /dev/null &
echo "$!" > "$TRIAL/runtime.pid"
echo '5 cm motion-capable stack started; supervisor must confirm before motion'
