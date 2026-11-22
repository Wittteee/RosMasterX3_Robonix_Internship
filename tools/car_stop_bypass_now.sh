set -e
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
source /opt/ros/melodic/setup.bash
source /home/jetson/rosmaster_x3_runtime/obstacle2_20261121/build_ws/devel/setup.bash
python - <<'PY'
import rospy,time,os,signal,json
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
rospy.init_node('operator_stop_bypass_now',anonymous=True)
zero=rospy.Publisher('/target_navigation/approach_velocity',Twist,queue_size=1)
permit=rospy.Publisher('/target_navigation/motion_permission',Header,queue_size=1)
time.sleep(.3)
try:print('CANCEL',rospy.ServiceProxy('/target_navigation/cancel',Trigger)())
except Exception as e:print('Cancel service',e)
for i in range(5):
 h=Header();h.stamp=rospy.Time.now();h.frame_id='STOP';permit.publish(h);zero.publish(Twist());time.sleep(.05)
m=rospy.wait_for_message('/odom',Odometry,timeout=4)
v=m.twist.twist
print('STOP_ODOMETRY',v.linear.x,v.linear.y,v.angular.z)
root='/home/jetson/rosmaster_x3_runtime/obstacle2_20261121/hardware_left_01'
json.dump({'operator_stop':True,'time':time.time(),'velocity':[v.linear.x,v.linear.y,v.angular.z]},open(root+'/operator_stop.json','w'),indent=2)
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
print('TRIAL_STACK_STOPPED',pid)
PY
