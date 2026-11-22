set -e
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
source /opt/ros/melodic/setup.bash
source /home/jetson/rosmaster_x3_runtime/obstacle2_20261121/build_ws/devel/setup.bash
cat /home/jetson/yahboomcar_ws/src/yahboomcar_bringup/param/robot_localization.yaml
python - <<'PY'
import rospy,math,time,json
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
rospy.init_node('bypass_sensor_diagnose',anonymous=True)
data={}
def angle(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def imu(name):return lambda m:data.setdefault(name,[]).append([time.time(),angle(m.orientation),m.angular_velocity.z])
def odom(name):return lambda m:data.setdefault(name,[]).append([time.time(),angle(m.pose.pose.orientation),m.twist.twist.angular.z])
subs=[rospy.Subscriber('/imu/imu_raw',Imu,imu('imu_raw')),rospy.Subscriber('/imu/imu_data',Imu,imu('imu_data')),
 rospy.Subscriber('/odom_raw',Odometry,odom('wheel_odom')),rospy.Subscriber('/odom',Odometry,odom('fused_odom')),
 rospy.Subscriber('/vel_raw',Twist,lambda m:data.setdefault('vel_raw',[]).append([time.time(),m.linear.x,m.linear.y,m.angular.z])),
 rospy.Subscriber('/target_navigation/motion_permission',Header,lambda m:data.update(permission=m.frame_id))]
time.sleep(8)
summary={}
for key,rows in data.items():
 if isinstance(rows,list):summary[key]={'count':len(rows),'first':rows[0],'last':rows[-1],'mean_last':sum(r[-1]for r in rows)/len(rows)}
 else:summary[key]=rows
print(json.dumps(summary,indent=2))
json.dump(data,open('/home/jetson/rosmaster_x3_runtime/obstacle2_20261121/hardware_left_01/sensor_diagnose.json','w'))
PY
