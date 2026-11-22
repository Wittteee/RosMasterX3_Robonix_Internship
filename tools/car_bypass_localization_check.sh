set -e
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
source /opt/ros/melodic/setup.bash
source /home/jetson/rosmaster_x3_runtime/obstacle2_20261121/build_ws/devel/setup.bash
python - <<'PY'
import rospy,math,time,json,tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
rospy.init_node('bypass_localization_check',anonymous=True)
buf=tf2_ros.Buffer();listener=tf2_ros.TransformListener(buf)
data={};rows=[]
def angle(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
subs=[rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped,lambda m:data.update(pose=m)),rospy.Subscriber('/odom',Odometry,lambda m:data.update(odom=m))]
for i in range(12):
    time.sleep(1)
    if 'pose' not in data or 'odom' not in data:continue
    p=data['pose'];o=data['odom'];v=o.twist.twist
    try:t=buf.lookup_transform('map','odom',rospy.Time(0));map_yaw=angle(t.transform.rotation)
    except Exception:map_yaw=None
    row={'time':time.time(),'amcl':[p.pose.pose.position.x,p.pose.pose.position.y,angle(p.pose.pose.orientation)],
         'odom':[o.pose.pose.position.x,o.pose.pose.position.y,angle(o.pose.pose.orientation)],
         'std':math.sqrt(max(p.pose.covariance[0],p.pose.covariance[7])),
         'yaw_std':math.sqrt(max(0,p.pose.covariance[35])),
         'velocity':[v.linear.x,v.linear.y,v.angular.z],'map_odom_yaw':map_yaw}
    rows.append(row)
json.dump(rows,open('/home/jetson/rosmaster_x3_runtime/obstacle2_20261121/hardware_left_01/localization_check.json','w'),indent=2)
print(json.dumps(rows,indent=2))
PY
