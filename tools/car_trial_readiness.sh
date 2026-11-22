set -e
export LC_ALL=C.UTF-8
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
source /opt/ros/melodic/setup.bash
source /home/jetson/yahboomcar_ws/devel/setup.bash
source /home/jetson/software/library_ws/devel/setup.bash
python - <<'PY'
import rospy,tf2_ros,math,json
from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs import do_transform_point
from x3_target_navigation.msg import TargetEstimate
from sensor_msgs.msg import LaserScan
rospy.init_node('obstacle1_readiness',anonymous=True)
buffer=tf2_ros.Buffer()
listener=tf2_ros.TransformListener(buffer)
target=rospy.wait_for_message('/target_navigation/fused_target',TargetEstimate,timeout=12)
point=PointStamped()
point.header.frame_id=target.header.frame_id
point.point=target.point
tf=buffer.lookup_transform('base_footprint',point.header.frame_id,rospy.Time(0),rospy.Duration(2))
base=do_transform_point(point,tf).point
scan=rospy.wait_for_message('/scan',LaserScan,timeout=8)
tf=buffer.lookup_transform('base_footprint',scan.header.frame_id,scan.header.stamp,rospy.Duration(2))
q=tf.transform.rotation
yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
points=[]
for i,r in enumerate(scan.ranges):
    if math.isnan(r) or math.isinf(r) or not scan.range_min<=r<=scan.range_max:continue
    a=scan.angle_min+i*scan.angle_increment+yaw
    x=tf.transform.translation.x+r*math.cos(a)
    y=tf.transform.translation.y+r*math.sin(a)
    if x>0 and abs(y)<.2:points.append([x-.117,x,y])
points.sort()
result={'target_base':[base.x,base.y,base.z],'target_age':(rospy.Time.now()-target.header.stamp).to_sec(),
        'target_bearing':math.atan2(base.y,base.x),'front_points':points[:5]}
print(json.dumps(result,indent=2))
with open('/home/jetson/rosmaster_x3_runtime/obstacle1_20261121_raised_trial/readiness.json','w')as f:json.dump(result,f,indent=2)
PY
