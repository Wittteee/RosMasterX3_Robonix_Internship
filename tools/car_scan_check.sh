set -e
export ROS_MASTER_URI=http://127.0.0.1:11311
source /opt/ros/melodic/setup.bash
source /home/jetson/yahboomcar_ws/devel/setup.bash
source /home/jetson/software/library_ws/devel/setup.bash
python - <<'PY'
import rospy,tf2_ros,math,json
from sensor_msgs.msg import LaserScan
rospy.init_node('obstacle1_scan_check',anonymous=True)
buffer=tf2_ros.Buffer()
listener=tf2_ros.TransformListener(buffer)
scan=rospy.wait_for_message('/scan',LaserScan,timeout=8)
tf=buffer.lookup_transform('base_footprint',scan.header.frame_id,scan.header.stamp,rospy.Duration(2))
q=tf.transform.rotation
yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
points=[]
for i,r in enumerate(scan.ranges):
    if math.isnan(r) or math.isinf(r) or not scan.range_min<=r<=scan.range_max: continue
    angle=scan.angle_min+i*scan.angle_increment+yaw
    x=tf.transform.translation.x+r*math.cos(angle)
    y=tf.transform.translation.y+r*math.sin(angle)
    if -.1<x<1.2 and abs(y)<.6: points.append([round(x,3),round(y,3)])
points.sort()
result={'laser_transform':str(tf),'near_forward_points':points,'scan_stamp':scan.header.stamp.to_sec()}
print(json.dumps(result,indent=2))
with open('/home/jetson/rosmaster_x3_runtime/obstacle1_20261121_5cm/static_lidar_check.json','w') as out: json.dump(result,out,indent=2)
PY
