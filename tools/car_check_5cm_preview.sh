set -e
export LC_ALL=C.UTF-8
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
source /opt/ros/melodic/setup.bash
source /home/jetson/rosmaster_x3_runtime/obstacle2_20261121/build_ws/devel/setup.bash
python - <<'PY'
from __future__ import print_function
import rospy,time,json,math
from sensor_msgs.msg import LaserScan,Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String,Header,Float32
from std_srvs.srv import Empty
from x3_target_navigation.msg import TargetEstimate
rospy.init_node('five_cm_preview_check',anonymous=True)
data={};times={};scan_times=[]
def keep(key):
 def cb(m):
  data[key]=m;times[key]=time.time()
  if key=='scan':scan_times.append(time.time())
 return cb
subs=[rospy.Subscriber(t,k,keep(key),queue_size=1)for key,t,k in[
 ('state','/target_navigation/state',String),('permission','/target_navigation/motion_permission',Header),
 ('scan','/scan',LaserScan),('rgb','/camera/rgb/image_raw',Image),('depth','/camera/depth_registered/image_raw',Image),
 ('odom','/odom',Odometry),('pose','/amcl_pose',PoseWithCovarianceStamped),('target','/target_navigation/fused_target',TargetEstimate),
 ('voltage','/voltage',Float32)]]
last_update=0.;deadline=time.time()+55
while time.time()<deadline and not rospy.is_shutdown():
 if time.time()-last_update>1.:
  try:
   rospy.wait_for_service('/request_nomotion_update',timeout=.2)
   rospy.ServiceProxy('/request_nomotion_update',Empty)()
  except (rospy.ROSException,rospy.ServiceException):pass
  last_update=time.time()
 if 'state'in data and data['state'].data.startswith('PREVIEW:') and all(k in data for k in('scan','rgb','depth','odom','pose','target','permission','voltage')):
  if all(time.time()-times[k]<2 for k in('scan','rgb','depth','odom','pose')) and (rospy.Time.now()-data['target'].header.stamp).to_sec()<10:break
 time.sleep(.1)
result={'state':data['state'].data if 'state'in data else 'NO_STATE',
 'execute_enabled':rospy.get_param('/target_navigation/target_task_manager/execute_enabled'),
 'obstacle_bypass_enabled':rospy.get_param('/target_navigation/target_task_manager/obstacle_bypass_enabled'),
 'bypass_detection_distance_m':rospy.get_param('/target_navigation/target_task_manager/bypass_detection_distance_m'),
 'permission':data['permission'].frame_id if 'permission'in data else None,
 'stream_age_sec':dict((k,time.time()-v)for k,v in times.items())}
if 'odom'in data:
 v=data['odom'].twist.twist;result['velocity']=[v.linear.x,v.linear.y,v.angular.z]
if 'target'in data:
 t=data['target'];result['target']={'label':t.label,'stable':t.stable,'point':[t.point.x,t.point.y,t.point.z],'age':(rospy.Time.now()-t.header.stamp).to_sec()}
if 'voltage'in data:result['voltage']=data['voltage'].data
if len(scan_times)>2:result['scan_hz']=(len(scan_times)-1)/(scan_times[-1]-scan_times[0])
assert result['execute_enabled'] is False and result['permission']=='STOP',result
assert result['bypass_detection_distance_m']==.05 and result['obstacle_bypass_enabled'] is True,result
assert max(map(abs,result.get('velocity',[99.])))<.01,result
json.dump(result,open('/home/jetson/rosmaster_x3_runtime/obstacle2_20261121/hardware_5cm_preview/status.json','w'),indent=2)
print(json.dumps(result,indent=2))
PY
