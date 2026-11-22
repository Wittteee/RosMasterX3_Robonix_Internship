from __future__ import print_function
import rospy,tf2_ros,math,json,time,os
from sensor_msgs.msg import LaserScan,Image
from geometry_msgs.msg import PointStamped,PoseWithCovarianceStamped,PoseStamped
from tf2_geometry_msgs import do_transform_point
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetPlan
from std_msgs.msg import String,Header,Float32
from std_srvs.srv import Empty
from x3_target_navigation.msg import TargetEstimate

ROOT=os.environ.get('X3_TRIAL_DIR','/home/jetson/rosmaster_x3_runtime/obstacle2_20261121/hardware_left_01')
rospy.init_node('bypass_live_readiness',anonymous=True)
buffer=tf2_ros.Buffer();listener=tf2_ros.TransformListener(buffer)
data={}
def keep(key):return lambda m:data.update({key:m})
subscriptions=[rospy.Subscriber(topic,kind,keep(key),queue_size=1) for key,topic,kind in [
 ('state','/target_navigation/state',String),('target','/target_navigation/fused_target',TargetEstimate),
 ('pose','/amcl_pose',PoseWithCovarianceStamped),('odom','/odom',Odometry),('scan','/scan',LaserScan),
 ('permission','/target_navigation/motion_permission',Header),('voltage','/voltage',Float32)]]
deadline=time.time()+60
last_nomotion=0.
while time.time()<deadline:
    if time.time()-last_nomotion>1.:
        try:rospy.ServiceProxy('/request_nomotion_update',Empty)()
        except rospy.ServiceException:pass
        last_nomotion=time.time()
    if all(k in data for k in ('state','target','pose','odom','scan','permission','voltage')):
        if data['state'].data.startswith('PREVIEW:') and 0<=(rospy.Time.now()-data['target'].header.stamp).to_sec()<9:break
    time.sleep(.1)
result={'state':data['state'].data if 'state' in data else None,'time':time.time()}
if 'target' in data:
    target=data['target'];point=PointStamped();point.header.frame_id=target.header.frame_id;point.point=target.point
    bottle=do_transform_point(point,buffer.lookup_transform('base_footprint',point.header.frame_id,rospy.Time(0),rospy.Duration(2))).point
    result['target_base']=[bottle.x,bottle.y,bottle.z]
    result['target_age']=(rospy.Time.now()-target.header.stamp).to_sec()
if 'scan' in data:
    scan=data['scan'];tf=buffer.lookup_transform('base_footprint',scan.header.frame_id,scan.header.stamp,rospy.Duration(2))
    q=tf.transform.rotation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    points=[]
    for i,r in enumerate(scan.ranges):
        if math.isnan(r) or math.isinf(r) or not scan.range_min<=r<=scan.range_max:continue
        a=scan.angle_min+i*scan.angle_increment+yaw
        x=tf.transform.translation.x+r*math.cos(a);y=tf.transform.translation.y+r*math.sin(a)
        if abs(x)<=.117 and abs(y)<=.1:continue
        if -.7<x<1.3 and abs(y)<.7:points.append([x,y])
    result['points']=points
    if 'target_base' in result:
        bearing=math.atan2(bottle.y,bottle.x);c=math.cos(bearing);s=math.sin(bearing)
        route=[(c*x+s*y,-s*x+c*y)for x,y in points]
        intruders=[(x,y)for x,y in route if .1<x<math.hypot(bottle.x,bottle.y)-.08 and abs(y)<.15]
        if intruders:
            low=min(y for x,y in intruders);high=max(y for x,y in intruders)
            advance=max(0,min(x for x,y in intruders)-.117-.35+.01)
            far=max(x for x,y in intruders)-advance
            offset=max(.2,high+.2) if low+high<0 else min(-.2,low-.2)
            distance=math.hypot(bottle.x,bottle.y)-advance
            pass_x=far+.267
            result['projected_bypass']={'advance':advance,'low_y':low,'high_y':high,'offset':offset,
               'pass_x':pass_x,'target_distance_after_shift_and_pass':math.hypot(distance-pass_x,offset),
               'required_standoff':.297}
    result['scan_age']=(rospy.Time.now()-scan.header.stamp).to_sec()
if 'odom' in data:
    v=data['odom'].twist.twist;result['velocity']=[v.linear.x,v.linear.y,v.angular.z]
if 'permission' in data:result['permission']=data['permission'].frame_id
if 'voltage'in data:result['voltage']=data['voltage'].data
if 'pose'in data:
    p=data['pose'];q=p.pose.pose.orientation
    result['pose']=[p.pose.pose.position.x,p.pose.pose.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
json.dump(result,open(ROOT+'/readiness.json','w'),indent=2)
brief=dict(result);brief['points_count']=len(brief.pop('points',[]))
print(json.dumps(brief,indent=2))
