"""One real, authorized function-2 trial with a separate stop supervisor."""
from __future__ import print_function
import json,math,os,time,threading
import rospy,tf2_ros,cv2,numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist,PoseWithCovarianceStamped,PointStamped
from tf2_geometry_msgs import do_transform_point
from sensor_msgs.msg import LaserScan,Image
from std_msgs.msg import String,Header
from std_srvs.srv import Trigger,Empty
from x3_target_navigation.msg import TargetEstimate
ROOT=os.environ.get('X3_TRIAL_DIR','/home/jetson/rosmaster_x3_runtime/obstacle2_20261121/hardware_left_01')
rospy.init_node('bypass_hardware_supervisor',anonymous=True)
buf=tf2_ros.Buffer();listener=tf2_ros.TransformListener(buf)
data={'state':'','permission':'','command':[0.,0.,0.],'applied':[0.,0.,0.],
      'scan_time':0.,'points':[],'odom':None,'pose':None,'target':None,'image':None}
report={'events':[],'samples':[],'confirm_success':False}
publisher=rospy.Publisher('/target_navigation/approach_velocity',Twist,queue_size=1)
cancel=rospy.ServiceProxy('/target_navigation/cancel',Trigger)
active=False
def angle(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def velocity(m):return [m.linear.x,m.linear.y,m.angular.z]
def snapshot(name):
 m=data['image']
 if m is None:return
 a=np.frombuffer(m.data,dtype=np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3)
 if m.encoding=='rgb8':a=cv2.cvtColor(a,cv2.COLOR_RGB2BGR)
 if m.encoding in ('rgb8','bgr8'):cv2.imwrite(ROOT+'/'+name+'.png',a)
def state_cb(m):
 data['state']=m.data;report['events'].append({'time':time.time(),'state':m.data})
 if active and 'initial'in report:
  wrong = 'BYPASS_LEFT_' if report['initial']['side']=='RIGHT' else 'BYPASS_RIGHT_'
  if wrong in m.data:
   data['wrong_side']=True
   publisher.publish(Twist())
 print('STATE',m.data)
 snapshot('phase_%02d'%len(report['events']))
def odom_cb(m):
 data['odom']={'x':m.pose.pose.position.x,'y':m.pose.pose.position.y,'yaw':angle(m.pose.pose.orientation),
  'velocity':velocity(m.twist.twist),'stamp':m.header.stamp.to_sec()}
def pose_cb(m):
 data['pose']={'x':m.pose.pose.position.x,'y':m.pose.pose.position.y,'yaw':angle(m.pose.pose.orientation),
  'std':math.sqrt(max(m.pose.covariance[0],m.pose.covariance[7])),'stamp':m.header.stamp.to_sec()}
def scan_cb(m):
 try:
  tf=buf.lookup_transform('base_footprint',m.header.frame_id,m.header.stamp,rospy.Duration(.05));yaw=angle(tf.transform.rotation)
  points=[]
  for i,r in enumerate(m.ranges):
   if math.isnan(r)or math.isinf(r)or not m.range_min<=r<=m.range_max:continue
   a=m.angle_min+i*m.angle_increment+yaw;x=tf.transform.translation.x+r*math.cos(a);y=tf.transform.translation.y+r*math.sin(a)
   if (abs(x)<=.117 and abs(y)<=.1)or(-.120<=x<-.117 and abs(y)<=.025):continue
   points.append([x,y])
  data['points']=points;data['scan_time']=m.header.stamp.to_sec()
 except Exception as e:data['scan_error']=str(e)
subs=[rospy.Subscriber(t,k,cb,queue_size=1)for t,k,cb in [
 ('/target_navigation/state',String,state_cb),('/target_navigation/motion_permission',Header,lambda m:data.update(permission=m.frame_id)),
 ('/target_navigation/approach_velocity',Twist,lambda m:data.update(command=velocity(m))),
 ('/driver_node/applied_cmd_vel',Twist,lambda m:data.update(applied=velocity(m))),
 ('/odom',Odometry,odom_cb),('/amcl_pose',PoseWithCovarianceStamped,pose_cb),('/scan',LaserScan,scan_cb),
 ('/target_navigation/fused_target',TargetEstimate,lambda m:data.update(target=m)),
 ('/camera/rgb/image_raw',Image,lambda m:data.update(image=m))]]
def stop(reason):
 global active
 report['supervisor_stop']=reason
 print('SUPERVISOR STOP',reason)
 try:cancel()
 except Exception as e:report['cancel_error']=str(e)
 for i in range(4):publisher.publish(Twist());time.sleep(.05)
 active=False
try:
 deadline=time.time()+60;last_nomotion=0.
 while time.time()<deadline and not rospy.is_shutdown():
  if time.time()-last_nomotion>1.:
   try:
    rospy.wait_for_service('/request_nomotion_update',timeout=.2)
    rospy.ServiceProxy('/request_nomotion_update',Empty)()
   except (rospy.ROSException,rospy.ServiceException):pass
   last_nomotion=time.time()
  t=data['target'];o=data['odom']
  if data['state'].startswith('PREVIEW:')and t is not None and o is not None and data['pose'] is not None:
   age=rospy.get_time()-t.header.stamp.to_sec()
   if 0<=age<8 and 0<=rospy.get_time()-data['scan_time']<.4 and math.hypot(*o['velocity'][:2])<.003 and abs(o['velocity'][2])<.01:
    try:
     target_tf=buf.lookup_transform('base_footprint',t.header.frame_id,rospy.Time(0),rospy.Duration(.1))
     break
    except Exception:pass
  time.sleep(.1)
 else:raise RuntimeError('No fresh target preview with stopped odometry')
 p=PointStamped();p.header.frame_id=t.header.frame_id;p.point=t.point
 target=do_transform_point(p,target_tf).point
 assert .6<target.x<1.5 and abs(math.atan2(target.y,target.x))<.30,'Bottle not in forward trial bounds'
 bearing=math.atan2(target.y,target.x);c=math.cos(bearing);s=math.sin(bearing)
 intruders=[(c*x+s*y,-s*x+c*y)for x,y in data['points']if .15<x<target.x-.08 and abs(-s*x+c*y)<.15]
 assert intruders,'No intervening partial obstruction detected'
 low=min(y for x,y in intruders);high=max(y for x,y in intruders)
 assert not(low<=-.1 and high>=.1)and abs(low+high)>.02,'Full or ambiguous blockage'
 side=-1 if low+high>0 else 1
 assert side==-1,'Current trial expects left intrusion and right bypass'
 assert not any(-.18<x<1.1 and -.50<y<-.16 for x,y in data['points']),'Right bypass lane occupied'
 report['initial']={'target_base':[target.x,target.y,target.z],'side':'RIGHT','intrusion_y':[low,high],
  'odom':dict(o),'pose':dict(data['pose']),'wheel_heading_trial':True}
 snapshot('before')
 response=rospy.ServiceProxy('/target_navigation/confirm',Trigger)()
 report['confirm_success']=response.success;report['confirm_message']=response.message
 print('CONFIRM',response.success,response.message)
 if not response.success:raise RuntimeError(response.message)
 active=True;started=time.time();terminal=None;lastprint=0.;lastpose=dict(data['pose']);lastpostime=time.time();shift_anchor=None
 while time.time()-started<130 and not rospy.is_shutdown():
  now=time.time();o=data['odom'];pose=data['pose'];state=data['state']
  row={'time':now,'state':state,'permission':data['permission'],'command':list(data['command']),
    'applied':list(data['applied']),'odom':dict(o),'pose':dict(pose),'scan_age':rospy.get_time()-data['scan_time']}
  row['body_clearance']=min([math.hypot(max(abs(x)-.117,0.),max(abs(y)-.1,0.))for x,y in data['points']]or[99.])
  row['front_gap']=min([x-.117 for x,y in data['points']if x>0. and abs(y)<.15]or[99.])
  report['samples'].append(row)
  with open(ROOT+'/trial_samples.jsonl','a')as f:f.write(json.dumps(row)+'\n')
  if now-lastprint>1.:print('SAMPLE',json.dumps(row));lastprint=now
  if state.startswith(('FAILED:','SUCCEEDED:','PREEMPTED:')):
   if terminal is None:terminal=now;report['terminal_state']=state
   if now-terminal>2.:active=False;break
  else:
   if data.get('wrong_side'):stop('Bypass side changed from independently checked clear side');break
   if row['scan_age']>.6 or rospy.get_time()-o['stamp']>.6:stop('Sensor stream stale');break
   if row['body_clearance']<.025:stop('Obstacle inside 2.5 cm body clearance');break
   travel=math.hypot(o['x']-report['initial']['odom']['x'],o['y']-report['initial']['odom']['y'])
   if travel>1.5:stop('Trial travel limit');break
   if 'BYPASS_RIGHT_SHIFT'in state:
    if shift_anchor is None:shift_anchor=dict(o)
    initial_yaw=shift_anchor['yaw'];dx=o['x']-shift_anchor['x'];dy=o['y']-shift_anchor['y']
    if -math.sin(initial_yaw)*dx+math.cos(initial_yaw)*dy>.025:stop('Actual lateral displacement is toward blocked side');break
   if now-lastpostime>.4:
    jump=math.hypot(pose['x']-lastpose['x'],pose['y']-lastpose['y'])
    yawjump=abs(math.atan2(math.sin(pose['yaw']-lastpose['yaw']),math.cos(pose['yaw']-lastpose['yaw'])))
    if jump>.08 or yawjump>.15:stop('Localization jump');break
    lastpose=dict(pose);lastpostime=now
  time.sleep(.05)
 if active:stop('Trial timeout')
 report['final_odom']=data['odom'];report['final_permission']=data['permission'];report['final_state']=data['state']
 snapshot('after')
except Exception as e:
 report['error']=str(e);print('ERROR',str(e))
finally:
 if active:stop('Supervisor exiting')
 for i in range(3):publisher.publish(Twist());time.sleep(.05)
 json.dump(report,open(ROOT+'/trial_report.json','w'),indent=2)
 print('REPORT',json.dumps(dict((k,v)for k,v in report.items()if k!='samples'),indent=2))
