"""Record synchronized command and feedback; never publish motion commands."""
import json
import math
import threading
import time
from pathlib import Path
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

rospy.init_node('record_chassis_feedback',anonymous=True)
path=Path.home()/'rosmaster_x3_deploy/logs/chassis_feedback.jsonl'
stream=path.open('w')
lock=threading.Lock()
counts={}
def receive(topic,msg):
    row={'topic':topic,'receive_monotonic':time.monotonic(),'receive_epoch':time.time()}
    if hasattr(msg,'header'):
        row.update(stamp=msg.header.stamp.to_sec(),frame_id=msg.header.frame_id)
    if isinstance(msg,Twist): twist=msg
    elif isinstance(msg,Odometry):
        p=msg.pose.pose.position;q=msg.pose.pose.orientation
        row.update(x=p.x,y=p.y,yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),yaw_variance=msg.pose.covariance[35])
        twist=msg.twist.twist
    else:
        q=msg.orientation
        row.update(q=[q.x,q.y,q.z,q.w],wz=msg.angular_velocity.z)
        twist=None
    if twist: row.update(vx=twist.linear.x,vy=twist.linear.y,wz=twist.angular.z)
    with lock:
        if stream.closed:return
        stream.write(json.dumps(row)+'\n')
        counts[topic]=counts.get(topic,0)+1

subscriptions=[]
for topic,kind in (('/cmd_vel',Twist),('/driver_node/applied_cmd_vel',Twist),('/vel_raw',Twist),('/odom_raw',Odometry),('/odom',Odometry),('/imu/imu_raw',Imu),('/imu/imu_data',Imu)):
    subscriptions.append(rospy.Subscriber(topic,kind,lambda msg,topic=topic:receive(topic,msg),queue_size=100))
try:
    rospy.sleep(10)
finally:
    for subscription in subscriptions:subscription.unregister()
    with lock:stream.close()
print(json.dumps({'path':str(path),'counts':counts}))
