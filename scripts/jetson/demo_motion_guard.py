"""Independent ROS goal observer and cancellation guard; never publishes velocity."""
import json
import math
import threading
import time
from pathlib import Path
import rospy
from actionlib_msgs.msg import GoalID
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

rospy.init_node('demo_motion_guard', anonymous=False)
lock = threading.RLock()
state = {'goal':None, 'started':None, 'odom':None, 'origin':None,
         'last_odom':0, 'wheel_speed':0, 'last_wheel':0, 'reason':None,
         'peak_raw_linear':0,'raw_linear_path_m':0}
path = Path.home()/'rosmaster_x3_deploy/logs/guard_30cm.json'
cancel = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
def odom(msg):
    with lock:
        p = msg.pose.pose.position
        state['odom'] = (p.x,p.y)
        state['last_odom'] = time.monotonic()
def wheel(msg):
    with lock:
        now=time.monotonic()
        linear=math.hypot(msg.linear.x,msg.linear.y)
        if state['goal'] is not None:
            state['peak_raw_linear']=max(state['peak_raw_linear'],linear)
            state['raw_linear_path_m']+=linear*min(.2,max(0,now-state['last_wheel']))
        state['wheel_speed'] = max(math.hypot(msg.linear.x,msg.linear.y),abs(msg.angular.z))
        state['last_wheel'] = now
def goal(msg):
    with lock:
        if state['goal'] is not None:
            cancel.publish(GoalID(id=msg.goal_id.id))
            return
        state['goal'] = msg.goal_id.id
        state['started'] = time.monotonic()
        state['origin'] = state['odom']
rospy.Subscriber('/odom',Odometry,odom,queue_size=10)
rospy.Subscriber('/vel_raw',Twist,wheel,queue_size=10)
rospy.Subscriber('/move_base/goal',MoveBaseActionGoal,goal,queue_size=10)
rate = rospy.Rate(20)
while not rospy.is_shutdown():
    with lock:
        now = time.monotonic()
        ready = now-state['last_odom']<1 and now-state['last_wheel']<1
        report = dict(state,ready=ready,heartbeat=now)
        if state['goal'] is not None:
            elapsed = now-state['started']
            distance = math.hypot(*(a-b for a,b in zip(state['odom'],state['origin']))) if state['odom'] and state['origin'] else None
            reason = ('telemetry_stale' if not ready else
                      'missing_start_pose' if distance is None else
                      'distance_limit' if distance>0.45 else
                      'time_limit' if elapsed>20 else None)
            if reason:
                state['reason'] = state['reason'] or reason
            if state['reason']:
                cancel.publish(GoalID(id=state['goal']))
            report.update(elapsed_sec=elapsed,displacement_m=distance,reason=state['reason'])
        tmp = path.with_suffix('.tmp')
        tmp.write_text(json.dumps(report))
        tmp.replace(path)
    rate.sleep()
