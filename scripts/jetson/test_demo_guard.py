"""Exercise the independent guard on isolated ROS topics, without motion."""
import json
import time
from pathlib import Path
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseActionGoal
from actionlib_msgs.msg import GoalID
rospy.init_node('test_demo_guard')
seen=[]
rospy.Subscriber('/guard_test/cancel',GoalID,lambda m:seen.append(m.id))
odom=rospy.Publisher('/guard_test/odom',Odometry,queue_size=10)
wheel=rospy.Publisher('/guard_test/wheel',Twist,queue_size=10)
goals=rospy.Publisher('/guard_test/goal',MoveBaseActionGoal,queue_size=10)
time.sleep(2)
for _ in range(10):
    odom.publish(Odometry());wheel.publish(Twist());time.sleep(.1)
assert not seen, 'guard canceled before a goal'
goal=MoveBaseActionGoal();goal.goal_id.id=''
goals.publish(goal)
# Stop publishing odometry: stale telemetry must cancel the observed goal.
deadline=time.time()+4
while time.time()<deadline and not seen: time.sleep(.1)
assert '' in seen, 'guard failed to cancel empty-ID goal on stale telemetry'
print('PASS: no pre-goal cancellation; actual goal ID canceled on stale telemetry')
