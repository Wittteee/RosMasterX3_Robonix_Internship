from __future__ import print_function
import json
import math
import os
import time

import rospy
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import String, Header
from x3_target_navigation.msg import TargetEstimate

directory = os.environ.get('X3_TRIAL_DIR', '/home/jetson/rosmaster_x3_runtime/obstacle1_20261121_5cm')
rospy.init_node('obstacle1_observer', anonymous=True)
buffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(buffer)
state = {'events': [], 'samples': [], 'state': '', 'target': None,
         'pose': None, 'odom': None, 'command': None, 'applied': None,
         'permission': None, 'scan': None}


def state_cb(message):
    state['state'] = message.data
    state['events'].append({'time': time.time(), 'state': message.data})
    print('STATE', message.data)


def target_cb(message):
    state['target'] = {'x': message.point.x, 'y': message.point.y,
                       'surface_x': message.surface_point.x, 'surface_y': message.surface_point.y,
                       'stamp': message.header.stamp.to_sec(), 'stable': message.stable,
                       'depth': message.depth_m, 'surface_depth': message.surface_depth_m,
                       'frame': message.header.frame_id}


def pose_cb(message):
    state['pose'] = {'x': message.pose.pose.position.x, 'y': message.pose.pose.position.y,
                     'std': math.sqrt(max(message.pose.covariance[0], message.pose.covariance[7])),
                     'stamp': message.header.stamp.to_sec()}


def velocity(message):
    return [message.linear.x, message.linear.y, message.angular.z]


def odom_cb(message):
    state['odom'] = {'x': message.pose.pose.position.x, 'y': message.pose.pose.position.y,
                     'velocity': velocity(message.twist.twist), 'stamp': message.header.stamp.to_sec()}


def scan_cb(message):
    try:
        tf = buffer.lookup_transform('base_footprint', message.header.frame_id,
                                     message.header.stamp, rospy.Duration(.1))
        q = tf.transform.rotation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        tx, ty = tf.transform.translation.x, tf.transform.translation.y
        points = []
        for index, r in enumerate(message.ranges):
            if math.isnan(r) or math.isinf(r) or not message.range_min <= r <= message.range_max:
                continue
            angle = message.angle_min + index*message.angle_increment + yaw
            x, y = tx+r*math.cos(angle), ty+r*math.sin(angle)
            if x > 0 and abs(y) < .20:
                points.append([x-.117, x, y, r])
        points.sort()
        state['scan'] = {'stamp': message.header.stamp.to_sec(), 'range_min': message.range_min,
                         'front_points': points[:16], 'front_gap': points[0][0] if points else None}
    except Exception as error:
        state['scan'] = {'error': str(error)}


subscribers = [
    rospy.Subscriber('/target_navigation/state', String, state_cb, queue_size=100),
    rospy.Subscriber('/target_navigation/fused_target', TargetEstimate, target_cb),
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, pose_cb),
    rospy.Subscriber('/odom', Odometry, odom_cb),
    rospy.Subscriber('/scan', LaserScan, scan_cb),
    rospy.Subscriber('/target_navigation/approach_velocity', Twist,
                     lambda m: state.update(command=velocity(m))),
    rospy.Subscriber('/driver_node/applied_cmd_vel', Twist,
                     lambda m: state.update(applied=velocity(m))),
    rospy.Subscriber('/target_navigation/motion_permission', Header,
                     lambda m: state.update(permission=m.frame_id)),
]
start = time.time()
deadline = start+600
last_print = 0
while not rospy.is_shutdown() and time.time() < deadline:
    row = dict((key, state[key]) for key in ('state', 'target', 'pose', 'odom', 'command', 'applied', 'permission', 'scan'))
    row['time'] = time.time()
    state['samples'].append(row)
    with open(os.path.join(directory, 'observations.jsonl'), 'a') as handle:
        handle.write(json.dumps(row)+'\n')
    with open(os.path.join(directory, 'latest_status.json'), 'w') as handle:
        json.dump(row, handle)
    if time.time()-last_print > 5:
        print(json.dumps(row))
        last_print = time.time()
    rospy.sleep(.1)
with open(os.path.join(directory, 'observations.json'), 'w') as handle:
    json.dump(state, handle, indent=2)
print('Observation complete', len(state['samples']))
