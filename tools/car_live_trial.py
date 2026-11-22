"""One authorized low-speed trial, with independent monitoring and a cancel guard."""
from __future__ import print_function
import json
import math
import os
import time

import rospy
import tf2_ros
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from x3_target_navigation.msg import TargetEstimate

ROOT = '/home/jetson/rosmaster_x3_runtime/obstacle1_20261121_raised_trial'
rospy.init_node('obstacle1_live_supervisor', anonymous=True)
buffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(buffer)
data = {'state': '', 'scan_time': 0., 'gap': None, 'target': None,
        'odom': None, 'command': [0., 0.], 'applied': [0., 0.], 'events': []}


def state_cb(message):
    data['state'] = message.data
    data['events'].append({'time': time.time(), 'state': message.data})
    print('STATE', message.data)


def scan_cb(scan):
    try:
        tf = buffer.lookup_transform('base_footprint', scan.header.frame_id,
                                     scan.header.stamp, rospy.Duration(.1))
        q = tf.transform.rotation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        gaps = []
        for i, r in enumerate(scan.ranges):
            if math.isnan(r) or math.isinf(r) or not scan.range_min <= r <= scan.range_max:
                continue
            angle = scan.angle_min+i*scan.angle_increment+yaw
            x = tf.transform.translation.x+r*math.cos(angle)
            y = tf.transform.translation.y+r*math.sin(angle)
            if x > 0. and abs(y) < .20:
                gaps.append(x-.117)
        data['gap'] = min(gaps) if gaps else None
        data['scan_time'] = scan.header.stamp.to_sec()
    except Exception as error:
        data['scan_error'] = str(error)


def odom_cb(message):
    data['odom'] = {'x': message.pose.pose.position.x, 'y': message.pose.pose.position.y,
                    'vx': message.twist.twist.linear.x, 'vy': message.twist.twist.linear.y,
                    'wz': message.twist.twist.angular.z, 'stamp': message.header.stamp.to_sec()}


subscriptions = [
    rospy.Subscriber('/target_navigation/state', String, state_cb),
    rospy.Subscriber('/scan', LaserScan, scan_cb, queue_size=1),
    rospy.Subscriber('/target_navigation/fused_target', TargetEstimate,
                     lambda m: data.update(target=m)),
    rospy.Subscriber('/odom', Odometry, odom_cb),
    rospy.Subscriber('/target_navigation/approach_velocity', Twist,
                     lambda m: data.update(command=[m.linear.x, m.angular.z])),
    rospy.Subscriber('/driver_node/applied_cmd_vel', Twist,
                     lambda m: data.update(applied=[m.linear.x, m.angular.z])),
]
cancel = rospy.ServiceProxy('/target_navigation/cancel', Trigger)
report = {'confirm_success': False, 'events': data['events'], 'samples': []}
running = False
try:
    deadline = time.time()+90
    while time.time() < deadline and not rospy.is_shutdown():
        target = data['target']
        if (data['state'].startswith('PREVIEW:') and target is not None and
                data['odom'] is not None and data['gap'] is not None and
                rospy.get_time()-data['scan_time'] < .5 and
                0.15 < data['gap'] < .65 and
                0. <= rospy.get_time()-target.header.stamp.to_sec() <= 9.):
            break
        time.sleep(.1)
    else:
        raise RuntimeError('No fresh target preview plus detectable intervening obstacle')
    point = PointStamped()
    point.header.frame_id, point.point = target.header.frame_id, target.point
    tf = buffer.lookup_transform('base_footprint', point.header.frame_id, rospy.Time(0), rospy.Duration(2))
    bottle = do_transform_point(point, tf).point
    if abs(math.atan2(bottle.y,bottle.x)) > .35 or bottle.x < data['gap']+.117+.05:
        raise RuntimeError('Obstacle/target not in the required forward order')
    report['initial_target_base'] = [bottle.x,bottle.y,bottle.z]
    report['initial_gap_m'] = data['gap']
    report['initial_odom'] = dict(data['odom'])
    rospy.wait_for_service('/target_navigation/confirm',timeout=5)
    response = rospy.ServiceProxy('/target_navigation/confirm',Trigger)()
    report['confirm_success'], report['confirm_message'] = response.success, response.message
    print('CONFIRM',response.success,response.message)
    if not response.success: raise RuntimeError('Confirmation rejected: '+response.message)
    running = True
    started = time.time()
    terminal_time = None
    last_print = 0.
    while not rospy.is_shutdown() and time.time()-started < 65.:
        row = dict((key,data[key]) for key in ['state','gap','scan_time','odom','command','applied'])
        row['time'] = time.time()
        report['samples'].append(row)
        with open(ROOT+'/trial_samples.jsonl','a') as stream:
            stream.write(json.dumps(row)+'\n')
        if row['command'][0] > 0. and 'first_forward' not in report:
            report['first_forward'] = row
        if 'first_forward' in report and row['command'] == [0.,0.] and 'first_zero' not in report:
            report['first_zero'] = row
        if time.time()-last_print > 1.:
            print('SAMPLE',json.dumps(row))
            last_print = time.time()
        if data['state'].startswith(('FAILED:','SUCCEEDED:','PREEMPTED:')):
            if terminal_time is None:
                terminal_time = time.time()
                report['terminal_state'] = data['state']
            if time.time()-terminal_time >= 3.:
                running = False
                break
        elif data['state'].startswith('NAVIGATING:'):
            if (rospy.get_time()-data['scan_time'] > .6 or data['gap'] is None or
                    data['gap'] < .035):
                report['supervisor_cancel'] = 'sensor unavailable or gap below 3.5 cm'
                cancel()
        time.sleep(.05)
    if running:
        report['supervisor_cancel'] = 'trial timeout'
    report['final_gap_m'] = data['gap']
    report['final_odom'] = data['odom']
    report['final_applied'] = data['applied']
    report['held_stopped'] = (terminal_time is not None and
        all(abs(v) < 1e-6 for v in data['applied']))
    if data['odom'] is not None:
        report['odom_travel_m'] = math.hypot(data['odom']['x']-report['initial_odom']['x'],
                                            data['odom']['y']-report['initial_odom']['y'])
except Exception as error:
    report['error'] = str(error)
finally:
    if running:
        try: cancel()
        except Exception as error: report['cancel_error'] = str(error)
    with open(ROOT+'/trial_report.json','w') as stream:
        json.dump(report,stream,indent=2)
    print('REPORT',json.dumps(dict((k,v) for k,v in report.items() if k!='samples'),indent=2))
