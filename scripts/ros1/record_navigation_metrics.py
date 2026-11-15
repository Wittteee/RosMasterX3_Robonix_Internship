#!/usr/bin/env python3
"""Record physical target-navigation metrics without commanding the robot."""
from __future__ import print_function

import argparse
import json
import math
import time

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion
from x3_target_navigation.msg import TargetEstimate


class Recorder(object):
    def __init__(self):
        self.state = ''
        self.events = []
        self.amcl = None
        self.odom = None
        self.target = None
        rospy.Subscriber('/target_navigation/state', String,
                         self.state_cb, queue_size=50)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped,
                         self.amcl_cb, queue_size=20)
        rospy.Subscriber('/odom', Odometry, self.odom_cb, queue_size=50)
        rospy.Subscriber('/target_navigation/fused_target', TargetEstimate,
                         self.target_cb, queue_size=20)

    def state_cb(self, message):
        self.state = message.data
        self.events.append({'time': rospy.get_time(), 'state': self.state})

    def amcl_cb(self, message):
        p = message.pose.pose
        yaw = euler_from_quaternion((p.orientation.x, p.orientation.y,
                                     p.orientation.z, p.orientation.w))[2]
        self.amcl = {'x': p.position.x, 'y': p.position.y,
                     'yaw_rad': yaw, 'stamp': message.header.stamp.to_sec(),
                     'x_variance': message.pose.covariance[0],
                     'y_variance': message.pose.covariance[7]}

    def odom_cb(self, message):
        p = message.pose.pose
        self.odom = {'x': p.position.x, 'y': p.position.y,
                     'stamp': message.header.stamp.to_sec()}

    def target_cb(self, message):
        self.target = {'x': message.point.x, 'y': message.point.y,
                       'z': message.point.z, 'depth_m': message.depth_m,
                       'confidence': message.confidence,
                       'stamp': message.header.stamp.to_sec()}

    def result(self):
        result = {'terminal_state': self.state, 'events': self.events,
                  'final_amcl': self.amcl, 'final_odom': self.odom,
                  'final_target': self.target}
        if self.amcl and self.target:
            result['final_target_to_base_m'] = math.hypot(
                self.target['x'] - self.amcl['x'],
                self.target['y'] - self.amcl['y'])
        return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--wait', type=float, default=150.0)
    parser.add_argument('--output', default='navigation_metrics.json')
    args = parser.parse_args()
    rospy.init_node('record_navigation_metrics', anonymous=True)
    recorder = Recorder()
    deadline = time.time() + args.wait
    while not rospy.is_shutdown() and time.time() < deadline:
        if recorder.state.startswith(('SUCCEEDED:', 'FAILED:', 'PREEMPTED:')):
            break
        rospy.sleep(0.1)
    result = recorder.result()
    with open(args.output, 'w') as stream:
        json.dump(result, stream, indent=2, sort_keys=True)
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
