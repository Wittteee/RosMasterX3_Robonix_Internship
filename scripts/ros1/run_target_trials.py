#!/usr/bin/env python3
"""Run attended, one-confirmation-at-a-time target navigation trials.

The script records state transitions and AMCL poses.  It never publishes
cmd_vel and refuses to execute unless --execute is supplied.  Between trials
the operator must reposition the robot/target and press Enter.
"""
from __future__ import print_function

import argparse
import json
import os
import time

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger


class TrialRunner(object):
    def __init__(self):
        self.state = ''
        self.pose = None
        self.events = []
        rospy.Subscriber('/target_navigation/state', String,
                         self.state_callback, queue_size=20)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped,
                         self.pose_callback, queue_size=10)

    def state_callback(self, message):
        self.state = message.data
        self.events.append({'time': rospy.get_time(), 'state': self.state})

    def pose_callback(self, message):
        p = message.pose.pose.position
        self.pose = {'x': p.x, 'y': p.y, 'stamp': message.header.stamp.to_sec()}

    def wait_for_terminal(self, timeout):
        deadline = time.time() + timeout
        while not rospy.is_shutdown() and time.time() < deadline:
            if any(self.state.startswith(prefix)
                   for prefix in ('SUCCEEDED:', 'FAILED:', 'PREEMPTED:')):
                return self.state
            rospy.sleep(0.1)
        return self.state or 'TIMEOUT: no state received'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--trials', type=int, default=1)
    parser.add_argument('--timeout', type=float, default=120.0)
    parser.add_argument('--output', default='target_trials.json')
    parser.add_argument('--execute', action='store_true',
                        help='required before any confirm service call')
    args = parser.parse_args()
    if args.trials < 1 or args.timeout <= 0:
        parser.error('invalid trial count or timeout')
    rospy.init_node('run_target_trials', anonymous=True)
    runner = TrialRunner()
    rospy.wait_for_service('/target_navigation/confirm', timeout=15.0)
    confirm = rospy.ServiceProxy('/target_navigation/confirm', Trigger)
    results = []
    for index in range(args.trials):
        if not args.execute:
            print('Trial %d preview only: add --execute for an attended real trial.'
                  % (index + 1))
            results.append({'trial': index + 1, 'mode': 'preview_only'})
            continue
        input('Place the bottle and clear the floor for trial %d, then press Enter: '
              % (index + 1))
        runner.events = []
        before = runner.pose
        response = confirm()
        row = {'trial': index + 1, 'confirm_success': response.success,
               'confirm_message': response.message, 'start_pose': before,
               'events': runner.events}
        if response.success:
            row['terminal_state'] = runner.wait_for_terminal(args.timeout)
        else:
            row['terminal_state'] = runner.state
        row['end_pose'] = runner.pose
        results.append(row)
        print(json.dumps(row, indent=2, sort_keys=True))
    with open(args.output, 'w') as stream:
        json.dump({'trials': results, 'note': 'Attended trials; no cmd_vel published.'},
                  stream, indent=2, sort_keys=True)


if __name__ == '__main__':
    main()
