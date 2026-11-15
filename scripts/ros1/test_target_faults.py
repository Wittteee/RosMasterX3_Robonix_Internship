#!/usr/bin/env python3
"""Attended fault-path checks for cancel and timeout handling.

This harness only calls the existing confirm/cancel services.  It does not
invent a goal and cannot test an unreachable target unless the operator first
prepares one through the normal preview path.
"""
from __future__ import print_function

import argparse
import json
import time

import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger


class State(object):
    def __init__(self):
        self.value = ''
        rospy.Subscriber('/target_navigation/state', String,
                         self.callback, queue_size=20)

    def callback(self, message):
        self.value = message.data


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--cancel-after', type=float, default=0.0,
                        help='seconds after confirm to request cancel; 0 disables')
    parser.add_argument('--wait', type=float, default=20.0)
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args()
    rospy.init_node('test_target_faults', anonymous=True)
    if not args.execute:
        print('Safety stop: pass --execute only for an attended fault test.')
        return 2
    rospy.wait_for_service('/target_navigation/confirm', timeout=15.0)
    confirm = rospy.ServiceProxy('/target_navigation/confirm', Trigger)
    cancel = rospy.ServiceProxy('/target_navigation/cancel', Trigger)
    state = State()
    response = confirm()
    result = {'confirm': {'success': response.success,
                          'message': response.message}}
    if response.success and args.cancel_after > 0:
        rospy.sleep(args.cancel_after)
        cancel_response = cancel()
        result['cancel'] = {'success': cancel_response.success,
                            'message': cancel_response.message}
    deadline = time.time() + args.wait
    while time.time() < deadline and not rospy.is_shutdown():
        if state.value.startswith(('PREEMPTED:', 'FAILED:', 'SUCCEEDED:')):
            break
        rospy.sleep(0.1)
    result['final_state'] = state.value
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    main()
