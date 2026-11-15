#!/usr/bin/env python3
"""Capture real RGB-D target samples for camera/extrinsic validation.

This tool never sends a navigation goal.  Put a bottle at a measured distance,
run this script, and compare the fused target distance with the tape measure.
The resulting JSON is evidence for calibration; it does not silently change
TF or camera parameters.
"""
from __future__ import print_function

import argparse
import json
import math
import os
import time

import rospy

from geometry_msgs.msg import PoseWithCovarianceStamped
from x3_target_navigation.msg import TargetEstimate


class Capture(object):
    def __init__(self, expected_m, samples, output):
        self.expected_m = expected_m
        self.samples_needed = samples
        self.output = output
        self.pose = None
        self.rows = []
        self.target_sub = rospy.Subscriber(
            '/target_navigation/raw_target', TargetEstimate,
            self.target_callback, queue_size=20)
        self.pose_sub = rospy.Subscriber(
            '/amcl_pose', PoseWithCovarianceStamped,
            self.pose_callback, queue_size=10)

    def pose_callback(self, message):
        self.pose = message.pose.pose.position

    def target_callback(self, message):
        if not message.stable and message.sample_count < 1:
            return
        if not message.header.stamp or not message.header.frame_id:
            return
        if not all(math.isfinite(v) for v in
                   (message.point.x, message.point.y, message.point.z,
                    message.depth_m, message.confidence)):
            return
        row = {
            'stamp': message.header.stamp.to_sec(),
            'frame_id': message.header.frame_id,
            'x': message.point.x,
            'y': message.point.y,
            'z': message.point.z,
            'planar_distance_m': math.hypot(message.point.x, message.point.y),
            'depth_m': message.depth_m,
            'depth_valid_ratio': message.depth_valid_ratio,
            'confidence': message.confidence,
            'sample_count': message.sample_count,
            'stable': bool(message.stable),
        }
        self.rows.append(row)
        rospy.loginfo('calibration sample %d/%d: map planar %.3f m, depth %.3f m',
                      len(self.rows), self.samples_needed,
                      row['planar_distance_m'], row['depth_m'])
        if len(self.rows) >= self.samples_needed:
            rospy.signal_shutdown('enough calibration samples captured')

    def result(self):
        distances = [row['planar_distance_m'] for row in self.rows]
        mean = sum(distances) / len(distances) if distances else None
        rms_error = None
        if distances and self.expected_m is not None:
            rms_error = math.sqrt(sum((x - self.expected_m) ** 2
                                      for x in distances) / len(distances))
        return {
            'expected_distance_m': self.expected_m,
            'sample_count': len(self.rows),
            'mean_planar_distance_m': mean,
            'rms_error_m': rms_error,
            'samples': self.rows,
            'note': 'Measurement evidence only; no TF or calibration parameter was changed.',
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--expected-distance', type=float, required=True,
                        help='tape-measured target-to-base distance in metres')
    parser.add_argument('--samples', type=int, default=10)
    parser.add_argument('--output', default='rgbd_calibration.json')
    args = parser.parse_args()
    if args.expected_distance <= 0 or args.samples < 3:
        parser.error('expected distance must be positive and samples >= 3')
    rospy.init_node('capture_rgbd_calibration', anonymous=True)
    capture = Capture(args.expected_distance, args.samples, args.output)
    rospy.loginfo('read-only capture started; no navigation command will be sent')
    rospy.spin()
    result = capture.result()
    directory = os.path.dirname(os.path.abspath(args.output))
    if directory and not os.path.isdir(directory):
        os.makedirs(directory)
    with open(args.output, 'w') as stream:
        json.dump(result, stream, indent=2, sort_keys=True)
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
