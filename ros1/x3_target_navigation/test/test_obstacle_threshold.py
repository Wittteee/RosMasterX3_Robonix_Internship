#!/usr/bin/env python
"""Real task-manager callbacks on an isolated rostest master, with no driver."""
from __future__ import print_function
import math
import threading
import time
import unittest

import rospy
import rostest
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Float32, Header, String
from std_srvs.srv import Trigger
from x3_target_navigation.msg import TargetEstimate


class ObstacleThreshold(unittest.TestCase):
    def test_five_cm_trigger_and_latch(self):
        self.state = ''
        self.commands = []
        self.permission = ''
        self.gap = .30
        self.running = True
        subscriptions = [
            rospy.Subscriber('/target_navigation/state', String,
                             lambda m: setattr(self, 'state', m.data)),
            rospy.Subscriber('/target_navigation/approach_velocity', Twist,
                             lambda m: self.commands.append((m.linear.x, m.angular.z))),
            rospy.Subscriber('/target_navigation/motion_permission', Header,
                             lambda m: setattr(self, 'permission', m.frame_id)),
        ]
        broadcasters = tf2_ros.StaticTransformBroadcaster()
        transforms = []
        for parent, child, x, yaw in [('map', 'base_footprint', 0., 0.),
                                      ('base_footprint', 'laser', .0435, math.pi),
                                      ('base_footprint', 'camera_rgb_optical_frame', 0., 0.)]:
            transform = TransformStamped()
            transform.header.stamp = rospy.Time.now()
            transform.header.frame_id = parent
            transform.child_frame_id = child
            transform.transform.translation.x = x
            transform.transform.rotation.z = math.sin(yaw / 2.)
            transform.transform.rotation.w = math.cos(yaw / 2.)
            transforms.append(transform)
        broadcasters.sendTransform(transforms)
        publishers = {
            'voltage': rospy.Publisher('/voltage', Float32, queue_size=1),
            'edition': rospy.Publisher('/edition', Float32, queue_size=1),
            'scan': rospy.Publisher('/scan', LaserScan, queue_size=1),
            'rgb': rospy.Publisher('/camera/rgb/image_raw', Image, queue_size=1),
            'depth': rospy.Publisher('/camera/depth_registered/image_raw', Image, queue_size=1),
            'pose': rospy.Publisher('/amcl_pose', PoseWithCovarianceStamped, queue_size=1),
            'odom': rospy.Publisher('/odom', Odometry, queue_size=1),
            'target': rospy.Publisher('/target_navigation/fused_target', TargetEstimate, queue_size=1),
        }

        def send_sensors():
            while self.running and not rospy.is_shutdown():
                stamp = rospy.Time.now()
                publishers['voltage'].publish(Float32(11.8))
                publishers['edition'].publish(Float32(2.3))
                scan = LaserScan()
                scan.header.stamp, scan.header.frame_id = stamp, 'laser'
                scan.angle_min, scan.angle_increment = math.pi, math.pi
                scan.range_min, scan.range_max = .01, 12.
                # Second return is the rear chassis at x=-0.093 m, not an
                # intervening obstacle. It must not prevent confirmation.
                scan.ranges = [.117 + self.gap - .0435, .1365]
                publishers['scan'].publish(scan)
                image = Image()
                image.header.stamp = stamp
                image.width, image.height = 1, 1
                publishers['rgb'].publish(image)
                publishers['depth'].publish(image)
                pose = PoseWithCovarianceStamped()
                pose.header.stamp, pose.header.frame_id = stamp, 'map'
                pose.pose.pose.orientation.w = 1.
                pose.pose.covariance[0] = pose.pose.covariance[7] = .001
                publishers['pose'].publish(pose)
                odom = Odometry()
                odom.header.stamp = stamp
                odom.pose.pose.orientation.w = 1.
                publishers['odom'].publish(odom)
                target = TargetEstimate()
                target.header.stamp, target.header.frame_id = stamp, 'map'
                target.stable = target.surface_valid = True
                target.label = 'bottle'
                target.point.x = target.surface_point.x = .9
                publishers['target'].publish(target)
                time.sleep(.05)

        def wait_for(predicate, timeout=8.):
            deadline = time.time() + timeout
            while time.time() < deadline and not rospy.is_shutdown():
                if predicate(): return True
                time.sleep(.02)
            return False

        thread = threading.Thread(target=send_sensors)
        thread.daemon = True
        thread.start()
        try:
            self.assertTrue(wait_for(lambda: self.state.startswith('PREVIEW:')), self.state)
            rospy.wait_for_service('/target_navigation/confirm', timeout=5)
            result = rospy.ServiceProxy('/target_navigation/confirm', Trigger)()
            self.assertTrue(result.success, result.message)
            self.assertTrue(wait_for(lambda: any(v[0] > 0. for v in self.commands)), self.state)
            self.gap = .051
            time.sleep(.7)
            self.assertTrue(self.state.startswith('NAVIGATING:'), self.state)
            self.assertGreater(self.commands[-1][0], 0.)
            self.gap = .050  # serialized through float32 LaserScan, not doubles.
            self.assertTrue(wait_for(lambda: self.state == 'FAILED: BLOCKED_NO_SAFE_PATH'), self.state)
            self.assertEqual(self.commands[-1], (0., 0.))
            self.assertEqual(self.permission, 'STOP')
            self.commands[:] = []
            self.gap = .60
            time.sleep(.7)
            self.assertEqual(self.state, 'FAILED: BLOCKED_NO_SAFE_PATH')
            self.assertFalse(any(v[0] != 0. or v[1] != 0. for v in self.commands))
            retry = rospy.ServiceProxy('/target_navigation/confirm', Trigger)()
            self.assertFalse(retry.success)
        finally:
            self.running = False
            thread.join(2)
        self.assertTrue(subscriptions)


if __name__ == '__main__':
    rospy.init_node('test_obstacle_threshold')
    rostest.rosrun('x3_target_navigation', 'obstacle_threshold', ObstacleThreshold)
