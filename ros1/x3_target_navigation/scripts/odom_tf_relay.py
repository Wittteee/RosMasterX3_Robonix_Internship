#!/usr/bin/env python
"""Publish odom to base_footprint at the current time.

The EKF pose can keep an old stamp. AMCL then drops every newer laser scan,
the map transform stops, and target detection cannot publish.
"""
from __future__ import print_function

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


def frame_name(name):
    if name.startswith("/"):
        return name[1:]
    return name


class OdomTfRelay(object):
    def __init__(self):
        self._broadcaster = tf2_ros.TransformBroadcaster()
        self._latest = None
        rospy.Subscriber("odom", Odometry, self._store, queue_size=5)
        rospy.Timer(rospy.Duration(0.05), self._publish)

    def _store(self, message):
        self._latest = message

    def _publish(self, event):
        del event
        message = self._latest
        if message is None or message.header.stamp.is_zero():
            return
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = frame_name(message.header.frame_id)
        transform.child_frame_id = frame_name(message.child_frame_id)
        if not transform.header.frame_id or not transform.child_frame_id:
            return
        transform.transform.translation.x = message.pose.pose.position.x
        transform.transform.translation.y = message.pose.pose.position.y
        transform.transform.translation.z = message.pose.pose.position.z
        transform.transform.rotation = message.pose.pose.orientation
        self._broadcaster.sendTransform(transform)


if __name__ == "__main__":
    rospy.init_node("odom_tf_relay")
    OdomTfRelay()
    rospy.spin()
