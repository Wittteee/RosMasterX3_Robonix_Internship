#!/usr/bin/env python
"""Vendor chassis driver pinned to /dev/x3_base, with metric speed limits."""
from __future__ import print_function

import json
import math
import os
import time

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
import Rosmaster_Lib


def monotonic():
    if hasattr(time, "monotonic"):
        return time.monotonic()
    return time.time()


def map_velocity(vx, vy, wz, max_linear, max_angular):
    values = [float(v) for v in (vx, vy, wz, max_linear, max_angular)]
    if any(math.isnan(v) or math.isinf(v) for v in values):
        raise ValueError("non-finite velocity")
    vx, vy, wz, max_linear, max_angular = values
    if max_linear <= 0 or max_angular <= 0:
        raise ValueError("invalid speed limit")
    norm = math.hypot(vx, vy)
    scale = min(1.0, max_linear / norm) if norm else 1.0
    return vx * scale, vy * scale, max(-max_angular, min(max_angular, wz))


_OriginalRosmaster = Rosmaster_Lib.Rosmaster


class BaseRosmaster(_OriginalRosmaster):
    def __init__(self, car_type=1, com="/dev/x3_base", delay=.002, debug=False):
        port = rospy.get_param("~port", "/dev/x3_base")
        if not os.path.exists(port):
            raise RuntimeError("Base serial port does not exist: %s" % port)
        _OriginalRosmaster.__init__(
            self, car_type=car_type, com=port, delay=delay, debug=debug)
        self.x3_port = port


Rosmaster_Lib.Rosmaster = BaseRosmaster

_VENDOR = os.path.expanduser(
    "~/yahboomcar_ws/src/yahboomcar_bringup/scripts/Mcnamu_driver.py")
import imp
_vendor = imp.load_source("x3_vendor_driver", _VENDOR)
Driver = _vendor.yahboomcar_driver

_lock = __import__("threading").RLock()
_state = {"driver": None, "last_command": None, "stopped": True}
_publisher = None
_max_linear = 0.1
_max_angular = 0.2
_require_navigation_permission = False
_permission_time = None
_permission_allowed = False


def _motion_permitted():
    if not _require_navigation_permission:
        return True
    age = monotonic() - _permission_time if _permission_time is not None else -1
    return _permission_allowed and 0.0 <= age <= 0.4


def _permission_callback(message):
    global _permission_time, _permission_allowed
    with _lock:
        age = (rospy.Time.now() - message.stamp).to_sec()
        _permission_allowed = (message.frame_id == "ALLOW" and
                               not message.stamp.is_zero() and 0.0 <= age <= 0.4)
        # Do not extend an old queued ALLOW heartbeat by another 0.4 seconds.
        _permission_time = monotonic() - max(0.0, age)
        if not _motion_permitted():
            _stop()


def _apply(driver, command):
    driver.car.set_car_motion(*command)
    _state["stopped"] = command == (0.0, 0.0, 0.0)
    message = Twist()
    message.linear.x, message.linear.y, message.angular.z = command
    if _publisher is not None:
        _publisher.publish(message)


def _corrected_callback(self, msg):
    with _lock:
        try:
            command = map_velocity(msg.linear.x, msg.linear.y, msg.angular.z,
                                   _max_linear, _max_angular)
        except (TypeError, ValueError):
            command = (0.0, 0.0, 0.0)
            rospy.logerr("Rejected non-finite base velocity; stopping")
        if not _motion_permitted():
            command = (0.0, 0.0, 0.0)
        _apply(self, command)
        _state["last_command"] = monotonic()


def _stop():
    with _lock:
        driver = _state["driver"]
        if driver is None:
            return
        for _ in range(3):
            _apply(driver, (0.0, 0.0, 0.0))


def _watchdog(event):
    del event
    with _lock:
        last = _state["last_command"]
        now = monotonic()
        if (not _state["stopped"] and last is not None and
                (now - last > 0.4 or now < last or not _motion_permitted())):
            driver = _state["driver"]
            if driver is not None:
                for _ in range(3):
                    _apply(driver, (0.0, 0.0, 0.0))
            rospy.logwarn("Base command timeout; zero command sent")


def _limits():
    profile_path = rospy.get_param("~calibration_profile", "")
    linear = 0.1
    angular = 0.2
    if profile_path:
        with open(profile_path) as handle:
            profile = json.load(handle)
        if profile.get("velocity_mapping", "metric_identity_with_limits") != "metric_identity_with_limits":
            raise ValueError("Unsupported velocity mapping")
        linear = float(profile.get("max_linear_mps", linear))
        angular = float(profile.get("max_angular_rps", angular))
    if not 0 < linear <= 0.1 or not 0 < angular <= 0.2:
        raise ValueError("Chassis speed limits exceed 0.1 m/s and 0.2 rad/s")
    return linear, angular


def _firmware_version(car):
    """Read firmware even when the vendor cache stays at 0.

    Rosmaster.get_version() only stores the value if the reply arrives inside
    a 20 ms window. A later packet sets the raw bytes and every later call
    returns the cached 0.
    """
    reported = float(car.get_version())
    if reported >= 1.0:
        return reported
    high = getattr(car, "_Rosmaster__version_H", 0)
    low = getattr(car, "_Rosmaster__version_L", 0)
    if high:
        return float(high) + float(low) / 10.0
    return reported


def main():
    global _publisher, _max_linear, _max_angular, _require_navigation_permission
    rospy.init_node("driver_node", anonymous=False)
    _max_linear, _max_angular = _limits()
    _require_navigation_permission = rospy.get_param("~require_navigation_permission", False)
    permission_subscriber = rospy.Subscriber(
        "/target_navigation/motion_permission", Header,
        _permission_callback, queue_size=1)
    Driver.cmd_vel_callback = _corrected_callback
    driver = Driver()
    version = -1.0
    voltage = 0.0
    deadline = time.time() + 12.0
    while time.time() < deadline:
        version = _firmware_version(driver.car)
        voltage = float(driver.car.get_battery_voltage())
        if version >= 1.0 and 10.0 <= voltage <= 13.5:
            break
        time.sleep(0.05)
    if version < 1.0 or not 10.0 <= voltage <= 13.5:
        raise RuntimeError(
            "Chassis precheck failed on %s: version=%s voltage=%s"
            % (driver.car.x3_port, version, voltage))
    # Vendor pub_data republishes get_version(). That call returns the cached
    # 0 after a late packet, which makes the navigation precheck reject the car.
    driver.car.get_version = lambda _version=version: _version
    rospy.loginfo("Chassis %s version %.1f voltage %.2f V; limits %.2f m/s %.2f rad/s",
                  driver.car.x3_port, version, voltage, _max_linear, _max_angular)
    _publisher = rospy.Publisher("~applied_cmd_vel", Twist, queue_size=20)
    _state["driver"] = driver
    _stop()
    rospy.on_shutdown(_stop)
    rospy.Timer(rospy.Duration(0.05), _watchdog)
    driver.pub_data()


if __name__ == "__main__":
    main()
