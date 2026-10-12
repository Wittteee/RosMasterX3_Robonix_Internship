"""Reject default zero telemetry when the ROS driver has no MCU response."""
import json
import math
import sys

import rospy
from std_msgs.msg import Float32

rospy.init_node("x3_chassis_health_check", anonymous=True, disable_signals=True)
try:
    version = rospy.wait_for_message("/edition", Float32, timeout=10).data
    voltage = rospy.wait_for_message("/voltage", Float32, timeout=10).data
    valid = (not math.isinf(version) and not math.isnan(version) and version > 0 and
             not math.isinf(voltage) and not math.isnan(voltage) and voltage > 0)
    print(json.dumps({"firmware_version": version, "voltage": voltage, "controller_reply_valid": valid}))
    sys.exit(0 if valid else 2)
except rospy.ROSException as error:
    print(json.dumps({"controller_reply_valid": False, "error": str(error)}))
    sys.exit(2)
