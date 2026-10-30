#!/usr/bin/env python
"""Run the vendor driver with the verified serial port explicitly selected."""
import os
import json
import imp
import threading
import time
import rospy
from geometry_msgs.msg import Twist
import Rosmaster_Lib
from velocity_mapping import map_velocity,command_expired,monotonic
from nav_calibration_policy import require_speed_limits
VendorRosmaster=Rosmaster_Lib.Rosmaster
class PinnedRosmaster(VendorRosmaster):
    def __init__(self,car_type=1,com='/dev/ttyUSB1',delay=.002,debug=False):
        VendorRosmaster.__init__(self,car_type=car_type,com='/dev/ttyUSB1',delay=delay,debug=debug)
Rosmaster_Lib.Rosmaster=PinnedRosmaster
vendor=imp.load_source('vendor_driver_source',os.path.expanduser('~/yahboomcar_ws/src/yahboomcar_bringup/scripts/Mcnamu_driver.py'))
Driver=vendor.yahboomcar_driver
lock=threading.RLock()
state={'driver':None,'last_command':None,'stopped':True}
publisher=None
max_linear=.1
max_angular=.2

def apply(driver,command):
    driver.car.set_car_motion(*command)
    state['stopped']=command==(0.,0.,0.)
    message=Twist()
    message.linear.x,message.linear.y,message.angular.z=command
    if publisher is not None: publisher.publish(message)

def corrected_callback(self,msg):
    with lock:
        try:
            command=map_velocity(msg.linear.x,msg.linear.y,msg.angular.z,max_linear,max_angular)
            requested=(msg.linear.x,msg.linear.y,msg.angular.z)
            if command != requested:
                rospy.logwarn_throttle(2., 'Velocity limited: requested=%s applied=%s (m/s, m/s, rad/s)' %
                                       (requested,command))
        except (ValueError,TypeError):
            command=(0.,0.,0.)
            rospy.logerr('Rejected non-finite base velocity; stopping')
        apply(self,command)
        state['last_command']=monotonic()

def stop():
    with lock:
        driver=state['driver']
        if driver is not None:
            for _ in range(3): apply(driver,(0.,0.,0.))

def watchdog(event):
    with lock:
        if not state['stopped'] and command_expired(state['last_command'],monotonic()):
            stop()
            rospy.logwarn('Base command timeout; zero command sent')

Driver.cmd_vel_callback=corrected_callback
rospy.init_node('driver_node',anonymous=False)
profile_path=rospy.get_param('~calibration_profile',
                            os.path.join(os.path.dirname(os.path.abspath(__file__)), 'chassis_nav_calibration.json'))
with open(profile_path) as profile_file:
    speed_profile=json.load(profile_file)
speed_limits=require_speed_limits(speed_profile,
                                  rospy.get_param('~command_max_linear_mps',None),
                                  rospy.get_param('~command_max_angular_rps',None))
max_linear=speed_limits['max_linear_mps']
max_angular=speed_limits['max_angular_rps']
map_velocity(0,0,0,max_linear,max_angular)
rospy.loginfo('Metric velocity mapping: linear %.3f m/s, angular %.3f rad/s; profile %s',
              max_linear,max_angular,profile_path)
publisher=rospy.Publisher('~applied_cmd_vel',Twist,queue_size=20)
driver=Driver()
driver.car.ser.write_timeout=.2
state['driver']=driver
stop()
rospy.on_shutdown(stop)
timer=rospy.Timer(rospy.Duration(.05),watchdog)
try:
    driver.pub_data()
finally:
    stop()
