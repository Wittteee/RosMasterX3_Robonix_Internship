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
from velocity_mapping import StraightHeadingHold,command_expired,map_velocity,monotonic
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
heading_hold=None

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
        now=monotonic()
        if heading_hold is not None:
            try:
                command=heading_hold.correct(command,now,max_angular)
            except ValueError:
                command=(0.,0.,0.)
                rospy.logwarn_throttle(2.,'Straight heading feedback unavailable; stopping')
        apply(self,command)
        state['last_command']=monotonic()


def update_heading(msg):
    with lock:
        age=(rospy.Time.now()-msg.header.stamp).to_sec()
        if (msg.header.frame_id.lstrip('/')!='odom' or
                msg.child_frame_id.lstrip('/')!='base_footprint' or
                msg.header.stamp.is_zero() or not -.05 <= age <= heading_hold.max_age_sec):
            heading_hold.sample=None
            return
        q=msg.pose.pose.orientation
        heading_hold.observe((q.x,q.y,q.z,q.w),monotonic()-max(0.,age))

def stop():
    with lock:
        if heading_hold is not None:
            heading_hold.reset()
        driver=state['driver']
        if driver is not None:
            for _ in range(3): apply(driver,(0.,0.,0.))

def watchdog(event):
    with lock:
        now=monotonic()
        if not state['stopped'] and command_expired(state['last_command'],now):
            stop()
            rospy.logwarn('Base command timeout; zero command sent')
        elif (heading_hold is not None and heading_hold.target_yaw is not None and
              not heading_hold.has_fresh_heading(now)):
            stop()
            rospy.logwarn('Straight heading feedback expired; zero command sent')

Driver.cmd_vel_callback=corrected_callback
rospy.init_node('driver_node',anonymous=False)
profile_path=rospy.get_param('~calibration_profile',
                            os.path.join(os.path.dirname(os.path.abspath(__file__)), 'chassis_nav_calibration.json'))
with open(profile_path) as profile_file:
    speed_profile=json.load(profile_file)
heading_hold_enabled=rospy.get_param('~straight_heading_hold',False)
if not isinstance(heading_hold_enabled,bool):
    raise ValueError('straight_heading_hold must be a ROS boolean')
if heading_hold_enabled and speed_profile.get('heading_sign_verified') is not True:
    raise ValueError('Verify odom heading sign before enabling straight heading hold')
speed_limits=require_speed_limits(speed_profile,
                                  rospy.get_param('~command_max_linear_mps',None),
                                  rospy.get_param('~command_max_angular_rps',None))
max_linear=speed_limits['max_linear_mps']
max_angular=speed_limits['max_angular_rps']
map_velocity(0,0,0,max_linear,max_angular)
rospy.loginfo('Metric velocity mapping: linear %.3f m/s, angular %.3f rad/s; profile %s',
              max_linear,max_angular,profile_path)
publisher=rospy.Publisher('~applied_cmd_vel',Twist,queue_size=20)
if heading_hold_enabled:
    from nav_msgs.msg import Odometry
    heading_hold=StraightHeadingHold()
    heading_subscription=rospy.Subscriber('/odom',Odometry,update_heading,queue_size=1)
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
