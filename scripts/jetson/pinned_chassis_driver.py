#!/usr/bin/env python
"""Run the vendor driver with the verified serial port explicitly selected."""
import os
import imp
import threading
import time
import rospy
from geometry_msgs.msg import Twist
import Rosmaster_Lib
from velocity_mapping import map_velocity,command_expired,monotonic
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
max_linear=float(rospy.get_param('~command_max_linear_mps',.1))
max_angular=float(rospy.get_param('~command_max_angular_rps',.2))
map_velocity(0,0,0,max_linear,max_angular)
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
