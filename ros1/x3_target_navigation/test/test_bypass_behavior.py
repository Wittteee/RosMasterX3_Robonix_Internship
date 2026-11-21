#!/usr/bin/env python
"""Isolated ROS tests: real manager, fake sensors/planner, never a base driver."""
from __future__ import print_function
import math
import threading
import time
import unittest

import actionlib
import rospy
import rostest
import tf2_ros
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal, MoveBaseResult
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from nav_msgs.srv import GetPlan, GetPlanResponse
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Float32, Header, String
from std_srvs.srv import Trigger
from x3_target_navigation.msg import TargetEstimate


class BypassBehavior(unittest.TestCase):
    def test_stop_select_side_bypass_and_reacquire(self):
        scenario = rospy.get_param('~scenario')
        real = scenario.startswith('real_')
        sign = -1. if scenario.endswith('left') else 1.
        self.state, self.phase = '', ''
        self.pose = [0., 0.]
        self.yaw = 0.
        self.velocity = [0.,0.,0.]
        self.goals, self.plans, self.received = [], [], []
        self.brake_time = None
        self.dynamic = False
        self.running = True
        self.permission = ''
        self.transitions = []
        self.raw_commands = []
        self.last_plan = None
        self.preview_goal = None

        def state_cb(message):
            self.state = message.data
            self.transitions.append(message.data)
            if 'BRAKING_FOR_' in message.data and self.brake_time is None:
                self.brake_time = time.time()

        subscriptions = [
            rospy.Subscriber('/target_navigation/preview_goal',PoseStamped,
                             lambda m:setattr(self,'preview_goal',m)),
            rospy.Subscriber('/target_navigation/state', String, state_cb),
            rospy.Subscriber('/target_navigation/approach_velocity', Twist,
                             lambda m: self.received.append((time.time(),m.linear.x,m.linear.y,m.angular.z))),
            rospy.Subscriber('/target_navigation/motion_permission', Header,
                             lambda m: setattr(self,'permission',m.frame_id)),
        ]
        if real:
            def applied(message):
                self.velocity = [message.linear.x,message.linear.y,message.angular.z]
            def goal_seen(message):
                p = message.goal.target_pose.pose.position
                self.goals.append((time.time(),p.x,p.y))
            def plan_seen(message):
                self.last_plan = [(p.pose.position.x,p.pose.position.y) for p in message.poses][::max(1,len(message.poses)//6)]
            subscriptions.extend([
                rospy.Subscriber('/target_navigation/approach_velocity',Twist,applied),
                rospy.Subscriber('/move_base/goal',MoveBaseActionGoal,goal_seen),
                rospy.Subscriber('/target_navigation/bypass_nav_velocity',Twist,
                                 lambda m:self.raw_commands.append((m.linear.x,m.linear.y,m.angular.z))),
                rospy.Subscriber('/move_base/GlobalPlanner/plan',Path,plan_seen)])
        static = tf2_ros.StaticTransformBroadcaster()
        dynamic_tf = tf2_ros.TransformBroadcaster()
        transforms = []
        for parent,child,x,yaw in [('map','odom',0.,0.),
                                   ('base_footprint','laser',.0435,math.pi),
                                   ('base_footprint','camera_rgb_optical_frame',0.,0.)]:
            tf = TransformStamped()
            tf.header.stamp, tf.header.frame_id, tf.child_frame_id = rospy.Time.now(), parent, child
            tf.transform.translation.x = x
            tf.transform.rotation.z, tf.transform.rotation.w = math.sin(yaw/2.),math.cos(yaw/2.)
            transforms.append(tf)
        static.sendTransform(transforms)
        publishers = {
            'voltage':rospy.Publisher('/voltage',Float32,queue_size=1),
            'edition':rospy.Publisher('/edition',Float32,queue_size=1),
            'scan':rospy.Publisher('/scan',LaserScan,queue_size=1),
            'rgb':rospy.Publisher('/camera/rgb/image_raw',Image,queue_size=1),
            'depth':rospy.Publisher('/camera/depth_registered/image_raw',Image,queue_size=1),
            'pose':rospy.Publisher('/amcl_pose',PoseWithCovarianceStamped,queue_size=1),
            'odom':rospy.Publisher('/odom',Odometry,queue_size=1),
            'target':rospy.Publisher('/target_navigation/fused_target',TargetEstimate,queue_size=1),
            'nav':rospy.Publisher('/target_navigation/bypass_nav_velocity',Twist,queue_size=1),
        }

        def make_plan(request):
            self.plans.append(time.time())
            response = GetPlanResponse()
            response.plan.header.frame_id = 'map'
            if scenario == 'no_path': return response
            a,b = request.start.pose.position,request.goal.pose.position
            count = max(2,int(math.hypot(b.x-a.x,b.y-a.y)/.01)+1)
            for index in range(count+1):
                p = PoseStamped()
                p.header.frame_id, p.header.stamp = 'map', rospy.Time.now()
                ratio = float(index)/count
                p.pose.position.x, p.pose.position.y = a.x+(b.x-a.x)*ratio,a.y+(b.y-a.y)*ratio
                p.pose.orientation = request.goal.pose.orientation
                response.plan.poses.append(p)
            return response

        service = None if real else rospy.Service('/move_base/make_plan',GetPlan,make_plan)

        def execute(goal):
            self.goals.append((time.time(),goal.target_pose.pose.position.x,goal.target_pose.pose.position.y))
            index = len(self.goals)-1
            started = time.time()
            while time.time()-started < (5.0 if scenario == 'wrong_direction' else .60) and not rospy.is_shutdown():
                if server.is_preempt_requested():
                    server.set_preempted(MoveBaseResult())
                    return
                velocity = Twist()
                if index == 0: velocity.linear.y = sign*.025*(-1. if scenario == 'wrong_direction' else 1.)
                else: velocity.linear.x = .025
                if scenario == 'dynamic' and time.time()-started > .25:
                    self.dynamic = True
                publishers['nav'].publish(velocity)
                time.sleep(.04)
            self.pose[:] = [goal.target_pose.pose.position.x,goal.target_pose.pose.position.y]
            time.sleep(.15)
            server.set_succeeded(MoveBaseResult())

        server = None
        if not real:
            server = actionlib.SimpleActionServer('/move_base',MoveBaseAction,execute_cb=execute,auto_start=False)
            if scenario != 'server_delayed': server.start()
        if real:
            map_publisher = rospy.Publisher('/map',OccupancyGrid,queue_size=1,latch=True)
            grid = OccupancyGrid()
            grid.header.frame_id,grid.header.stamp = 'map',rospy.Time.now()
            grid.info.resolution,grid.info.width,grid.info.height = .05,100,100
            grid.info.origin.position.x = grid.info.origin.position.y = -2.
            grid.info.origin.orientation.w = 1.
            grid.data = [0]*10000
            # A known small side obstruction, with the actual scan independently
            # supplying the dynamic obstacle layer. No real devices are used.
            for row in range(100):
                for col in range(100):
                    x,y = -2.+(col+.5)*.05,-2.+(row+.5)*.05
                    obstacle_x = .70 if scenario == 'real_far_left' else .40
                    if obstacle_x-.01<x<obstacle_x+.11 and -.19<sign*y<-.05: grid.data[row*100+col] = 100
            map_publisher.publish(grid)

        def send_sensors():
            last = time.time()
            while self.running and not rospy.is_shutdown():
                if real:
                    dt = min(.10,time.time()-last)
                    last = time.time()
                    vx,vy,wz = self.velocity
                    self.pose[0] += (math.cos(self.yaw)*vx-math.sin(self.yaw)*vy)*dt
                    self.pose[1] += (math.sin(self.yaw)*vx+math.cos(self.yaw)*vy)*dt
                    self.yaw += wz*dt
                stamp = rospy.Time.now()
                tf = TransformStamped()
                tf.header.stamp,tf.header.frame_id,tf.child_frame_id = stamp,'odom','base_footprint'
                tf.transform.translation.x,tf.transform.translation.y = self.pose
                tf.transform.rotation.z,tf.transform.rotation.w = math.sin(self.yaw/2.),math.cos(self.yaw/2.)
                dynamic_tf.sendTransform(tf)
                publishers['voltage'].publish(Float32(11.8))
                publishers['edition'].publish(Float32(2.3))
                scan = LaserScan()
                scan.header.stamp,scan.header.frame_id = stamp,'laser'
                scan.angle_min,scan.angle_increment = -math.pi,2*math.pi/720
                scan.range_min,scan.range_max = .01,12.
                scan.ranges = [float('inf')]*720
                obstacle_x = (.70 if scenario == 'real_far_left' else .40) if real else .16
                if scenario == 'brake_rebound' and self.brake_time is not None:
                    obstacle_x = .17  # A small brake recoil must not resume forward motion.
                obstacles = [(obstacle_x,-sign*.07),(obstacle_x,-sign*.10),(obstacle_x,-sign*.13)]
                if scenario == 'full': obstacles = [(.16,-.12),(.16,0.),(.16,.12)]
                if scenario == 'side_blocked': obstacles.append((0.,.25))
                if self.dynamic: obstacles.append((self.pose[0],self.pose[1]+.14))
                # Add the rear chassis return in base coordinates.
                body_points = [(math.cos(self.yaw)*(x-self.pose[0])+math.sin(self.yaw)*(y-self.pose[1]),
                                -math.sin(self.yaw)*(x-self.pose[0])+math.cos(self.yaw)*(y-self.pose[1]))
                               for x,y in obstacles]+[(-.093,.084)]
                for x,y in body_points:
                    lx,ly = -(x-.0435),-y  # reversed laser, translated forward
                    angle = math.atan2(ly,lx)
                    index = int(round((angle+math.pi)/scan.angle_increment))%720
                    scan.ranges[index] = min(scan.ranges[index],math.hypot(lx,ly))
                publishers['scan'].publish(scan)
                image = Image()
                image.header.stamp,image.width,image.height = stamp,1,1
                publishers['rgb'].publish(image)
                publishers['depth'].publish(image)
                pose = PoseWithCovarianceStamped()
                pose.header.stamp,pose.header.frame_id = stamp,'map'
                pose.pose.pose.position.x,pose.pose.pose.position.y = self.pose
                pose.pose.pose.orientation.z,pose.pose.pose.orientation.w = math.sin(self.yaw/2.),math.cos(self.yaw/2.)
                pose.pose.covariance[0] = pose.pose.covariance[7] = .001
                publishers['pose'].publish(pose)
                odom = Odometry()
                odom.header.stamp,odom.header.frame_id,odom.child_frame_id = stamp,'odom','base_footprint'
                odom.pose.pose = pose.pose.pose
                # The first stop is not considered complete until fresh
                # odometry actually reports zero after the braking interval.
                if real:
                    odom.twist.twist.linear.x,odom.twist.twist.linear.y,odom.twist.twist.angular.z = self.velocity
                elif self.brake_time is not None and time.time()-self.brake_time < .30:
                    odom.twist.twist.linear.x = .04
                publishers['odom'].publish(odom)
                target = TargetEstimate()
                target.header.stamp,target.header.frame_id = stamp,'map'
                target.stable = target.surface_valid = True
                target.label = 'bottle'
                target.point.x = target.surface_point.x = (1.31 if scenario == 'real_far_left' else
                    .667 if scenario == 'real_close_left' else 1.05)
                publishers['target'].publish(target)
                time.sleep(.05)

        def wait_for(predicate,timeout=12,break_on_failure=False):
            deadline = time.time()+timeout
            while time.time()<deadline and not rospy.is_shutdown():
                if predicate(): return True
                if break_on_failure and self.state.startswith('FAILED:'): return False
                time.sleep(.02)
            return False

        thread = threading.Thread(target=send_sensors)
        thread.daemon = True
        thread.start()
        try:
            self.assertTrue(wait_for(lambda:self.state.startswith('PREVIEW:')),self.state)
            self.assertTrue(wait_for(lambda:self.preview_goal is not None))
            bottle_x = (1.31 if scenario == 'real_far_left' else
                        .667 if scenario == 'real_close_left' else 1.05)
            # Initial approach must end at the bottle parking pose, never at
            # the nearer laser obstacle (0.16 m in the synthetic side cases).
            self.assertAlmostEqual(self.preview_goal.pose.position.x,bottle_x-.297,places=3)
            rospy.wait_for_service('/target_navigation/confirm',timeout=5)
            if scenario == 'server_delayed':
                started = time.time()
                early = rospy.ServiceProxy('/target_navigation/confirm',Trigger)()
                self.assertFalse(early.success)
                self.assertLess(time.time()-started,.5)
                self.assertTrue(self.state.startswith('PREVIEW:'))
                server.start()
            probe = actionlib.SimpleActionClient('/move_base',MoveBaseAction)
            self.assertTrue(probe.wait_for_server(rospy.Duration(15)))
            confirm = rospy.ServiceProxy('/target_navigation/confirm',Trigger)
            # The probe and the manager have independent action clients. The
            # manager's status handshake can complete one heartbeat later.
            deadline = time.time()+3.
            while True:
                result = confirm()
                if result.success or 'requires move_base' not in result.message or time.time()>deadline:break
                time.sleep(.1)
            self.assertTrue(result.success,result.message)
            if real or scenario in ('right','left','server_delayed','brake_rebound'):
                passed = wait_for(lambda:self.state.startswith('SUCCEEDED:'),100 if real else 20,True)
                self.assertTrue(passed,{'states':self.transitions,'goals':self.goals,
                    'raw_commands':self.raw_commands[:15],'plan':self.last_plan})
                self.assertEqual(len(self.goals),3)
                self.assertAlmostEqual(math.hypot(self.goals[2][1]-bottle_x,self.goals[2][2]),.297,places=3)
                self.assertGreater(sign*self.goals[0][2],.15)
                if not real: self.assertGreaterEqual(self.plans[0]-self.brake_time,.35)
                self.assertTrue(any(sign*row[2]>.01 for row in self.received))
                self.assertTrue(any('BYPASS_'+('LEFT' if sign>0 else 'RIGHT')+'_REJOIN' in s for s in self.transitions))
            else:
                self.assertTrue(wait_for(lambda:self.state.startswith('FAILED:'),15),self.transitions)
                if scenario in ('full','no_path','side_blocked'):
                    self.assertEqual(len(self.goals),0)
                    self.assertFalse(any(abs(vx)+abs(vy)>1e-9 for _,vx,vy,_ in self.received))
                elif scenario == 'wrong_direction':
                    self.assertIn('BYPASS_COMMAND_TIMEOUT',self.state)
                    self.assertFalse(any(abs(vx)+abs(vy)>1e-9 for _,vx,vy,_ in self.received))
                else:
                    self.assertIn('BYPASS_DYNAMIC_OBSTACLE',self.state)
            self.assertEqual(self.permission,'STOP')
            time.sleep(.1)
            self.assertTrue(all(abs(v)<1e-9 for v in self.received[-1][1:]))
        finally:
            self.running = False
            thread.join(2)
            if service is not None: service.shutdown()
        self.assertTrue(subscriptions)


if __name__ == '__main__':
    rospy.init_node('test_bypass_behavior')
    rostest.rosrun('x3_target_navigation','bypass_behavior_'+rospy.get_param('~scenario'),BypassBehavior)
