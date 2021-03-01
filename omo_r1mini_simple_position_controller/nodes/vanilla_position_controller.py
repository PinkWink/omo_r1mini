#!/usr/bin/env python

import rospy
import time 
from math import atan2, pi, sqrt, cos
from copy import deepcopy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class PID:
    def __init__(self):
        self.P = 0.0
        self.I = 0.0
        self.D = 0.0
        self.max_state = 0.0
        self.min_state = 0.0
        self.pre_state = 0.0
        self.dt = 0.0
        self.integrated_state = 0.0
        self.pre_time = rospy.Time.now()
        
    def process(self, state):
        dt = rospy.Time.now() - self.pre_time

        if self.dt == 0.:
            state_D = 0.
        else:
            state_D = (state - self.pre_state) / self.dt

        state_I = state + self.integrated_state

        out = self.P*state + self.D*state_D + self.I*state_I * self.dt

        if out > self.max_state:
            out = self.max_state
        elif out < self.min_state:
            out = self.min_state

        self.pre_state = state
        self.integrated_state = state_I
        self.pre_time = rospy.Time.now()

        return out

class RobotState(object):
    x = 0.0
    y = 0.0
    theta = 0.0

def calc_errors(msg, cur_pos, ref_pos):
    rot_q = msg.pose.pose.orientation 
    roll, pitch, theta = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    cur_pos.x = msg.pose.pose.position.x
    cur_pos.y = msg.pose.pose.position.y
    cur_pos.theta = theta

    delta_y_ref = ref_pos.y - cur_pos.y
    delta_x_ref = ref_pos.x - cur_pos.x
    ref_pos.theta = atan2(delta_y_ref, delta_x_ref)

    e_s = sqrt(delta_x_ref**2 + delta_y_ref**2) * cos( atan2(delta_y_ref, delta_x_ref) - cur_pos.theta )
    e_theta = ref_pos.theta - cur_pos.theta

    return e_s, e_theta

def position_control(msg):
    global cur_pos, ref_pos, speed
    
    e_s, e_theta = calc_errors(msg, cur_pos, ref_pos)

    speed.angular.z = theta_PID.process(e_theta)
    speed.linear.x = translation_PID.process(e_s)

    print "==========================="
    print "ref_pos.theta : " + str(round(ref_pos.theta * 180. / pi, 2)) + ", cur_pos.theta : " + str(round(cur_pos.theta * 180. / pi, 2))
    print "ref_pos.x : " + str(round(ref_pos.x, 2)) + ", ref_pos.y : " + str(round(ref_pos.y, 2))
    print "cur_pos.x : " + str(round(cur_pos.x, 2)) + ", cur_pos.y : " + str(round(cur_pos.y, 2))
    print "e_theta : " + str(round(e_theta * 180. / pi, 2)) + ", e_s : " + str(round(e_s, 2))
    print "speed.angular.z : " + str(round(speed.angular.z, 2))
    print "speed.linear.x : " + str(round(speed.linear.x, 2))

rospy.init_node("position_controller")

ref_pos = RobotState()

ref_pos.x = 0.5
ref_pos.y = 0.5
ref_pos.theta = 0.

cur_pos = RobotState()
speed = Twist()

theta_PID = PID()
theta_PID.P = 0.3
theta_PID.I = 0.001
theta_PID.max_state = 0.5
theta_PID.min_state = -0.5

translation_PID = PID()
translation_PID.P = 0.05
translation_PID.I = 0.001
translation_PID.max_state = 0.1
translation_PID.min_state = -0.1

sub = rospy.Subscriber("/odom", Odometry, position_control)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

rate = rospy.Rate(5)

while not rospy.is_shutdown():
    pub.publish(speed)
    rate.sleep()    