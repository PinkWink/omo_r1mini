#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2, pi, sqrt
from copy import deepcopy

fisrt_step = True

speed = Twist()
cur_pos = Twist()
goal_pos = Twist()

relative_pos = Twist()
relative_pos.linear.x = 1.0
relative_pos.linear.y = 0.

def calc_goal(goal_pos, cur_pos, relative_pos):
    goal_pos = deepcopy(cur_pos)
    goal_pos.linear.x += relative_pos.linear.x
    goal_pos.linear.y += relative_pos.linear.y
    goal_pos.angular.z += relative_pos.angular.z

    return goal_pos

def calc_direction(goal_pos, cur_pos):
    angle_to_goal = atan2( (goal_pos.linear.x - cur_pos.linear.x), 
                            (goal_pos.linear.y - cur_pos.linear.y) ) 
    angle_to_goal = angle_to_goal - pi / 2.

    return angle_to_goal - cur_pos.angular.z

def calc_translastion(goal_pos, cur_pos):
    dist = sqrt((goal_pos.linear.x - cur_pos.linear.x)**2 + (goal_pos.linear.y - cur_pos.linear.y)**2)
    return dist

def position_control(msg):
    global cur_pos, goal_pos, relative_pos, fisrt_step, speed

    rot_q = msg.pose.pose.orientation 
    roll, pitch, theta = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    cur_pos.linear.x = msg.pose.pose.position.x
    cur_pos.linear.x = msg.pose.pose.position.x
    cur_pos.angular.z = theta

    if fisrt_step:
        fisrt_step = False
        goal_pos = calc_goal(goal_pos, cur_pos, relative_pos)

    theta = calc_direction(goal_pos, cur_pos)
    x = calc_translastion(goal_pos, cur_pos)

    speed.linear.x = x / abs(x) * 0.01

    if theta == 0:
        speed.angular.z = 0
    else:
        speed.angular.z = theta / abs(theta) * 0.05

    print "theta : " + str(round(theta, 3)) + " , " + "x : " + str(round(x, 3)) 


rospy.init_node("position_controller")

sub = rospy.Subscriber("/odom", Odometry, position_control)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

rate = rospy.Rate(10)

while not rospy.is_shutdown():
    pub.publish(speed)
    rate.sleep()    