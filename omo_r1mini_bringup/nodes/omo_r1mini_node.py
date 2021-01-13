#!/usr/bin/env python

import sys
import rospy
import math
from time import sleep
from omo_packet_handler import PacketHandler

from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Point, Vector3, Quaternion
from tf.broadcaster import TransformBroadcaster
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from omo_r1mini_bringup.srv import Battery, BatteryResponse
from omo_r1mini_bringup.srv import Color, ColorResponse
from omo_r1mini_bringup.srv import SaveColor, SaveColorResponse

class OdomPose(object):
   x = 0.0
   y = 0.0
   theta = 0.0
   timestamp = 0

class OdomVel(object):
   x = 0.0
   y = 0.0
   w = 0.0

class Joint(object):
   joint_name = ['wheel_left_joint', 'wheel_right_joint']
   joint_pos = [0.0, 0.0]
   joint_vel = [0.0, 0.0]

class OMOR1miniNode:
   def __init__(self):
      self.ph = PacketHandler()
      self.ph.stop_periodic_comm()  
      
      self.gear_ratio = 21.3
      self.wheel_base = 0.17
      self.wheel_radius = 0.03375
      self.enc_pulse = 44.

      self.max_lin_vel_x = 1.2
      self.max_ang_vel_z = 1.0

      self.distance_per_pulse = 2*math.pi*self.wheel_radius / self.enc_pulse / self.gear_ratio

      self.is_enc_offset_reset = False
      self.left_enc_bias, self.right_enc_bias = 0., 0.

      self.odom_pose = OdomPose()
      self.odom_vel = OdomVel()
      self.joint = Joint() 

      rospy.loginfo('Robot GEAR ratio: %s', self.gear_ratio)
      rospy.loginfo('Robot wheel_base: %s', self.wheel_base)
      rospy.loginfo('Robot wheel_radius: %s', self.wheel_radius)
      rospy.loginfo('Robot enc_pulse: %s', self.enc_pulse)
      rospy.loginfo('Robot distance_per_pulse: %s', self.distance_per_pulse)

      rospy.Service('battery_status', Battery, self.battery_service_handle)
      rospy.Service('set_led_color', Color, self.led_color_service_handle)
      rospy.Service('save_led_color', Color, self.save_led_color_service_handle)
      rospy.Subscriber("cmd_vel", Twist, self.sub_cmd_vel, queue_size=1)

      self.pub_joint_states = rospy.Publisher('joint_states', JointState, queue_size=10)
      self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)
      self.odom_broadcaster = TransformBroadcaster()

      self.ph.stop_periodic_comm()
      self.ph.update_battery_state()
      self.ph.set_periodic_info()

      sleep(0.1)

      rospy.loginfo('==> Start R1mini ')
      rospy.Timer(rospy.Duration(0.01), self.update_robot)
      self.odom_pose.timestamp = rospy.Time.now().to_nsec()

   def update_odometry(self, odo_l, odo_r, trans_vel, orient_vel):
      odo_l /= 1000.
      odo_r /= 1000.
      theta = (odo_r - odo_l) / self.wheel_base
      r = (odo_r + odo_l) / 2.0

      timestamp_now = rospy.Time.now()
      timestamp_now_nsec = timestamp_now.to_nsec()

      self.odom_pose.timestamp = timestamp_now_nsec
      self.odom_pose.theta = theta
      self.odom_pose.x = math.cos(theta) * r
      self.odom_pose.y = math.sin(theta) * r

      self.odom_vel.x = trans_vel / 1000.
      self.odom_vel.y = 0.0
      self.odom_vel.w = orient_vel

      parent_frame_id = "odom"
      child_frame_id = "base_footprint"

      odom_orientation_quat = quaternion_from_euler(0, 0, self.odom_pose.theta)
      self.odom_broadcaster.sendTransform((self.odom_pose.x, self.odom_pose.y, 0.), odom_orientation_quat, timestamp_now, child_frame_id, parent_frame_id)
      
      odom = Odometry()
      odom.header.stamp = timestamp_now
      odom.header.frame_id = parent_frame_id
      odom.child_frame_id = child_frame_id
      odom.pose.pose = Pose(Point(self.odom_pose.x, self.odom_pose.y, 0.), Quaternion(*odom_orientation_quat))
      odom.twist.twist = Twist(Vector3(self.odom_vel.x, self.odom_vel.y, 0), Vector3(0, 0, self.odom_vel.w))
      
      self.odom_pub.publish(odom)

   def updateJointStates(self, odo_l, odo_r, trans_vel, orient_vel):
      odo_l /= 1000.
      odo_r /= 1000.

      wheel_ang_left = odo_l / self.wheel_radius
      wheel_ang_right = odo_r / self.wheel_radius

      wheel_ang_vel_left = (trans_vel - (self.wheel_base / 2.0) * orient_vel) / self.wheel_radius
      wheel_ang_vel_right = (trans_vel + (self.wheel_base / 2.0) * orient_vel) / self.wheel_radius

      self.joint.joint_pos = [wheel_ang_left, wheel_ang_right]
      self.joint.joint_vel = [wheel_ang_vel_left, wheel_ang_vel_right]

      joint_states = JointState()
      joint_states.header.frame_id = "base_link"
      joint_states.header.stamp = rospy.Time.now()
      joint_states.name = self.joint.joint_name
      joint_states.position = self.joint.joint_pos
      joint_states.velocity = self.joint.joint_vel
      joint_states.effort = []

      self.pub_joint_states.publish(joint_states)

   def update_robot(self, event):
      raw_data = self.ph.parser()

      try:
         [trans_vel, orient_vel] = self.ph.robot_state['VW']
         [odo_l, odo_r] = self.ph.robot_state['ODO']
         [gyro_x, gyro_y, gyro_z] = self.ph.robot_state['GYRO']
         [acc_x, acc_y, acc_z] = self.ph.robot_state['ACCL']
         [angle_x, angle_y, angle_z] = self.ph.robot_state['POSE']

         self.update_odometry(odo_l, odo_r, trans_vel, orient_vel)
         self.updateJointStates(odo_l, odo_r, trans_vel, orient_vel)

      except ValueError:
         rospy.logwarn("ValueError occupied during read robot status in update_robot state. \n\r Raw_data : %s \n\r Robot state : %s", 
                        raw_data,
                        self.ph.robot_state)
      
   def sub_cmd_vel(self, cmd_vel_msg):
      lin_vel_x = cmd_vel_msg.linear.x
      ang_vel_z = cmd_vel_msg.angular.z

      lin_vel_x = max(-self.max_lin_vel_x, min(self.max_lin_vel_x, lin_vel_x))
      ang_vel_z = max(-self.max_ang_vel_z, min(self.max_ang_vel_z, ang_vel_z))

      self.ph.set_wheel_velocity(lin_vel_x*1000, ang_vel_z*1000)

   def battery_service_handle(self, req):
      self.ph.update_battery_state()

      bat_status = self.ph.robot_state['BAT']
      if len(bat_status) == 3:
         volt = bat_status[0]*0.1
         SOC = bat_status[1]
         current = bat_status[2]*0.001
         return BatteryResponse(volt, SOC, current)
      else:
         rospy.logwarn("Battery Status is not correct.")

   def led_color_service_handle(self, req):
      command = "$cCOLOR," + str(req.red) + ',' + str(req.green) + ',' + str(req.blue)
      self.ph.write_port(command)
      return ColorResponse()

   def save_led_color_service_handle(self, req):
      command = "$sCOLOR," + str(req.red) + ',' + str(req.green) + ',' + str(req.blue)
      self.ph.write_port(command)
      return ColorResponse()

   def main(self):
      rospy.spin()

if __name__ == '__main__':
    rospy.init_node('omo_r1mini_node')
    node = OMOR1miniNode()
    node.main()