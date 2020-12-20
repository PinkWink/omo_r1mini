#!/usr/bin/env python

# original source code -> https://github.com/omorobot/omoros
# modified by Bishop Pearson

import sys
import rospy
import serial
import io
import math
from time import sleep

from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Point, Vector3, Quaternion
from tf.broadcaster import TransformBroadcaster
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from omo_r1mini_bringup.srv import Battery, BatteryResponse

class PacketHandler:
   def __init__(self):
      port_name = rospy.get_param('~port', '/dev/ttyMotor')
      baud_rate = rospy.get_param('~baud', 115200)

      self.ser = serial.Serial(port_name, baud_rate)
      self.ser_io = io.TextIOWrapper(io.BufferedRWPair(self.ser, self.ser, 1), 
                                       newline = '\r', 
                                       line_buffering = True)

      self.robot_state = {
            "VW" : [0., 0.],
            "DIFFV" : [0., 0.],
            "ENCOD" : [0., 0.],
            "ACCL" : [0., 0., 0.],
            "GYRO" : [0., 0., 0.],
            "BAT" : [0., 0., 0.],
      }

      rospy.loginfo('Serial port: %s', port_name)
      rospy.loginfo('Serial baud rate: %s', baud_rate)

   def get_port_state(self):
      return self.ser.isOpen()

   def read_port(self):
      return self.ser_io.readline()

   def write_port(self, buffer):
      if self.get_port_state() == True:
         self.ser.write(buffer + "\r\n")
      else:
         rospy.logwarn('Serial Port %s is not Opened in Writing process', port_name)

   def read_packet(self):
      if self.get_port_state() == True:
         return self.read_port()
      else:
         rospy.logwarn('Serial Port %s is not Opened in Reading process', port_name)

   def update_battery_state(self):
      self.write_port("$qBAT")
      sleep(0.01)

   def parser_R1mini(self):
      raw_data_o = self.read_packet()

      raw_data = raw_data_o.replace('\r', '')
      raw_data = raw_data.replace('\n', '')
      raw_data = raw_data.split('#')

      raw_data_split = raw_data[-1].split(',')
      key = raw_data_split[0]

      if key in list(self.robot_state.keys()):
         try:
            self.robot_state[key] = [float(each) for each in raw_data_split[1:]]
         except ValueError:
            rospy.logwarn("ValueError occupied in parser_R1mini. %s ", raw_data_o)

   def set_periodic_info(self):
      target_info = ['ENCOD', 'DIFFV', 'VW', 'ACCL', 'GYRO']
      for idx, each in enumerate(target_info):
         self.write_port("$cREGI," + str(idx) + "," + each)

      self.update_battery_state()
      self.write_port("$cPERI,20")
      self.write_port("$cPEEN,1")

   def stop_periodic_comm(self):
      self.write_port("$cPEEN,0")
      sleep(0.1)
      self.write_port("$cODO,0")
      sleep(0.1)

   def set_wheel_velocity(self, l_vel, r_vel):
      self.write_port('$cDIFFV,{:.0f},{:.0f}'.format(l_vel, r_vel))

class OMOR1miniNode:
   def __init__(self):
      self.ph = PacketHandler()
      self.ph.stop_periodic_comm()  
      
      self.gear_ratio = 21.3
      self.wheel_distance = 0.17
      self.wheel_radius = 0.03375

      self.max_lin_vel_x = 1.2
      self.max_ang_vel_z = 1.0

      rospy.loginfo('Robot GEAR ratio: %s', self.gear_ratio)
      rospy.loginfo('Robot wheel_distance: %s', self.wheel_distance)
      rospy.loginfo('Robot wheel_radius: %s', self.wheel_radius)

      rospy.Subscriber("cmd_vel", Twist, self.sub_cmd_vel, queue_size=1)

      rospy.Timer(rospy.Duration(0.01), self.update_robot)

   def sub_cmd_vel(self, cmd_vel_msg):
      lin_vel_x = cmd_vel_msg.linear.x
      ang_vel_z = cmd_vel_msg.angular.z

      lin_vel_x = max(-self.max_lin_vel_x, min(self.max_lin_vel_x, lin_vel_x))
      ang_vel_z = max(-self.max_ang_vel_z, min(self.max_ang_vel_z, ang_vel_z))

      vel_l_wheel = (lin_vel_x - (self.wheel_distance / 2.0) * ang_vel_z) / self.wheel_radius
      vel_r_wheel = (lin_vel_x + (self.wheel_distance / 2.0) * ang_vel_z) / self.wheel_radius

      self.ph.set_wheel_velocity(vel_l_wheel * self.wheel_radius * 1000, 
                                 vel_r_wheel * self.wheel_radius * 1000)


   def update_robot(self, event):
      self.ph.parser_R1mini()

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

   def battery_status_server(self):
      bat_server = rospy.Service('battery_status', Battery, self.battery_service_handle)

   def main(self):
      self.ph.stop_periodic_comm()
      self.ph.update_battery_state()
      self.ph.set_periodic_info()
      self.battery_status_server()
      rospy.spin()

if __name__ == '__main__':
    rospy.init_node('omo_r1mini_node')
    node = OMOR1miniNode()
    node.main()