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

   def get_battery_state(self):
      self.write_port("$qBAT")
      return self.read_packet()

   def parser_R1mini(self):
      raw_data = self.read_packet()

      raw_data = raw_data.replace('\r', '')
      raw_data = raw_data.replace('\n', '')
      raw_data = raw_data.split('#')

      raw_data_split = raw_data[-1].split(',')
      key = raw_data_split[0]

      if key in list(self.robot_state.keys()):
         try:
            self.robot_state[key] = [float(each) for each in raw_data_split[1:]]
         except ValueError:
            rospy.logwarn("ValueError occupied in parser_R1mini.")

   def set_periodic_info(self):
      target_info = ['ENCOD', 'DIFFV', 'VW', 'ACCL', 'GYRO']
      for idx, each in enumerate(target_info):
         self.write_port("$cREGI," + str(idx) + "," + each)

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

      rospy.loginfo('Robot GEAR ratio: %s', self.gear_ratio)
      rospy.loginfo('Robot wheel_distance: %s', self.wheel_distance)
      rospy.loginfo('Robot wheel_radius: %s', self.wheel_radius)

      rospy.Timer(rospy.Duration(0.01), self.get_R1mini_state)

   def get_R1mini_state(self, event):
      self.ph.parser_R1mini()
      print self.ph.robot_state

   def main(self):
      self.ph.stop_periodic_comm()
      self.ph.set_periodic_info()
      rospy.spin()

if __name__ == '__main__':
    rospy.init_node('omo_r1mini_node')
    node = OMOR1miniNode()
    node.main()