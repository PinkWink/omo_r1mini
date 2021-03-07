#!/usr/bin/env python

import serial
import rospy
import io
from time import sleep

class PacketHandler:
   def __init__(self):
      port_name = rospy.get_param('~port', '/dev/ttyMotor')
      baud_rate = rospy.get_param('~baud', 115200)

      self.ser = serial.Serial(port_name, baud_rate)
      self.ser_io = io.TextIOWrapper(io.BufferedRWPair(self.ser, self.ser, 1), 
                                       newline = '\r', 
                                       line_buffering = True)

      self.stop_periodic_comm()
      self.ser.reset_input_buffer() 
      self.ser.reset_output_buffer() 

      self.robot_state = {
            "ENCOD" : [0., 0.],
            "VW" : [0., 0.],
            "ODO" : [0., 0.],
            "ACCL" : [0., 0., 0.],
            "GYRO" : [0., 0., 0.],
            "BAT" : [0., 0., 0.],
      }

      self.incomming_info = ['ODO', 'VW', 'POSE', 'ACCL', 'GYRO']
      
      rospy.loginfo('Serial port: %s', port_name)
      rospy.loginfo('Serial baud rate: %s', baud_rate)

   def get_port_state(self):
      return self.ser.isOpen()

   def read_port(self):
      try:
         return self.ser_io.readline()
      except UnicodeDecodeError:
         rospy.logwarn('UnicodeDecodeError during serial comm. start byte')

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

   def parser(self):
      raw_data_o = self.read_packet()

      try:
         raw_data = raw_data_o.replace('\r', '')
         raw_data = raw_data.replace('\n', '')
         raw_data = raw_data.split('#')

         raw_data_split = raw_data[-1].split(',')
         key = raw_data_split[0]
      except AttributeError:
         rospy.logwarn("AttributeError: 'NoneType' object has no attribute 'replace' : %s ")
         self.write_port("$cPEEN,1")

      if key in list(self.robot_state.keys()):
         try:
            self.robot_state[key] = [float(each) for each in raw_data_split[1:]]
         except ValueError:
            rospy.logwarn("ValueError occupied in parser_R1mini. %s ", raw_data_o)
            self.write_port("$cPEEN,1")

      return raw_data_o

   def set_periodic_info(self):
      for idx, each in enumerate(self.incomming_info):
         self.write_port("$cREGI," + str(idx) + "," + each)

      self.update_battery_state()
      self.write_port("$cPERI,100")
      sleep(0.01)
      self.write_port("$ENCOD,0")
      self.write_port("$cPEEN,1")
      sleep(0.01)
      self.write_port("$cPEEN,1")

   def stop_periodic_comm(self):
      self.write_port("$cPEEN,0")
      sleep(0.01)
      self.write_port("$cODO,0")
      sleep(0.01)

   def set_wheel_velocity(self, l_vel, r_vel):
      self.write_port('$cVW,{:.0f},{:.0f}'.format(l_vel, r_vel))