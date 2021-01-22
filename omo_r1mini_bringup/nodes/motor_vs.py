#!/usr/bin/env python

import sys
import rospy
import math
from time import sleep
from omo_packet_handler import PacketHandler

class OMOR1miniMotorSetting:
    def __init__(self):
        self.ph = PacketHandler()
        self.ph.stop_periodic_comm()  

        self.gear_ratio = rospy.get_param("/motor_spec/gear_ratio")
        self.wheel_base = rospy.get_param("/motor_spec/wheel_base")
        self.wheel_radius = rospy.get_param("/motor_spec/wheel_radius")
        self.enc_pulse = rospy.get_param("/motor_spec/enc_pulse")

        rospy.loginfo('Robot GEAR ratio: %s', self.gear_ratio/10)
        rospy.loginfo('Robot wheel_base: %s', self.wheel_base)
        rospy.loginfo('Robot wheel_radius: %s', self.wheel_radius)
        rospy.loginfo('Robot enc_pulse: %s', self.enc_pulse)
        sleep(0.1)

    def get_gear_ratio(self):
        self.ph.ser.flushInput()
        command = "$qGEAR"
        self.ph.write_port(command)
        sleep(0.1)
        response = self.ph.read_packet()
        rospy.loginfo('Get command Robot gear_ratio: %s', response)
        return response

    def set_gear_ratio(self):
        command = "$sGEAR," + str(self.gear_ratio)
        self.ph.write_port(command)
        rospy.loginfo('Set command Robot gear_ratio: %s', command)
        sleep(0.1)

    def run_motors(self):
        cnt = 0
        while cnt < 6000:
            cnt += 1
            sleep(0.01)
            command = "$cRPM,12,12"
            self.ph.write_port(command)

    def main(self):
        self.set_gear_ratio()
        rospy.loginfo('Set Robot gear_ratio: %s', self.gear_ratio)
        current_gear_ratio = self.get_gear_ratio()
        sleep(1)
        rospy.loginfo('Now Robot gear_ratio is set as %s', current_gear_ratio)
        sleep(1)
        rospy.loginfo('Now Robot robot motor is running.')
        self.run_motors()

if __name__ == '__main__':
    rospy.init_node('omo_r1mini_motor_setting')
    node = OMOR1miniMotorSetting()
    node.main()