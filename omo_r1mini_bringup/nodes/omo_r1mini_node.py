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
from omo_r1mini_bringup.srv import ResetOdom, ResetOdomResponse
#from omo_r1mini_bringup.srv import Calg, CalgResponse

class OdomPose(object):
    x = 0.0
    y = 0.0
    theta = 0.0
    timestamp = 0
    pre_timestamp = 0

class OdomVel(object):
    x = 0.0
    y = 0.0
    w = 0.0

class Joint(object):
    joint_name = ['wheel_left_joint', 'wheel_right_joint']
    joint_pos = [0.0, 0.0]
    joint_vel = [0.0, 0.0]

class ComplementaryFilter():
    def __init__(self):
        self.theta = 0.
        self.pre_theta = 0.
        self.wheel_ang = 0.
        self.filter_coef = 2.5
        self.gyro_bias = 0.
        self.count_for_gyro_bias = 110

    def gyro_calibration(self, gyro):
        self.count_for_gyro_bias -= 1

        if self.count_for_gyro_bias > 100:
            return "Prepare for gyro_calibration"

        self.gyro_bias += gyro
        if self.count_for_gyro_bias == 1:
            self.gyro_bias /= 100
            rospy.loginfo('Complete : Gyro calibration')
            return "gyro_calibration OK"

        return "During gyro_calibration"

    def calc_filter(self, gyro, d_time):

        if self.count_for_gyro_bias != 1:
            tmp = self.gyro_calibration(gyro)
            return 0

        gyro -= self.gyro_bias

        self.pre_theta = self.theta
        temp = -1/self.filter_coef * (-self.wheel_ang + self.pre_theta) + gyro
        self.theta = self.pre_theta + temp*d_time

        #print self.theta*180/3.141, self.wheel_ang*180/3.141, gyro, d_time
        return self.theta

class OMOR1miniNode:
    def __init__(self):
        self.ph = PacketHandler()
        self.ph.ser.reset_input_buffer() 
        self.ph.ser.reset_output_buffer() 
        self.ph.stop_periodic_comm() 
        self.calc_yaw = ComplementaryFilter()

        self.max_lin_vel_x = 1.2
        self.max_ang_vel_z = 1.0

        self.ph.robot_state = {
            "VW" : [0., 0.],
            "ODO" : [0., 0.],
            "ACCL" : [0., 0., 0.],
            "GYRO" : [0., 0., 0.],
            "POSE" : [0., 0., 0.],
            "BAT" : [0., 0., 0.],
        } 

        self.ph.incomming_info = ['ODO', 'VW', "GYRO", "ACCL"]

        self.odom_pose = OdomPose()
        self.odom_vel = OdomVel()
        self.joint = Joint() 

        self.gear_ratio = rospy.get_param("/motor_spec/gear_ratio")/10.
        self.wheel_base = rospy.get_param("/motor_spec/wheel_base")
        self.wheel_radius = rospy.get_param("/motor_spec/wheel_radius")
        self.enc_pulse = rospy.get_param("/motor_spec/enc_pulse")
        self.use_gyro = rospy.get_param("/use_imu_during_odom_calc/use_imu")
        self.calc_yaw.filter_coef = rospy.get_param("/use_imu_during_odom_calc/complementary_filter_coef")

        self.distance_per_pulse = 2*math.pi*self.wheel_radius / self.enc_pulse / self.gear_ratio

        rospy.loginfo('Robot GEAR ratio: %s', self.gear_ratio)
        rospy.loginfo('Robot wheel_base: %s', self.wheel_base)
        rospy.loginfo('Robot wheel_radius: %s', self.wheel_radius)
        rospy.loginfo('Robot enc_pulse: %s', self.enc_pulse)
        rospy.loginfo('Robot distance_per_pulse: %s', self.distance_per_pulse)

        rospy.Service('battery_status', Battery, self.battery_service_handle)
        rospy.Service('set_led_color', Color, self.led_color_service_handle)
        rospy.Service('save_led_color', Color, self.save_led_color_service_handle)
        rospy.Service('reset_odom', ResetOdom, self.reset_odom_handle)
        #rospy.Service('calibrate_gyro', Calg, self.calibrate_gyro)

        rospy.Subscriber("cmd_vel", Twist, self.sub_cmd_vel, queue_size=1)

        self.pub_joint_states = rospy.Publisher('joint_states', JointState, queue_size=10)
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)
        self.odom_broadcaster = TransformBroadcaster()

        self.ph.update_battery_state()
        self.ph.set_periodic_info()

        sleep(0.1)

        self.ph.set_periodic_info()

        rospy.loginfo('==> Start R1mini ')
        rospy.Timer(rospy.Duration(0.01), self.update_robot)
        self.odom_pose.timestamp = rospy.Time.now()
        self.odom_pose.pre_timestamp = rospy.Time.now()

    def convert2odo_from_each_wheel(self, enc_l, enc_r):
        return enc_l * self.distance_per_pulse, enc_r * self.distance_per_pulse

    def update_odometry(self, odo_l, odo_r, trans_vel, orient_vel, vel_z):
        odo_l /= 1000.
        odo_r /= 1000.
        trans_vel /= 1000.
        orient_vel /= 1000.

        self.odom_pose.timestamp = rospy.Time.now()
        dt = (self.odom_pose.timestamp - self.odom_pose.pre_timestamp).to_sec()
        self.odom_pose.pre_timestamp = self.odom_pose.timestamp

        if self.use_gyro:
            self.calc_yaw.wheel_ang += orient_vel * dt
            self.odom_pose.theta = self.calc_yaw.calc_filter(vel_z*math.pi/180., dt)
            rospy.loginfo('R1mini state : whl pos %1.2f, %1.2f, gyro : %1.2f, whl odom : %1.2f, robot theta : %1.2f', 
                            odo_l, odo_r, vel_z,
                            self.calc_yaw.wheel_ang*180/math.pi, 
                            self.odom_pose.theta*180/math.pi )
        else:
            self.odom_pose.theta += orient_vel * dt
            rospy.loginfo('R1mini state : wheel pos %s, %s, speed %s, %s', 
                            odo_l, odo_r, trans_vel, orient_vel)

        d_x = trans_vel * math.cos(self.odom_pose.theta) 
        d_y = trans_vel * math.sin(self.odom_pose.theta) 

        self.odom_pose.x += d_x * dt
        self.odom_pose.y += d_y * dt

        odom_orientation_quat = quaternion_from_euler(0, 0, self.odom_pose.theta)

        self.odom_vel.x = trans_vel
        self.odom_vel.y = 0.
        self.odom_vel.w = orient_vel

        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"

        self.odom_broadcaster.sendTransform((self.odom_pose.x, self.odom_pose.y, 0.), 
                                                odom_orientation_quat, self.odom_pose.timestamp, 
                                                odom.child_frame_id, odom.header.frame_id)
      
        odom.header.stamp = rospy.Time.now()
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
            [vel_x, vel_y, vel_z] = self.ph.robot_state['GYRO']
            [acc_x, acc_y, acc_z] = self.ph.robot_state['ACCL']
            # [roll_imu, pitch_imu, yaw_imu] = self.ph.robot_state['POSE']

            self.update_odometry(odo_l, odo_r, trans_vel, orient_vel, vel_z)
            self.updateJointStates(odo_l, odo_r, trans_vel, orient_vel)

        except ValueError:
            rospy.logwarn("ValueError occupied during read robot status in update_robot state. \n\r Raw_data : %s", 
                            raw_data)
      
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

    def reset_odom_handle(self, req):
        self.odom_pose.x = req.x
        self.odom_pose.y = req.y
        self.odom_pose.theta = req.theta

        return ResetOdomResponse()

    # def calibrate_gyro(self, req):
    #     command = "$qCALG,1"
    #     self.ph.write_port(command)
    #     return CalgResponse()


    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('omo_r1mini_node')
    node = OMOR1miniNode()
    node.main()