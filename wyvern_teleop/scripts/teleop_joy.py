#!/usr/bin/env python
# Copyright (c) 2018, CoXSys Robotics, Inc.
# * joy2vel : 2 July 2018
# * author : Sirawat Soksawatmakin
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import argparse


msg = """
Control Your Robot!
---------------------------
Moving around by Analog (Don't forget enable MODE on joystick) :
    Linear -- Left Analog
    Angular -- Right Analog
CTRL-C to quit
"""

e = """
Communications Failed
"""


class joy2vel():

    def __init__(self):

        self.omni = rospy.get_param("/config/omni", default=False)
        self.config = rospy.get_param("~keymap")
        self.LIMIT_LIN_VEL = rospy.get_param("/config/limit_lin_vel", 1.0)
        self.LIMIT_ANG_VEL = rospy.get_param("/config/limit_ang_vel", 1.8)
        self.LIN_VEL_STEP_SIZE = self.LIMIT_LIN_VEL / 4.0
        self.ANG_VEL_STEP_SIZE = self.LIMIT_ANG_VEL / 4.0
        self.LIN_VEL = self.LIMIT_LIN_VEL / 2.0
        self.ANG_VEL = self.LIMIT_ANG_VEL / 2.0
        self.SEQ = 0

        self.sub_joy = rospy.Subscriber('joy', Joy, self.callback)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        self.twist = Twist()
        self.emergency_break = False

    def callback(self, data):

        if data.buttons[self.config["break"]] == 1 and not self.emergency_break:
            self.emergency_break = True
            rospy.set_param("/config/emergency_break", True)
            self.reset()
            return 0
        elif data.buttons[self.config["break"]] == 0 and self.emergency_break:
            self.emergency_break = False
            rospy.set_param("/config/emergency_break", False)

        if not self.emergency_break:
            if data.buttons[self.config["lin_speed_up"]] == 1 and self.SEQ != data.header.seq:
                self.LIN_VEL += self.LIN_VEL_STEP_SIZE
                if self.LIN_VEL > self.LIMIT_LIN_VEL:
                    self.LIN_VEL = self.LIMIT_LIN_VEL

            elif data.buttons[self.config["lin_speed_down"]] == 1 and self.SEQ != data.header.seq:
                self.LIN_VEL -= self.LIN_VEL_STEP_SIZE
                if self.LIN_VEL < 0:
                    self.LIN_VEL = 0
            elif data.buttons[self.config["ang_speed_up"]] == 1 and self.SEQ != data.header.seq:
                self.ANG_VEL += self.ANG_VEL_STEP_SIZE
                if self.ANG_VEL > self.LIMIT_ANG_VEL:
                    self.ANG_VEL = self.LIMIT_ANG_VEL
            elif data.buttons[self.config["ang_speed_down"]] == 1 and self.SEQ != data.header.seq:
                self.ANG_VEL -= self.ANG_VEL_STEP_SIZE
                if self.ANG_VEL < 0:
                    self.ANG_VEL = 0
            elif data.buttons[self.config["reset"]] == 1 and self.SEQ != data.header.seq:
                self.LIN_VEL = self.LIMIT_LIN_VEL / 2.0
                self.ANG_VEL = self.LIMIT_ANG_VEL / 2.0
            # else:
            #     self.LIN_VEL = rospy.get_param("~max_lin_vel")
            #     self.ANG_VEL = rospy.get_param("~max_ang_vel")

            vel_x = data.axes[self.config["lin_vel_x"]]
            if abs(vel_x) < 0.2:
                vel_x = 0.0
            self.twist.linear.x = vel_x * self.LIN_VEL
            if self.omni:
                vel_y = -data.axes[self.config["lin_vel_y"]]
                if abs(vel_y) < 0.2:
                    vel_y = 0.0
                self.twist.linear.y = vel_y * self.LIN_VEL
            ang_z = data.axes[self.config["ang_vel_z"]]
            if abs(ang_z) < 0.2:
                ang_z = 0.0
            self.twist.angular.z = ang_z * self.ANG_VEL
            self.SEQ = data.header.seq
            self.LIMIT_LIN_VEL = rospy.get_param("/config/limit_lin_vel", 0.6)
            self.LIMIT_ANG_VEL = rospy.get_param("/config/limit_ang_vel", 1.0)
            self.LIN_VEL_STEP_SIZE = self.LIMIT_LIN_VEL / 4.0
            self.ANG_VEL_STEP_SIZE = self.LIMIT_ANG_VEL / 4.0

            self.pub.publish(self.twist)

    def reset(self):
        self.twist = Twist()
        self.pub.publish(self.twist)


if __name__ == "__main__":

    rospy.init_node('teleop_joy_to_velocity')
    joycmd = joy2vel()
    try:
        print msg
        rospy.spin()

    except BaseException:
        print e

    finally:
        joycmd.reset()
