#!/usr/bin/env python
# Copyright (c) 2018, CoXSys, Inc.
# * joy2vel : 20 August 2019
# * author : Sirawat Soksawatmakin
import rospy
import os
import yaml
from sensor_msgs.msg import Joy
from evdev import InputDevice, categorize, ecodes
from select import select
from time import sleep


def translate(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)


dir_path = os.path.dirname(os.path.realpath(__file__))
keys_info = dict()
with open(dir_path + "/../config/joy_bluetooth_keymap.yaml") as file:
    ev_config = yaml.load(file)
    for key, value in ev_config.items():
        keys_info["{}{}{}".format(value["code"], value["type"], value["value"])] = key

print(keys_info.keys())


axes = [0.0] * 6
buttons = [0.0] * 6

rospy.init_node("joy")
joy_pb = rospy.Publisher("/joy", Joy, queue_size=10)
joy = Joy()
seq = 1
port = rospy.get_param("~port", "event21")

dev = InputDevice('/dev/input/' + port)
try:
    while True:
        r, w, x = select([dev], [], [])
        # code, type, val
        for event in dev.read():
            keyword = "{}{}{}".format(event.code, event.type, event.value)
            if keyword in keys_info.keys():
                print(keys_info[keyword])
                if keys_info[keyword] == "x_left":
                    axes[3] = 1.0
                elif keys_info[keyword] == "x_center":
                    axes[3] = 0.0
                elif keys_info[keyword] == "x_right":
                    axes[3] = -1.0
                if keys_info[keyword] == "y_up":
                    axes[1] = 1.0
                elif keys_info[keyword] == "y_center":
                    axes[1] = 0.0
                elif keys_info[keyword] == "y_down":
                    axes[1] = -1.0

                if keys_info[keyword] == "C_up":
                    buttons[3] = 0.0
                elif keys_info[keyword] == "C_down":
                    buttons[3] = 1.0
                if keys_info[keyword] == "D_up":
                    buttons[0] = 0.0
                elif keys_info[keyword] == "D_down":
                    buttons[0] = 1.0
                if keys_info[keyword] == "B_up":
                    buttons[2] = 0.0
                elif keys_info[keyword] == "B_down":
                    buttons[2] = 1.0
                if keys_info[keyword] == "A_up":
                    buttons[1] = 0.0
                elif keys_info[keyword] == "A_down":
                    buttons[1] = 1.0
                if keys_info[keyword] == "T1_up":
                    buttons[4] = 0.0
                elif keys_info[keyword] == "T1_down":
                    buttons[4] = 1.0
                if keys_info[keyword] == "T2_up":
                    buttons[5] = 0.0
                elif keys_info[keyword] == "T2_down":
                    buttons[5] = 1.0

                joy.header.seq = seq
                joy.axes = axes
                joy.buttons = buttons
                joy_pb.publish(joy)
                seq += 1
except KeyboardInterrupt:
    exit()
