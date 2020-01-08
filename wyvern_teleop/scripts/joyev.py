#!/usr/bin/env python
# Copyright (c) 2018, CoXSys, Inc.
# * joy2vel : 16 July 2018
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
with open(dir_path+"/../config/keymap.yaml") as file:
    config = yaml.load(file)


axes = [0]*len(config[3].keys())
buttons = [0]*len(config[1].keys())


rospy.init_node("joy")
joy_pb = rospy.Publisher("/joy", Joy, queue_size=10)
joy = Joy()
seq = 1
port = rospy.get_param("joyevent_port", "event3")
dev = InputDevice('/dev/input/'+port)
try:
    while True:
        r, w, x = select([dev], [], [])
        # code, type, val
        for event in dev.read():
            if event.type in config.keys() and event.code not in config["ignore"]["code"]:
                if event.type == 3:
                    if event.code in config["digital_axes"].keys():
                        val = -1*event.value
                        axes[config["digital_axes"]
                             [event.code]["order"]] = val
                    else:
                        if event.value in [127, 128]:
                            val = 0
                        else:
                            val = -1*translate(event.value, 0, 255, -1.0, 1.0)
                        axes[config[event.type][event.code]["order"]] = val
                elif event.type == 1:
                    buttons[config[event.type][event.code]
                            ["order"]] = event.value

                joy.header.seq = seq
                joy.axes = axes
                joy.buttons = buttons
                joy_pb.publish(joy)
                seq += 1
except KeyboardInterrupt:
    exit()
