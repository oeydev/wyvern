#!/usr/bin/env python
from __future__ import print_function
import rospy
from geometry_msgs.msg import Twist
import sys
import argparse
import select
import termios
import tty

parser = argparse.ArgumentParser(description="Send command velocity(Twist) by keyboard press")
parser.add_argument("topic", help="ros topic", default="/cmd_vel", type=str, nargs='?')
parser.add_argument("max_lin_vel", help="maximum linear velocity (x,y)", default=1, type=float, metavar='v', nargs='?')
parser.add_argument("max_ang_vel", help="maximum omega (angular velocity z)", default=1.5, type=float, metavar='w', nargs='?')
parser.add_argument("lin_vel_step_size", help="linear velocity step", default=0.1, type=float, metavar='v_s', nargs='?')
parser.add_argument("ang_vel_step_size", help="angular velocity step", default=0.1, type=float, metavar='w_s', nargs='?')

args = parser.parse_args()
try:
    TOPIC = args.topic
    MAX_LIN_VEL = args.max_lin_vel
    MAX_ANG_VEL = args.max_ang_vel
    LIN_VEL_STEP_SIZE = args.lin_vel_step_size
    ANG_VEL_STEP_SIZE = args.ang_vel_step_size
except BaseException:
    print("Args Failed!")


msg = """
Control Your Robot!
topic := {0}
---------------------------
Moving around:
        w
   a    s    d         f   g
        x

w/x : increase/decrease linear velocity (x) by {1}
f/g : increase/decrease linear velocity (y) by {1}
a/d : increase/decrease angular velocity (z) by {2}
max lienar velocity : {3}
max angular velocity : {4}

space key, s : force stop

CTRL-C to quit
""".format(TOPIC, LIN_VEL_STEP_SIZE, ANG_VEL_STEP_SIZE, MAX_LIN_VEL, MAX_ANG_VEL)

e = """
Communications Failed
"""


def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(target_linear_vel, target_angular_vel, xx):
    return "currently:\tlinear vel %s\t angular vel %s  linear_y %s" % (target_linear_vel, target_angular_vel, xx)


def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min(input, output + slop)
    elif input < output:
        output = max(input, output - slop)
    else:
        output = input

    return output


def constrain(input, low, high):
    if input < low:
        input = low
    elif input > high:
        input = high
    else:
        input = input

    return input


def checkLinearLimitVelocity(vel):
    vel = constrain(vel, -MAX_LIN_VEL, MAX_LIN_VEL)

    return vel


def checkAngularLimitVelocity(vel):
    vel = constrain(vel, -MAX_ANG_VEL, MAX_ANG_VEL)

    return vel


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('key_teleop')
    pub = rospy.Publisher(TOPIC, Twist, queue_size=10)
    # pub = rospy.Publisher('cmd_vel_chassis', Twist, queue_size=10)

    status = 0
    target_linear_vel = 0.0
    target_linear_vel_y = 0.0
    target_angular_vel = 0.0
    control_linear_vel = 0.0
    control_linear_vel_y = 0.0
    control_angular_vel = 0.0

    print(msg)
    pub_flag = False
    while(1):
        try:
            key = getKey()
            if key == 'w':
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel, target_angular_vel, target_linear_vel_y))
                pub_flag = True
            elif key == 'x':
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel, target_angular_vel, target_linear_vel_y))
                pub_flag = True
            elif key == 'a':
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel, target_angular_vel, target_linear_vel_y))
                pub_flag = True
            elif key == 'd':
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel - ANG_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel, target_angular_vel, target_linear_vel_y))
                pub_flag = True
            elif key == ' ' or key == 's':
                target_linear_vel = 0.0
                control_linear_vel = 0.0
                target_linear_vel_y = 0.0
                control_linear_vel_y = 0.0
                target_angular_vel = 0.0
                control_angular_vel = 0.0
                print(vels(target_linear_vel, target_angular_vel, target_linear_vel_y))
                pub_flag = True

            elif key == 'g':
                target_linear_vel_y = checkLinearLimitVelocity(target_linear_vel_y + LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel, target_angular_vel, target_linear_vel_y))
                pub_flag = True
            elif key == 'f':
                target_linear_vel_y = checkLinearLimitVelocity(target_linear_vel_y - LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel, target_angular_vel, target_linear_vel_y))
                pub_flag = True

            else:
                if (key == '\x03'):
                    twist = Twist()
                    twist.linear.x = 0.0
                    twist.linear.y = 0.0
                    twist.linear.z = 0.0
                    twist.angular.x = 0.0
                    twist.angular.y = 0.0
                    twist.angular.z = 0.0
                    pub.publish(twist)
                    break

            if status == 20:
                print(msg1)
                status = 0
            if pub_flag:
                pub_flag = False
                twist = Twist()

                control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE))
                twist.linear.x = control_linear_vel
                twist.linear.z = 0.0

                control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE))
                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = control_angular_vel

                control_linear_vel_y = makeSimpleProfile(control_linear_vel_y, target_linear_vel_y, (LIN_VEL_STEP_SIZE))
                twist.linear.y = control_linear_vel_y
                twist.linear.z = 0.0

                pub.publish(twist)

        except BaseException as this_e:
            print("{}\t:\t{}".format(e, this_e))

        # finally:
        #     twist = Twist()
        #     twist.linear.x = 0.0
        #     twist.linear.y = 0.0
        #     twist.linear.z = 0.0
        #     twist.angular.x = 0.0
        #     twist.angular.y = 0.0
        #     twist.angular.z = 0.0
        #     pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
