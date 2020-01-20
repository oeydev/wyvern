#!/usr/bin/env python2
from __future__ import print_function
import rospy
import argparse
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry
import math

parse = argparse.ArgumentParser(description="Pose2D path recorder input topic:= /robot_pose")
parse.add_argument("-n", "--len", help="path length", default=100, type=float)
parse.add_argument("-c", help="output format c++ array", action="store_const", default=False, const=True)
args = parse.parse_args()

DESCRIPTION = """
input position topic    : /robot_pose (Pose2D)
path length             : {}
c++ format              : {}
""".format(args.len, args.c)

x_path = []
y_path = []
theta_path = []
dir_path = []

twist = Twist()
old_direction = 1


def pose_callback(data):
    global x_path, y_path, theta_path, twist, old_direction
    x_path.append(data.x)
    y_path.append(data.y)
    theta_path.append(data.theta)
    if twist.linear.x > 1e-2:
        # forward
        old_direction = 1
    elif twist.linear.x < -1e-2:
        # backward
        old_direction = 0
    else:
        # same
        old_direction = old_direction

    dir_path.append(old_direction)


def odom_callback(data):
    global twist
    twist = data.twist.twist


def print_path():
    global x_path, y_path, theta_path, dir_path
    plen = len(x_path)
    n = plen / args.len
    n = int(math.ceil(n))
    n = max(n, 1)
    x_path = x_path[0::n]
    y_path = y_path[0::n]
    theta_path = theta_path[0::n]
    dir_path = dir_path[0::n]
    print("\n")
    if not args.c:

        print("x_path = {}".format(x_path))
        print("\n\n")
        print("y_path = {}".format(y_path))
        print("\n\n")
        print("theta_path = {}".format(theta_path))
        print("\n\n")
        print("dir_path = {}".format(dir_path))
    if args.c:
        x_str = "{}".format(x_path)[1:-1]
        y_str = "{}".format(y_path)[1:-1]
        theta_str = "{}".format(theta_path)[1:-1]
        dir_str = "{}".format(dir_path)[1:-1]
        print("x_path[]= {" + x_str + "};")
        print("\n\n")
        print("y_path[]= {" + y_str + "};")
        print("\n\n")
        print("theta_path[]= {" + theta_str + "};")
        print("\n\n")
        print("dir_path[]= {" + dir_str + "};")
    print("record_length={}\tout_length={}\tselect_every={}".format(plen, len(x_path), n))


def main():
    rospy.init_node("pose2d_path_recorder")
    print(DESCRIPTION)
    rospy.on_shutdown(print_path)

    rospy.Subscriber("/robot_pose", Pose2D, pose_callback)
    rospy.Subscriber("/odom", Odometry, odom_callback)
    rospy.spin()


if __name__ == "__main__":
    main()
