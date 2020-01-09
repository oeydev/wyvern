#!/usr/bin/env python2
from __future__ import print_function
import rospy
import argparse
from geometry_msgs.msg import Pose2D
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


def pose_callback(data):
    global x_path, y_path, theta_path
    x_path.append(data.x)
    y_path.append(data.y)
    theta_path.append(data.theta)


def print_path():
    global x_path, y_path, theta_path
    plen = len(x_path)
    n = plen / args.len
    n = int(math.ceil(n))
    n = max(n, 1)
    x_path = x_path[0::n]
    y_path = y_path[0::n]
    theta_path = theta_path[0::n]
    print("\n")
    if not args.c:

        print("x_path = {}".format(x_path))
        print("\n\n")
        print("y_path = {}".format(y_path))
        print("\n\n")
        print("theta_path = {}".format(theta_path))
    if args.c:
        x_str = "{}".format(x_path)[1:-1]
        y_str = "{}".format(y_path)[1:-1]
        theta_str = "{}".format(theta_path)[1:-1]
        print("x_path[]= {" + x_str + "};")
        print("\n\n")
        print("y_path[]= {" + y_str + "};")
        print("\n\n")
        print("theta_path[]= {" + theta_str + "};")
    print("record_length={}\tout_length={}\tselect_every={}".format(plen, len(x_path), n))


def main():
    rospy.init_node("pose2d_path_recorder")
    print(DESCRIPTION)
    rospy.on_shutdown(print_path)

    rospy.Subscriber("/robot_pose", Pose2D, pose_callback)
    rospy.spin()


if __name__ == "__main__":
    main()
