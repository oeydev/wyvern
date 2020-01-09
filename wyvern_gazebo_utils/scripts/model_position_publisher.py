#!/usr/bin/env python2
from __future__ import print_function
import rospy
import argparse
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose2D, Twist
from tf.transformations import euler_from_quaternion

parse = argparse.ArgumentParser(description="Publish gazebo robot's position topic:= /robot_pose")
parse.add_argument("model", help="model name", type=str)
parse.add_argument("-f", "--freq", help="publish frequency", default=30, type=float)
args = parse.parse_args()

DESCRIPTION = """
robot's model : {}
position topic         : /robot_pose (Pose2D)
velocity topic         : /robot_vel  (Twist)
freq          : {}
""".format(args.model, args.freq)

current_pose = Pose2D()
current_twist = Twist()


def model_callback(data):
    global current_pose, current_twist
    index = data.name.index(args.model)

    q = data.pose[index].orientation
    roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

    current_pose.x = data.pose[index].position.x
    current_pose.y = data.pose[index].position.y
    current_pose.theta = yaw
    current_twist = data.twist[index]


def main():
    rospy.init_node("gazebo_robot_pose_pub")
    r = rospy.Rate(args.freq)
    print(DESCRIPTION)

    rospy.Subscriber("/gazebo/model_states", ModelStates, model_callback)
    pose_pub = rospy.Publisher("/robot_pose", Pose2D, queue_size=10)
    vel_pub = rospy.Publisher("/robot_vel", Twist, queue_size=10)

    while not rospy.is_shutdown():
        pose_pub.publish(current_pose)
        vel_pub.publish(current_twist)

        r.sleep()


if __name__ == "__main__":
    main()
