#!/usr/bin/env python
import rospy
import roslaunch
import subprocess
import signal
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion

last_id = 1
flag = False
x, y, yaw = 0, 0, 0
package_name = ""


def init_callback(data):
    global last_id, flag, x, y, yaw
    flag = True
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    q = data.pose.pose.orientation
    yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
    # x = round(x, 3)
    # y = round(y, 3)
    # yaw = round(yaw, 3)

    finish_traj = subprocess.Popen(["rosservice", "call", "/finish_trajectory", str(last_id)])
    rospy.sleep(0.5)
    start_new_traj = subprocess.Popen(["roslaunch", package_name, "start_trajectory.launch",
                                       "x:={}".format(x),
                                       "y:={}".format(y),
                                       "yaw:={}".format(yaw)])

    last_id += 1


rospy.init_node("cartographer_initializer")
package_name = rospy.get_param("~package_name", default="")
if package_name == "":
    print("Required package_name!")
    exit()

rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, init_callback)

rospy.spin()
# rate = rospy.Rate(5)
# while not rospy.is_shutdown():
#     if flag:
#         flag = False
#         cli_args = ['/home/oey/catkin_ws/src/powered_pallet/powered_pallet_bringup/launch/start_trajectory.launch',
#                     'x:={}'.format(x),
#                     'y:={}'.format(y),
#                     'yaw:={}'.format(yaw)]
#         roslaunch_args = cli_args[1:]
#         roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

#         uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
#         parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
#         parent.start()
#     rate.sleep()
