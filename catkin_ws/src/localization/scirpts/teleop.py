#!/usr/bin/env python3
import sys

import rospy
from geometry_msgs.msg import Twist
from getkey import getkey, keys
import numpy as np
rospy.init_node('teleop', anonymous=True)
cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
r = rospy.Rate(30)
cmd = Twist()
print("--- Teleop your turtlebot3 ---")
print("W: Move forward\nX: Move backward\nA: Rotate positive Z\nD: Rotate negative Z\nS: Stop")
print("------------------------------")
while not rospy.is_shutdown():
    key = getkey()
    if key == "w":
        cmd.linear.x += 0.01
    elif key == "x":
        cmd.linear.x -= 0.01
    elif key == "a":
        cmd.angular.z += 0.05
    elif key == "d":
        cmd.angular.z -= 0.05
    elif key == "s":
        cmd.linear.x = 0
        cmd.angular.z = 0

    cmd.linear.x = np.clip(cmd.linear.x, -0.22, 0.22)
    cmd.angular.z = np.clip(cmd.angular.z, -2.84, 2.84)
    cmd_pub.publish(cmd)
    r.sleep()


