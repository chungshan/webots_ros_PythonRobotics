#!/usr/bin/env python3

import rospy
from webots_ros.srv import *
from geometry_msgs.msg import Twist
import numpy as np
import sys

rospy.init_node('slam', anonymous=True)
pid = sys.argv[1]
righ_wheel_set_pos = rospy.ServiceProxy(f'/{pid}/right_wheel_motor/set_position', set_float)
left_wheel_set_pos = rospy.ServiceProxy(f'/{pid}/left_wheel_motor/set_position', set_float)
righ_wheel_set_vel = rospy.ServiceProxy(f'/{pid}/right_wheel_motor/set_velocity', set_float)
left_wheel_set_vel = rospy.ServiceProxy(f'/{pid}/left_wheel_motor/set_velocity', set_float)
righ_wheel_set_pos(float("inf"))
left_wheel_set_pos(float("inf"))
righ_wheel_set_vel(0)
left_wheel_set_vel(0)

# lidar_motor
lds_main_motor_set_pos = rospy.ServiceProxy(f'/{pid}/LDS_01_main_motor/set_position', set_float)
lds_sec_motor_set_pos = rospy.ServiceProxy(f'/{pid}/LDS_01_secondary_motor/set_position', set_float)
lds_main_motor_set_vel = rospy.ServiceProxy(f'/{pid}/LDS_01_main_motor/set_velocity', set_float)
lds_sec_motor_set_vel = rospy.ServiceProxy(f'/{pid}/LDS_01_secondary_motor/set_velocity', set_float)
lds_main_motor_set_pos(float("inf"))
lds_sec_motor_set_pos(float("inf"))
lds_main_motor_set_vel(30.0)
lds_sec_motor_set_vel(60.0)

# lidar
lds_enable = rospy.ServiceProxy(f'/{pid}/LDS_01/enable', set_int)
lds_pcl_enable = rospy.ServiceProxy(f'/{pid}/LDS_01/enable_point_cloud', set_bool)
lds_enable(int(32))
lds_pcl_enable(True)

# gps
gps_enable = rospy.ServiceProxy(f'/{pid}/gps/enable', set_int)
gps_enable(int(32))

# imu: get pitch, roll, yaw of agv
imu_enable = rospy.ServiceProxy(f'/{pid}/inertial_unit/enable', set_int)
imu_enable(int(32))

def cmd_cb(data):
    L = 0.15492957746
    r = 0.033
    A = np.mat([[1, 1], [1, -1]])
    B = np.mat([2 * data.linear.x / r, data.angular.z * L / r]).T
    r = np.linalg.solve(A, B)
    righ_wheel_set_vel(r[0])
    left_wheel_set_vel(r[1])

rospy.Subscriber("cmd_vel", Twist, cmd_cb)
r = rospy.Rate(30)

while not rospy.is_shutdown():
    r.sleep()