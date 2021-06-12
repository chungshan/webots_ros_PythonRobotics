#!/usr/bin/env python3
import math
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as Rot
import rospy
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Twist, PoseStamped
from publisher import ros_publisher
from tf.transformations import *
import sys
# Covariance for EKF simulation
Q = np.diag([
    0.1,  # variance of location on x-axis
    0.1,  # variance of location on y-axis
    np.deg2rad(1.0),  # variance of yaw angle
    1.0  # variance of velocity
]) ** 2  # predict state covariance
R = np.diag([1.0, 1.0]) ** 2  # Observation x,y position covariance

#  Simulation parameter
INPUT_NOISE = np.diag([0.5, np.deg2rad(1)]) ** 2
GPS_NOISE = np.diag([0.05, 0.05]) ** 2

DT = 1/30  # time tick [s]

class Turtlebot3_state:
    def __init__(self):
        self.cmd_input = [None, None]  # cmd input: v, yaw_rate
        self.gps = [None, None, None]  # gps of TurtleBot3
        self.orientation = [None, None, None]  # pitch, roll, yaw
        self.landmarks = [None] * 4  # 4 landmark's position

def calc_input():
    v = turtlebot3_state.cmd_input[0]  # [m/s]
    yawrate = turtlebot3_state.cmd_input[1]  # [rad/s]
    u = np.array([[v], [yawrate]])
    return u


def observation(xd, u):
    xTrue = np.array([[turtlebot3_state.gps[0]], [turtlebot3_state.gps[1]], [turtlebot3_state.orientation[2]], [u[0]]]) # x, y, yaw, v

    # add noise to gps x-y
    z = observation_model(xTrue) + GPS_NOISE @ np.random.randn(2, 1)
    # add noise to input
    ud = u + INPUT_NOISE @ np.random.randn(2, 1)

    xd = motion_model(xd, ud)

    return xTrue, z, xd, ud


def motion_model(x, u):
    F = np.array([[1.0, 0, 0, 0],
                  [0, 1.0, 0, 0],
                  [0, 0, 1.0, 0],
                  [0, 0, 0, 0]])
    B = np.array([[DT * math.cos(x[2, 0]), 0],
                  [DT * math.sin(x[2, 0]), 0],
                  [0.0, DT],
                  [1.0, 0.0]])

    x = F @ x + B @ u

    return x


def observation_model(x):
    H = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0]
    ])

    z = H @ x

    return z


def jacob_f(x, u):
    """
    Jacobian of Motion Model

    motion model
    x_{t+1} = x_t+v*dt*cos(yaw)
    y_{t+1} = y_t+v*dt*sin(yaw)
    yaw_{t+1} = yaw_t+omega*dt
    v_{t+1} = v{t}
    so
    dx/dyaw = -v*dt*sin(yaw)
    dx/dv = dt*cos(yaw)
    dy/dyaw = v*dt*cos(yaw)
    dy/dv = dt*sin(yaw)
    """
    yaw = x[2, 0]
    v = u[0, 0]
    jF = np.array([
        [1.0, 0.0, -DT * v * math.sin(yaw), DT * math.cos(yaw)],
        [0.0, 1.0, DT * v * math.cos(yaw), DT * math.sin(yaw)],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]])

    return jF


def jacob_h():
    # Jacobian of Observation Model
    jH = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0]
    ])

    return jH


def ekf_estimation(xEst, PEst, z, u):
    # Predict
    xPred = motion_model(xEst, u) # (1)
    jF = jacob_f(xEst, u) # Jacobian of state transition
    PPred = jF @ PEst @ jF.T + Q # (2)

    # Update
    jH = jacob_h() # Jacobian of observation matrix
    zPred = observation_model(xPred)
    y_residual = z - zPred # (3)
    S = jH @ PPred @ jH.T + R # (4)
    K = PPred @ jH.T @ np.linalg.inv(S) # (5)
    xEst = xPred + K @ y_residual # (6)
    PEst = (np.eye(len(xEst)) - K @ jH) @ PPred # (7)
    return xEst, PEst

turtlebot3_state = Turtlebot3_state()
def gps_cb(data):
    turtlebot3_state.gps = [data.latitude, -data.longitude, data.altitude]

def cmd_cb(data):
    turtlebot3_state.cmd_input = [data.linear.x, data.angular.z]

def imu_cb(data):
    turtlebot3_state.orientation = euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])

def main():
    print(__file__ + " start!!")
    rospy.init_node('ekf', anonymous=True)
    pid = sys.argv[1]
    rospy.Subscriber(f"/{pid}/gps/values", NavSatFix, gps_cb)
    rospy.Subscriber(f"/{pid}/inertial_unit/roll_pitch_yaw", Imu, imu_cb)
    rospy.Subscriber("cmd_vel", Twist, cmd_cb)
    r = rospy.Rate(30)
    ros_pub = ros_publisher()
    while None in turtlebot3_state.cmd_input or None in turtlebot3_state.gps or None in turtlebot3_state.orientation:
        r.sleep()
    # State Vector [x y yaw v]'
    xEst = np.zeros((4, 1)) # estimate state
    xTrue = np.zeros((4, 1)) # True state
    PEst = np.eye(4)

    xDR = np.zeros((4, 1))  # Dead reckoning

    while not rospy.is_shutdown():
        u = calc_input()
        xTrue, z, xDR, ud = observation(xDR, u)
        xEst, PEst = ekf_estimation(xEst, PEst, z, ud)
        # trajectory pub
        ros_pub.publish(xTrue, xEst, xDR)
        r.sleep()

if __name__ == '__main__':
    main()