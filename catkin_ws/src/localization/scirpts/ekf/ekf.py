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
SIM_TIME = 50.0  # simulation time [s]

show_animation = False


def calc_input():
    global cmd_input
    v = cmd_input[0]  # [m/s]
    yawrate = cmd_input[1]  # [rad/s]
    u = np.array([[v], [yawrate]])
    return u


def observation(xTrue, xd, u):
    xTrue = np.array([[gps[0]], [gps[1]], [orientation[2]], [u[0]]]) # x, y, yaw, v

    # add noise to gps x-y
    z = observation_model(xTrue) + GPS_NOISE @ np.random.randn(2, 1)
    # z = np.array([[gps[0]], [gps[1]]])
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
    # print(F @ x)
    # print(x)
    # print(B)
    # print(u)
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
    #  Predict
    xPred = motion_model(xEst, u)
    jF = jacob_f(xEst, u)
    PPred = jF @ PEst @ jF.T + Q

    #  Update
    jH = jacob_h()
    zPred = observation_model(xPred)
    y = z - zPred
    S = jH @ PPred @ jH.T + R
    K = PPred @ jH.T @ np.linalg.inv(S)
    xEst = xPred + K @ y
    PEst = (np.eye(len(xEst)) - K @ jH) @ PPred
    return xEst, PEst


def plot_covariance_ellipse(xEst, PEst):  # pragma: no cover
    Pxy = PEst[0:2, 0:2]
    eigval, eigvec = np.linalg.eig(Pxy)

    if eigval[0] >= eigval[1]:
        bigind = 0
        smallind = 1
    else:
        bigind = 1
        smallind = 0

    t = np.arange(0, 2 * math.pi + 0.1, 0.1)
    a = math.sqrt(eigval[bigind])
    b = math.sqrt(eigval[smallind])
    x = [a * math.cos(it) for it in t]
    y = [b * math.sin(it) for it in t]
    angle = math.atan2(eigvec[1, bigind], eigvec[0, bigind])
    rot = Rot.from_euler('z', angle).as_matrix()[0:2, 0:2]
    fx = rot @ (np.array([x, y]))
    px = np.array(fx[0, :] + xEst[0, 0]).flatten()
    py = np.array(fx[1, :] + xEst[1, 0]).flatten()
    plt.plot(px, py, "--r")

cmd_input = [None, None]
gps = [None, None, None]
orientation = [None, None, None] # pitch, roll, yaw
def gps_cb(data):
    global gps
    gps[0] = data.latitude
    gps[1] = -data.longitude
    gps[2] = data.altitude
    # print(lat, lon , alti)

def cmd_cb(data):
    global cmd_input
    cmd_input[0] = data.linear.x
    cmd_input[1] = data.angular.z

def imu_cb(data):
    global orientation
    orientation = euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])

def main():
    print(__file__ + " start!!")
    rospy.init_node('ekf', anonymous=True)
    pid = sys.argv[1]
    rospy.Subscriber(f"/TurtleBot3Burger_{pid}_alfred_Aspire_VX5_591G/gps/values", NavSatFix, gps_cb)
    rospy.Subscriber(f"/TurtleBot3Burger_{pid}_alfred_Aspire_VX5_591G/inertial_unit/roll_pitch_yaw", Imu, imu_cb)
    rospy.Subscriber("cmd_vel", Twist, cmd_cb)
    ros_pub = ros_publisher()
    while None in cmd_input or None in gps or None in orientation:
        continue
    r = rospy.Rate(30)
    time = 0.0
    # State Vector [x y yaw v]'
    xEst = np.zeros((4, 1)) # estimate state
    xTrue = np.zeros((4, 1)) # True state
    PEst = np.eye(4)

    xDR = np.zeros((4, 1))  # Dead reckoning

    while not rospy.is_shutdown():
        time += DT
        u = calc_input()
        xTrue, z, xDR, ud = observation(xTrue, xDR, u)
        xEst, PEst = ekf_estimation(xEst, PEst, z, ud)
        # print(f"x: {xTrue[0]},y: {xTrue[1]},yaw: {xTrue[2]},v: {xTrue[3]}")
        # trajectory pub
        ros_pub.publish(xTrue, xEst, xDR)
        r.sleep()

if __name__ == '__main__':
    main()