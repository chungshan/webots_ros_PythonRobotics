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
import scipy
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

#  UKF Parameter
ALPHA = 0.001
BETA = 2
KAPPA = 0

def calc_input():
    global cmd_input
    v = cmd_input[0]  # [m/s]
    yawrate = cmd_input[1]  # [rad/s]
    u = np.array([[v, yawrate]]).T
    return u


def observation(xTrue, xd, u):
    xTrue = np.array([[gps[0]], [gps[1]], [orientation[2]], [u[0][0]]]) # x, y, yaw, v
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


def setup_ukf(nx):
    lamb = ALPHA ** 2 * (nx + KAPPA) - nx
    # calculate weights
    wm = [lamb / (lamb + nx)]
    wc = [(lamb / (lamb + nx)) + (1 - ALPHA ** 2 + BETA)]
    for i in range(2 * nx):
        wm.append(1.0 / (2 * (nx + lamb)))
        wc.append(1.0 / (2 * (nx + lamb)))
    gamma = math.sqrt(nx + lamb)

    wm = np.array([wm])
    wc = np.array([wc])

    return wm, wc, gamma

def generate_sigma_points(xEst, PEst, gamma):
    sigma = xEst
    # print(PEst)
    Psqrt = scipy.linalg.sqrtm(PEst)
    n = len(xEst[:, 0])
    # Positive direction
    for i in range(n):
        sigma = np.hstack((sigma, xEst + gamma * Psqrt[:, i:i + 1]))

    # Negative direction
    for i in range(n):
        sigma = np.hstack((sigma, xEst - gamma * Psqrt[:, i:i + 1]))

    return sigma

def predict_sigma_motion(sigma, u):
    """
        Sigma Points prediction with motion model
    """
    for i in range(sigma.shape[1]):
        sigma[:, i:i + 1] = motion_model(sigma[:, i:i + 1], u)

    return sigma

def calc_sigma_covariance(x, sigma, wc, Pi):
    nSigma = sigma.shape[1]
    d = sigma - x[0:sigma.shape[0]]
    P = Pi
    for i in range(nSigma):
        P = P + wc[0, i] * d[:, i:i + 1] @ d[:, i:i + 1].T
    return P

def predict_sigma_observation(sigma):
    """
        Sigma Points prediction with observation model
    """
    for i in range(sigma.shape[1]):
        sigma[0:2, i] = observation_model(sigma[:, i])

    sigma = sigma[0:2, :]

    return sigma

def calc_pxz(sigma, x, z_sigma, zb, wc):
    nSigma = sigma.shape[1]
    dx = sigma - x
    dz = z_sigma - zb[0:2]
    P = np.zeros((dx.shape[0], dz.shape[0]))

    for i in range(nSigma):
        P = P + wc[0, i] * dx[:, i:i + 1] @ dz[:, i:i + 1].T

    return P

def ukf_estimation(xEst, PEst, z, u, wm, wc, gamma):
    #  Predict
    sigma = generate_sigma_points(xEst, PEst, gamma)
    # print(xEst)
    sigma = predict_sigma_motion(sigma, u)
    xPred = (wm @ sigma.T).T

    PPred = calc_sigma_covariance(xPred, sigma, wc, Q)

    #  Update
    zPred = observation_model(xPred)
    y = z - zPred
    sigma = generate_sigma_points(xPred, PPred, gamma)
    zb = (wm @ sigma.T).T
    z_sigma = predict_sigma_observation(sigma)
    st = calc_sigma_covariance(zb, z_sigma, wc, R)
    Pxz = calc_pxz(sigma, xPred, z_sigma, zb, wc)
    K = Pxz @ np.linalg.inv(st)
    xEst = xPred + K @ y
    PEst = PPred - K @ st @ K.T

    return xEst, PEst


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
    rospy.init_node('ukf', anonymous=True)
    pid = sys.argv[1]
    rospy.Subscriber(f"/TurtleBot3Burger_{pid}_alfred_Aspire_VX5_591G/gps/values", NavSatFix, gps_cb)
    rospy.Subscriber(f"/TurtleBot3Burger_{pid}_alfred_Aspire_VX5_591G/inertial_unit/roll_pitch_yaw", Imu, imu_cb)
    rospy.Subscriber("cmd_vel", Twist, cmd_cb)
    ros_pub = ros_publisher()
    while None in cmd_input or None in gps or None in orientation:
        continue
    r = rospy.Rate(30)
    time = 0.0
    nx = 4# State Vector [x y yaw v]'
    xEst = np.zeros((nx, 1)) # estimate state
    xTrue = np.zeros((nx, 1)) # True state
    PEst = np.eye(nx)
    xDR = np.zeros((nx, 1))  # Dead reckoning
    wm, wc, gamma = setup_ukf(nx)
    while not rospy.is_shutdown():
        time += DT
        u = calc_input()
        xTrue, z, xDR, ud = observation(xTrue, xDR, u)
        # print(xEst)
        xEst, PEst = ukf_estimation(xEst, PEst, z, ud, wm, wc, gamma)
        # print(xEst)
        # print(f"x: {xTrue[0]},y: {xTrue[1]},yaw: {xTrue[2]},v: {xTrue[3]}")
        # trajectory pub
        ros_pub.publish(xTrue, xEst, xDR)
        r.sleep()

if __name__ == '__main__':
    main()