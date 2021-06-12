#!/usr/bin/env python3
import math
import numpy as np
import rospy
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Twist, PoseStamped
from publisher import ros_publisher
from tf.transformations import *
import sys
import scipy.linalg
# Covariance for UKF simulation
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

#  UKF Parameter
ALPHA = 0.001
BETA = 2
KAPPA = 0


class Turtlebot3_state:
    def __init__(self):
        self.cmd_input = [None, None]  # cmd input: v, yaw_rate
        self.gps = [None, None, None]  # gps of TurtleBot3
        self.orientation = [None, None, None]  # pitch, roll, yaw
        self.landmarks = [None] * 4  # 4 landmark's position

def calc_input():

    v = turtlebot3_state.cmd_input[0]  # [m/s]
    yawrate = turtlebot3_state.cmd_input[1]  # [rad/s]
    u = np.array([[v, yawrate]]).T
    return u


def observation(xd, u):
    xTrue = np.array([[turtlebot3_state.gps[0]], [turtlebot3_state.gps[1]], [turtlebot3_state.orientation[2]], [u[0][0]]]) # x, y, yaw, v
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
        P = wc[0, i] * d[:, i:i + 1] @ d[:, i:i + 1].T + P
    return P

def predict_sigma_observation(sigma):
    """
        Sigma Points prediction with observation model
    """
    for i in range(sigma.shape[1]):
        sigma[0:2, i] = observation_model(sigma[:, i])

    sigma = sigma[0:2, :]

    return sigma

def calc_cxz(sigma, x, z_sigma, zb, wc):
    nSigma = sigma.shape[1]
    dx = sigma - x
    dz = z_sigma - zb[0:2]
    P = np.zeros((dx.shape[0], dz.shape[0]))

    for i in range(nSigma):
        P = P + wc[0, i] * dx[:, i:i + 1] @ dz[:, i:i + 1].T

    return P

def ukf_estimation(xEst, PEst, z, u, wm, wc, gamma):
    #  Predict:
    sigma = generate_sigma_points(xEst, PEst, gamma) # Given estimates of the mean and covariance and generate sigma points
    sigma = predict_sigma_motion(sigma, u) # (1)
    xPred = (wm @ sigma.T).T # (2)

    PPred = calc_sigma_covariance(xPred, sigma, wc, Q) # (3)

    #  Update:

    sigma = generate_sigma_points(xPred, PPred, gamma) # Given prediction estimates, generate a new set of sigma points
    z_sigma = predict_sigma_observation(sigma) # (4)
    zPred = (wm @ z_sigma.T).T # (5)
    st = calc_sigma_covariance(zPred, z_sigma, wc, R) # (6)

    Cxz = calc_cxz(sigma, xPred, z_sigma, zPred, wc) # (7)
    K = Cxz @ np.linalg.inv(st) # (8)
    y_residual = z - zPred
    xEst = xPred + K @ y_residual # (9)
    PEst = PPred - K @ st @ K.T # (10)

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
    rospy.init_node('ukf', anonymous=True)
    pid = sys.argv[1]
    rospy.Subscriber(f"/{pid}/gps/values", NavSatFix, gps_cb)
    rospy.Subscriber(f"/{pid}/inertial_unit/roll_pitch_yaw", Imu, imu_cb)
    rospy.Subscriber("cmd_vel", Twist, cmd_cb)
    ros_pub = ros_publisher()
    while None in turtlebot3_state.cmd_input or None in turtlebot3_state.gps or None in turtlebot3_state.orientation:
        continue
    r = rospy.Rate(30)

    nx = 4 # State Vector [x y yaw v]'
    xEst = np.zeros((nx, 1)) # estimate state
    xTrue = np.zeros((nx, 1)) # True state
    PEst = np.eye(nx)
    xDR = np.zeros((nx, 1))  # Dead reckoning
    wm, wc, gamma = setup_ukf(nx)
    while not rospy.is_shutdown():
        u = calc_input()
        xTrue, z, xDR, ud = observation(xDR, u)
        xEst, PEst = ukf_estimation(xEst, PEst, z, ud, wm, wc, gamma)
        # trajectory pub
        ros_pub.publish(xTrue, xEst, xDR)
        r.sleep()

if __name__ == '__main__':
    main()