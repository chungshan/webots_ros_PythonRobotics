#!/usr/bin/env python3
import math
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as Rot
import rospy
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Twist, PoseStamped
from geometry_msgs.msg import PoseArray, Pose

from publisher import ros_publisher
from tf.transformations import *
import sys
import scipy

# Estimation parameter of PF
Q = np.diag([0.01]) ** 2  # range error
R = np.diag([0.5, np.deg2rad(10)]) ** 2  # input error

#  Simulation parameter
Q_sim = np.diag([0.01]) ** 2
R_sim = np.diag([0.5, np.deg2rad(10)]) ** 2

DT = 1/30  # time tick [s]
MAX_RANGE = 5  # maximum observation range
# Particle filter parameter
NP = 100  # Number of Particle
NTh = NP / 2.0  # Number of particle for re-sampling


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


def observation(xd, u, rf_id):
    x_true = np.array([[turtlebot3_state.gps[0]], [turtlebot3_state.gps[1]], [turtlebot3_state.orientation[2]], [u[0][0]]]) # x, y, yaw, v
    # add noise to gps x-y
    z = np.zeros((0, 3))
    for i in range(len(rf_id[:, 0])):

        dx = x_true[0, 0] - rf_id[i, 0]
        dy = x_true[1, 0] - rf_id[i, 1]
        d = math.hypot(dx, dy)
        if d <= MAX_RANGE:
            dn = d + np.random.randn() * Q_sim[0, 0] ** 0.5  # add noise
            zi = np.array([[dn, rf_id[i, 0], rf_id[i, 1]]])
            z = np.vstack((z, zi))

    # add noise to input
    ud1 = u[0, 0] + np.random.randn() * R_sim[0, 0] ** 0.5
    ud2 = u[1, 0] + np.random.randn() * R_sim[1, 1] ** 0.5
    ud = np.array([[ud1, ud2]]).T
    xd = motion_model(xd, ud)

    return x_true, z, xd, ud


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

def gauss_likelihood(x, sigma):
    p = 1.0 / math.sqrt(2.0 * math.pi * sigma ** 2) * \
        math.exp(-x ** 2 / (2 * sigma ** 2))

    return p

def calc_covariance(x_est, px, pw):
    """
    calculate covariance matrix
    see ipynb doc
    """
    cov = np.zeros((3, 3))
    n_particle = px.shape[1]
    for i in range(n_particle):
        dx = (px[:, i:i + 1] - x_est)[0:3]
        cov += pw[0, i] * dx @ dx.T
    cov *= 1.0 / (1.0 - pw @ pw.T)

    return cov

def pf_localization(px, pw, z, u):
    """
    Localization with Particle filter
    """

    for ip in range(NP):
        x = np.array([px[:, ip]]).T
        w = pw[0, ip]

        #  Predict with random input sampling
        ud1 = u[0, 0] + np.random.randn() * R[0, 0] ** 0.5
        ud2 = u[1, 0] + np.random.randn() * R[1, 1] ** 0.5
        ud = np.array([[ud1, ud2]]).T
        x = motion_model(x, ud) # (1)

        #  Calc Importance Weight
        for i in range(len(z[:, 0])):
            dx = x[0, 0] - z[i, 1]
            dy = x[1, 0] - z[i, 2]
            pre_z = math.hypot(dx, dy)
            dz = pre_z - z[i, 0]
            w = w * gauss_likelihood(dz, math.sqrt(Q[0, 0])) # (2)

        px[:, ip] = x[:, 0]
        pw[0, ip] = w
    pw = pw / pw.sum()  # normalize

    x_est = px.dot(pw.T) # (3)
    p_est = calc_covariance(x_est, px, pw)

    N_eff = 1.0 / (pw.dot(pw.T))[0, 0]  # Effective particle number
    if N_eff < NTh:
        px, pw = re_sampling(px, pw)
    return x_est, p_est, px, pw

def re_sampling(px, pw):
    """
    low variance re-sampling
    """

    w_cum = np.cumsum(pw)
    base = np.arange(0.0, 1.0, 1 / NP)
    r = np.random.uniform(0, 1 / NP)
    re_sample_id = r + base # (4)
    indexes = []
    ind = 0
    for ip in range(NP):
        while re_sample_id[ip] > w_cum[ind]: # (5)
            ind += 1
        indexes.append(ind)

    px = px[:, indexes] # (6)
    pw = np.zeros((1, NP)) + 1.0 / NP  # init weight

    return px, pw

turtlebot3_state = Turtlebot3_state()
def gps_cb(data):
    turtlebot3_state.gps = [data.latitude, -data.longitude, data.altitude]

def cmd_cb(data):
    turtlebot3_state.cmd_input = [data.linear.x, data.angular.z]

def imu_cb(data):
    turtlebot3_state.orientation = euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])

def landmark_pose_cb(data):
    for i in range(len(data.poses)):
        turtlebot3_state.landmarks[i] = [data.poses[i].position.x, data.poses[i].position.y]

def main():
    print(__file__ + " start!!")
    rospy.init_node('particle_filter', anonymous=True)
    pid = sys.argv[1]
    rospy.Subscriber(f"/{pid}/gps/values", NavSatFix, gps_cb)
    rospy.Subscriber(f"/{pid}/inertial_unit/roll_pitch_yaw", Imu, imu_cb)
    rospy.Subscriber("cmd_vel", Twist, cmd_cb)
    rospy.Subscriber("LM_pos", PoseArray, landmark_pose_cb)
    r = rospy.Rate(30)

    ros_pub = ros_publisher()
    while None in turtlebot3_state.cmd_input or None in turtlebot3_state.gps or None in turtlebot3_state.orientation:
        r.sleep()

    # State Vector [x y yaw v]
    x_est = np.zeros((4, 1))
    x_true = np.zeros((4, 1))
    px = np.zeros((4, NP))  # Particle initialization
    pw = np.zeros((1, NP)) + 1.0 / NP  # Particle weight
    x_dr = np.zeros((4, 1))  # Dead reckoning

    while not rospy.is_shutdown():
        u = calc_input()
        # landmark positions [x, y]
        LM_pos = np.array(turtlebot3_state.landmarks)

        x_true, z, x_dr, ud = observation(x_dr, u, LM_pos)

        x_est, PEst, px, pw = pf_localization(px, pw, z, ud)

        ros_pub.publish(x_true, px, x_dr, LM_pos, MAX_RANGE)
        r.sleep()
if __name__ == '__main__':
    main()