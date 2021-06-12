#!/usr/bin/env python3
import math
import copy

import numpy as np
import rospy
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Twist, PoseStamped
from publisher import ros_publisher
from tf.transformations import *
import sys
from scipy.ndimage import gaussian_filter
from scipy.stats import norm
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseArray, Pose
# Parameters
EXTEND_AREA = 10.0  # [m] grid map extended length
DT = 0.36  # time tick [s]
MAX_RANGE = 5.0  # maximum observation range
MOTION_STD = 1.0  # standard deviation for motion gaussian distribution
RANGE_STD = 3.0  # standard deviation for observation gaussian distribution

# grid map param
XY_RESOLUTION = 0.5  # xy grid resolution
MIN_X = -15.0
MIN_Y = -5.0
MAX_X = 15.0
MAX_Y = 25.0

# simulation parameters
NOISE_RANGE = 0.2  # [m] 1σ range noise parameter
NOISE_SPEED = 0.1  # [m/s] 1σ speed noise parameter


class Turtlebot3_state:
    def __init__(self):
        self.cmd_input = [None, None]  # cmd input: v, yaw_rate
        self.gps = [None, None, None]  # gps of TurtleBot3
        self.orientation = [None, None, None]  # pitch, roll, yaw
        self.landmarks = [None] * 4  # 4 landmark's position

class GridMap:

    def __init__(self):
        self.data = None
        self.xy_resolution = None
        self.min_x = None
        self.min_y = None
        self.max_x = None
        self.max_y = None
        self.x_w = None
        self.y_w = None
        self.dx = 0.0  # movement distance
        self.dy = 0.0  # movement distance

def histogram_filter_localization(grid_map, u, z, yaw):
    grid_map = motion_update(grid_map, u, yaw) # (1)
    grid_map = observation_update(grid_map, z, RANGE_STD) # (2)

    return grid_map

def calc_gaussian_observation_pdf(grid_map, z, iz, ix, iy, std):
    # predicted range
    x = ix * grid_map.xy_resolution + grid_map.min_x
    y = iy * grid_map.xy_resolution + grid_map.min_y
    d = math.hypot(x - z[iz, 1], y - z[iz, 2])

    # likelihood
    pdf = (1.0 - norm.cdf(abs(d - z[iz, 0]), 0.0, std))

    return pdf


def observation_update(grid_map, z, std):
    for iz in range(z.shape[0]):
        for ix in range(grid_map.x_w):
            for iy in range(grid_map.y_w):
                grid_map.data[ix][iy] *= calc_gaussian_observation_pdf(grid_map, z, iz, ix, iy, std)

    grid_map = normalize_probability(grid_map)

    return grid_map

def calc_input():
    v = turtlebot3_state.cmd_input[0]  # [m/s]
    yawrate = turtlebot3_state.cmd_input[1]  # [rad/s]
    u = np.array([[v, yawrate]]).T
    return u


def observation(u, LM_pos, xd):
    xTrue = np.array([[turtlebot3_state.gps[0]], [turtlebot3_state.gps[1]], [turtlebot3_state.orientation[2]], [u[0][0]]]) # x, y, yaw, v

    z = np.zeros((0, 3))

    for i in range(len(LM_pos[:, 0])):

        dx = xTrue[0, 0] - LM_pos[i, 0]
        dy = xTrue[1, 0] - LM_pos[i, 1]
        d = math.hypot(dx, dy)
        if d <= MAX_RANGE:
            # add noise to range observation
            dn = d + np.random.randn() * NOISE_RANGE
            zi = np.array([dn, LM_pos[i, 0], LM_pos[i, 1]])
            z = np.vstack((z, zi))

    # add noise to speed
    ud = u[:, :]
    ud[0] += np.random.randn() * NOISE_SPEED
    xd = motion_model(xd, ud)
    return xTrue, z, ud, xd

def normalize_probability(grid_map):
    sump = sum([sum(i_data) for i_data in grid_map.data])

    for ix in range(grid_map.x_w):
        for iy in range(grid_map.y_w):
            grid_map.data[ix][iy] /= sump

    return grid_map

def init_grid_map(xy_resolution, min_x, min_y, max_x, max_y):
    grid_map = GridMap()

    grid_map.xy_resolution = xy_resolution
    grid_map.min_x = min_x
    grid_map.min_y = min_y
    grid_map.max_x = max_x
    grid_map.max_y = max_y
    grid_map.x_w = int(round((grid_map.max_x - grid_map.min_x)
                             / grid_map.xy_resolution))
    grid_map.y_w = int(round((grid_map.max_y - grid_map.min_y)
                             / grid_map.xy_resolution))

    grid_map.data = [[1.0 for _ in range(grid_map.y_w)]
                     for _ in range(grid_map.x_w)]
    grid_map = normalize_probability(grid_map)

    return grid_map

def map_shift(grid_map, x_shift, y_shift):
    tmp_grid_map = copy.deepcopy(grid_map.data)

    for ix in range(grid_map.x_w):
        for iy in range(grid_map.y_w):
            nix = ix + x_shift
            niy = iy + y_shift

            if 0 <= nix < grid_map.x_w and 0 <= niy < grid_map.y_w:
                grid_map.data[ix + x_shift][iy + y_shift] =\
                    tmp_grid_map[ix][iy]
    return grid_map

def motion_update(grid_map, u, yaw):
    grid_map.dx += DT * math.cos(yaw) * u[0]
    grid_map.dy += DT * math.sin(yaw) * u[0]

    x_shift = grid_map.dx // grid_map.xy_resolution
    y_shift = grid_map.dy // grid_map.xy_resolution

    if abs(x_shift) >= 1.0 or abs(y_shift) >= 1.0:  # map should be shifted
        grid_map = map_shift(grid_map, int(x_shift), int(y_shift))
        grid_map.dx -= x_shift * grid_map.xy_resolution
        grid_map.dy -= y_shift * grid_map.xy_resolution

    grid_map.data = gaussian_filter(grid_map.data, sigma=MOTION_STD)

    return grid_map

def calc_grid_index(grid_map):
    mx, my = np.mgrid[slice(grid_map.min_x - grid_map.xy_resolution / 2.0,
                            grid_map.max_x + grid_map.xy_resolution / 2.0,
                            grid_map.xy_resolution),
                      slice(grid_map.min_y - grid_map.xy_resolution / 2.0,
                            grid_map.max_y + grid_map.xy_resolution / 2.0,
                            grid_map.xy_resolution)]

    return mx, my

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

def convert_to_pos(max_grid, grid_map):

    x = max_grid[0] * XY_RESOLUTION + grid_map.min_x
    y = max_grid[1] * XY_RESOLUTION + grid_map.min_y
    return [x, y]


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
    rospy.init_node('histogram_filter', anonymous=True)
    pid = sys.argv[1]

    rospy.Subscriber(f"/{pid}/gps/values", NavSatFix, gps_cb)
    rospy.Subscriber(f"/{pid}/inertial_unit/roll_pitch_yaw", Imu, imu_cb)
    rospy.Subscriber("cmd_vel", Twist, cmd_cb)
    rospy.Subscriber("LM_pos", PoseArray, landmark_pose_cb)
    ros_pub = ros_publisher()
    r = rospy.Rate(30)

    # wait for state
    while None in turtlebot3_state.cmd_input or None in turtlebot3_state.gps or None in turtlebot3_state.orientation:
        r.sleep()

    x_dr = np.zeros((4, 1))  # Dead reckoning
    xTrue = np.zeros((4, 1))
    grid_map = init_grid_map(XY_RESOLUTION, MIN_X, MIN_Y, MAX_X, MAX_Y)

    while not rospy.is_shutdown():
        u = calc_input()

        # landmark positions [x, y]
        LM_pos = np.array(turtlebot3_state.landmarks)

        yaw = xTrue[2, 0]  # Orientation is known
        xTrue, z, ud, x_dr = observation(u, LM_pos, x_dr)
        grid_map = histogram_filter_localization(grid_map, u, z, yaw)
        inds = grid_map.data.flatten().argsort()[-10:]
        x_est = []
        for ind in inds:
            max_grid = np.unravel_index(ind, grid_map.data.shape)
            x_est.append(convert_to_pos(max_grid, grid_map))
        ros_pub.publish(xTrue, x_est, x_dr, LM_pos, MAX_RANGE)
        r.sleep()

if __name__ == '__main__':
    main()