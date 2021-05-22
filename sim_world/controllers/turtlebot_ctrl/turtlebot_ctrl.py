"""turtlebot_ctrl controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor, Lidar, LidarPoint
import rospy
from geometry_msgs.msg import Twist
import numpy as np
# create the Robot instance.
robot = Robot()


# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
right_wheel = robot.getMotor('right wheel motor')
left_wheel = robot.getMotor('left wheel motor')
right_wheel.setPosition(float("inf"))
left_wheel.setPosition(float("inf"))
# lidar
lidar = robot.getLidar("LDS-01")
lidar.enable(timestep)
lidar.enablePointCloud()
lidar_main_motor = robot.getMotor("LDS-01_main_motor")
lidar_secondary_motor = robot.getMotor("LDS-01_secondary_motor")
lidar_main_motor.setPosition(float("inf"))
lidar_secondary_motor.setPosition(float("inf"))
lidar_main_motor.setVelocity(30.0)
lidar_secondary_motor.setVelocity(60.0)



# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)


def cmd_to_wheel_vel(linear_v, ang_v):
    A = np.mat([[1/2, 1/2], [1/2, -1/2]])
    B = np.mat([linear_v, ang_v]).T
    r = np.linalg.solve(A, B)
 
    right_wheel.setVelocity(r[0])
    left_wheel.setVelocity(r[1])

def cmd_cb(data):
    cmd_to_wheel_vel(data.linear.x, data.angular.z)
    
rospy.init_node('turtlebot_controller', anonymous=True)
cmd_sub = rospy.Subscriber("cmd_vel", Twist, cmd_cb)


# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    # right_wheel.setVelocity(0.9)
    # left_wheel.setVelocity(1.1)
    # Process sensor data here.
    print("------------------------!!!!!!!!!!!!!-")
    for point in lidar.getPointCloud():
        print(point.x, point.y, point.z, point.time)
    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
