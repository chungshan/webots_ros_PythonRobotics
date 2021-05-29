"""supervisor controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Supervisor
import rospy
from geometry_msgs.msg import PoseArray, Pose
# create the Robot instance.
supervisor = Supervisor()

# get the time step of the current world.
timestep = int(supervisor.getBasicTimeStep())
rfid_01 = supervisor.getFromDef('rfid_01')
rfid_02 = supervisor.getFromDef('rfid_02')
rfid_03 = supervisor.getFromDef('rfid_03')
rfid_04 = supervisor.getFromDef('rfid_04')
rfid_list = [rfid_01, rfid_02, rfid_03, rfid_04]
# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)
rospy.init_node('supervisor', anonymous=True)
rfid_pos_pub = rospy.Publisher("rfid_pos", PoseArray, queue_size=10)
r = rospy.Rate(300)
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while supervisor.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    rfid_pos = PoseArray()
    poses = []
    for i in range(len(rfid_list)):
        rfid_pose = Pose()
        rfid_position = rfid_list[i].getPosition()
        rfid_pose.position.x = rfid_position[0]
        rfid_pose.position.y = -rfid_position[2]
        rfid_pose.position.z = rfid_position[1]
        poses.append(rfid_pose)
    rfid_pos.poses = poses
    rfid_pos_pub.publish(rfid_pos)
    r.sleep()
    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
