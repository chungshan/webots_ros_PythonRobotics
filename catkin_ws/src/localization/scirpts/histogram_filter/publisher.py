import numpy as np
import rospy
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import PoseStamped, Point32, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from tf.transformations import *

class ros_publisher():
    def __init__(self):
        self.xTrue_pub = rospy.Publisher("xTrue", Path, queue_size=10)
        self.xDR_pub = rospy.Publisher("xDR", Path, queue_size=10)
        self.xEst_pub = rospy.Publisher("xEst", PointCloud, queue_size=10)
        self.rfid_marker = rospy.Publisher("visualization_marker_rfid", Marker, queue_size=10)
        self.line_marker = rospy.Publisher("visualization_marker_line", Marker, queue_size=10)
        self.xTrue_path = Path()
        self.xDR_path = Path()
        self.xEst_path = Path()
        self.xTrue_list = []

        self.xDR_list = []
    def publish(self, xTrue, xEst, xDR, rfid_pose, max_range):
        grid = PointCloud()
        grid.header.stamp = rospy.Time.now()
        grid.header.frame_id = "map"
        grid_list = []
        for x in xEst:
            point_ = Point32()
            point_.x = x[0]
            point_.y = x[1]
            point_.z = 0
            grid_list.append(point_)
        grid.points = grid_list
        self.xEst_pub.publish(grid)
        # trajectory
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position.x = xTrue[0]
        pose.pose.position.y = xTrue[1]
        q = quaternion_from_euler(0, 0, xTrue[2])
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        self.xTrue_list.append(pose)
        self.xTrue_path.header.stamp = rospy.Time.now()
        self.xTrue_path.poses = self.xTrue_list
        self.xTrue_path.header.frame_id = "map"
        self.xTrue_pub.publish(self.xTrue_path)
        # trajectory
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position.x = xDR[0]
        pose.pose.position.y = xDR[1]
        # q = quaternion_from_euler(0, 0, xDR[2])
        # pose.pose.orientation.x = q[0]
        # pose.pose.orientation.y = q[1]
        # pose.pose.orientation.z = q[2]
        # pose.pose.orientation.w = q[3]

        self.xDR_list.append(pose)
        self.xDR_path.header.stamp = rospy.Time.now()
        self.xDR_path.poses = self.xDR_list
        self.xDR_path.header.frame_id = "map"
        self.xDR_pub.publish(self.xDR_path)


        # rfid marker
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = "map"
        marker.type = 8
        for pos in rfid_pose:
            rfid_pos = Point()
            rfid_pos.x = pos[0]
            rfid_pos.y = pos[1]
            marker.points.append(rfid_pos)
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1

            marker.color.r = 1
            marker.color.g = 1
            marker.color.b = 0
            marker.color.a = 1
        self.rfid_marker.publish(marker)

        # rfid lines
        line_list = Marker()
        line_list.header.stamp = rospy.Time.now()
        line_list.header.frame_id = "map"
        line_list.type = 5
        for pos in rfid_pose:
            if np.sqrt(np.sum((xTrue[0:2].T[0] - pos)**2)) < max_range:
                robot_pos = Point()
                robot_pos.x = xTrue[0]
                robot_pos.y = xTrue[1]
                line_list.points.append(robot_pos)

                rfid_pos = Point()
                rfid_pos.x = pos[0]
                rfid_pos.y = pos[1]
                line_list.points.append(rfid_pos)
                line_list.scale.x = 0.01

                line_list.color.r = 75/255
                line_list.color.g = 70/255
                line_list.color.b = 1
                line_list.color.a = 1
        self.line_marker.publish(line_list)