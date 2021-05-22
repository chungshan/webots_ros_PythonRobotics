import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf.transformations import *

class ros_publisher():
    def __init__(self):
        self.xTrue_pub = rospy.Publisher("xTrue", Path, queue_size=10)
        self.xDR_pub = rospy.Publisher("xDR", Path, queue_size=10)
        self.xEst_pub = rospy.Publisher("xEst", Path, queue_size=10)
        self.xTrue_path = Path()
        self.xDR_path = Path()
        self.xEst_path = Path()
        self.xTrue_list = []
        self.xEst_list = []
        self.xDR_list = []
    def publish(self, xTrue, xEst, xDR):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position.x = xEst[0]
        pose.pose.position.y = xEst[1]
        q = quaternion_from_euler(0, 0, xEst[2])
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        self.xEst_list.append(pose)
        self.xEst_path.header.stamp = rospy.Time.now()
        self.xEst_path.poses = self.xEst_list
        self.xEst_path.header.frame_id = "map"
        self.xEst_pub.publish(self.xEst_path)
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
