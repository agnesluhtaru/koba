#! /usr/bin/env python

import rospy
from std_msgs.msg import Int16
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose

class Intermediate:
    def __init__(self):
        rospy.init_node('intermediate', anonymous=True)
        self.pub_goal = rospy.Publisher('aruco_single/pose', PoseStamped, queue_size=1)
        self.pub_obstacle = rospy.Publisher('aruco_obstacle/pose', PoseStamped, queue_size=1)
        rospy.Subscriber("recognized", Int16, self.recognizing_marker)
        self.marker_ids = [5, 26, 99, 182, 253]
        rospy.Subscriber("aruco_single_5/pose", PoseStamped, self.pose_5)
        rospy.Subscriber("aruco_single_26/pose", PoseStamped, self.pose_26)
        rospy.Subscriber("aruco_single_99/pose", PoseStamped, self.pose_99)
        rospy.Subscriber("aruco_single_182/pose", PoseStamped, self.pose_182)
        rospy.Subscriber("aruco_single_253/pose", PoseStamped, self.pose_253)
        self.recognized = None
        self.added_5 = False
        self.added_26 = False
        self.added_99 = False
        self.added_182 = False
        self.added_253 = False

    def recognizing_marker(self, data):
        self.recognized = int(data.data)
    
    def pose_5(self, data):
        marker_pose = data
        if self.recognized == 5:
            self.pub_goal.publish(marker_pose)
        elif self.recognized and not self.added_5:
            self.pub_obstacle.publish(marker_pose)
            self.added_5 = True
            rospy.loginfo("added obstacle 5")
    
    def pose_26(self, data):
        marker_pose = data
        if self.recognized == 26:
            self.pub_goal.publish(marker_pose)
        elif self.recognized and not self.added_26:
            self.pub_obstacle.publish(marker_pose)
            self.added_26 = True
            rospy.loginfo("added obstacle 26")
    
    def pose_99(self, data):
        marker_pose = data
        if self.recognized == 99:
            self.pub_goal.publish(marker_pose)
        elif self.recognized and not self.added_99:
            self.pub_obstacle.publish(marker_pose)
            self.added_99 = True
            rospy.loginfo("added obstacle 99")
    
    def pose_182(self, data):
        marker_pose = data
        if self.recognized == 182:
            self.pub_goal.publish(marker_pose)
        elif self.recognized and not self.added_182:
            self.pub_obstacle.publish(marker_pose)
            self.added_182 = True
            rospy.loginfo("added obstacle 182")
    
    def pose_253(self, data):
        marker_pose = data
        if self.recognized == 253:
            self.pub_goal.publish(marker_pose)
        elif self.recognized and not self.added_253:
            self.pub_obstacle.publish(marker_pose)
            self.added_253 = True
            rospy.loginfo("added obstacle 253")

if __name__ == '__main__':
    try:
        Intermediate()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

