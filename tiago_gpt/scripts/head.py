#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def talker():
    head_cmd = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=1)
    rospy.init_node('head_lower', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        rospy.loginfo("Moving head down")
	jt = JointTrajectory()
	jt.joint_names = ['head_1_joint', 'head_2_joint']
	jtp = JointTrajectoryPoint()
	jtp.positions = [0.0, -0.75]
	jtp.time_from_start = rospy.Duration(2.0)
	jt.points.append(jtp)
	head_cmd.publish(jt)
        rospy.loginfo("Done.")
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
