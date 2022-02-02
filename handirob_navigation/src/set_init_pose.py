#!/usr/bin/python3

import rospy
import sys
from geometry_msgs.msg import PoseWithCovarianceStamped

rospy.init_node('set_init_pose', anonymous=True)
pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)

initpose_msg = PoseWithCovarianceStamped()
initpose_msg.header.frame_id = "map"
initpose_msg.pose.pose.position.x = -2.872450351715088
initpose_msg.pose.pose.position.y = 0.5851105451583862
initpose_msg.pose.pose.orientation.x = 0.0
initpose_msg.pose.pose.orientation.y = 0.0
initpose_msg.pose.pose.orientation.z = 0.9595291103491465
initpose_msg.pose.pose.orientation.w = 0.2816094572143761

rospy.sleep(1)

rospy.loginfo ( "Setting initial pose: ")
pub.publish(initpose_msg)
rospy.loginfo ( "Initial pose SET")

