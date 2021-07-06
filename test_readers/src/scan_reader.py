# !/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

def callback(data):
    rospy.loginfo(data.ranges)

def scan_reader():
    rospy.init_node('scan_reader', anonymous=True)
    rospy.Subscriber('scan', LaserScan, callback)
    rospy.spin()

if __name__ == '__main__':
    scan_reader()
