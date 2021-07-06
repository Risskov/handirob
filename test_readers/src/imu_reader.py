# !/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
#from nav_msgs.msg import Odometry

def callback(data):
    rospy.loginfo(data)

def scan_reader():
    rospy.init_node('imu_reader', anonymous=True)
    rospy.Subscriber('imu', Imu, callback)
    #rospy.Subscriber('odom', Odometry, callback)
    rospy.spin()

if __name__ == '__main__':
    scan_reader()
