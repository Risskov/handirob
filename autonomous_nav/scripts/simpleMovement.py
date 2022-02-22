#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from rospy.rostime import Duration
import random





def swivel(speed, sleepTime):
    twist = Twist()
    pub_rate = rospy.Rate(10)
    twist.linear.x = speed
    randtime = 0.4 + (random.random() * (2 - 0.4))
    for _ in range (sleepTime*9):
        cmd_publisher.publish(twist)
        pub_rate.sleep()
    rospy.sleep(Duration(randtime))
    twist.linear.x = -speed

    for _ in range (sleepTime*10):
        cmd_publisher.publish(twist)
        pub_rate.sleep()
    randtime = 0.4 + (random.random() * (2 - 0.4))
    rospy.sleep(Duration(randtime))





rospy.init_node('SimpleTurning')
cmd_publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
turning = rospy.Publisher("cmd_vel", Twist, queue_size = 10)

while not rospy.is_shutdown():
        swivel(0.1, 1)


