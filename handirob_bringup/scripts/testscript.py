#!/usr/bin/env python3 
import rospy
from sensor_msgs.msg import BatteryState

def callback(data):
    rospy.loginfo("I heard %s",data)
    
def listener():
    rospy.init_node('node_name')
    rospy.Subscriber("chatter", BatteryState, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

listener()