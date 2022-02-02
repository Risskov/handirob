#!/usr/bin/env python3 

import rospy
from sensor_msgs.msg import BatteryState
import os
import subprocess


class emgBtn:
    def __init__(self):
        self.voltageMsg = 100
        self.battery_sub = rospy.Subscriber("battery_state", BatteryState, self.callback)

    def callback(self, msg):
        self.voltageMsg = msg.voltage
        #print(self.msg)

    def checkVoltage(self):
        if self.voltageMsg < 0.5



def main():
    rospy.init_node("Emergerncy_button_handler")
    emergencyButton = emgBtn()
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        emergencyButton.checkVoltage()
        r.sleep()

# def callback(data):
#     rospy.loginfo(data.voltage)


# def main():
#     rospy.init_node("Emergerncy_button_handler")
#     rospy.Subscriber("battery_state", BatteryState, callback)
#     #voltage = rospy.wait_for_message("battery_state", BatteryState, 10)
#     rospy.spin()

if __name__ == '__main__':
    main()
