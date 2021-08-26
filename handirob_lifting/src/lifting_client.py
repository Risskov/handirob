#! /usr/bin/env python3
import rospy
import actionlib
import RPi.GPIO as GPIO
from handirob_lifting.msg import *


if __name__ == '__main__':
    rospy.init_node('lifting_client')
    client = actionlib.SimpleActionClient('lift_stand', liftAction)
    client.wait_for_server()
    
    
    up = 1
    down = 0
    # Fill in the goal here
    goal = liftGoal(lift=1)
    client.send_goal(goal)
    feedback = rospy.wait_for_message("lift_stand/feedback", liftFeedback).feedback.lifting
    # client.wait_for_result(rospy.Duration.from_sec(30.0))
    print(feedback)
    
    rospy.sleep(5)
    print("stand is lifted. Will lower again")
    goal = liftGoal(lift=0)
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(30.0))
