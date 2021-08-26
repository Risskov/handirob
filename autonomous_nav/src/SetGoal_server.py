#!/usr/bin/python3

NAME = 'set_goal_server'

import rospy
from autonomous_nav.srv import *
from geometry_msgs.msg import PoseStamped


def set_goal_handler(req):
    # rospy.loginfo("Setting goal pose: %f, %f, %f, %f, %f, %f, %f",
    # req.px, req.py, req.pz, req.rx, req.ry, req.rz, req.rw)
    pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.header.stamp = rospy.Time.now()
    goal.pose.position.x = req.px
    goal.pose.position.y = req.py
    goal.pose.position.z = req.pz
    goal.pose.orientation.x = req.rx
    goal.pose.orientation.y = req.ry
    goal.pose.orientation.z = req.rz
    goal.pose.orientation.w = req.rw

    rospy.sleep(0.5)
    rospy.loginfo("Sending goal to navigation stack at pose: \n%s .", goal.pose)
    pub.publish(goal)
    rospy.loginfo("Goal sent to navigation stack.")
    return SetGoalResponse(1)


# def get_stand_tf(stand_num):


def set_goal_server():
    rospy.init_node(NAME)
    print("starting set_goal service")
    s1 = rospy.Service('set_goal', SetGoal, set_goal_handler)

    rospy.spin()


if __name__ == "__main__":
    set_goal_server()
