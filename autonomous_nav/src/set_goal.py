#!/usr/bin/python3

import rospy
import actionlib
import sys
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped

# transform = (double(sys.argv[1]), double(sys.argv[2]), double(sys.argv[3]), double(sys.argv[4]), double(sys.argv[5]),
#             double(sys.argv[6]), double(sys.argv[7]))

# Callbacks definition



def active_cb(extra):
    rospy.loginfo("Goal pose being processed")


def feedback_cb(feedback):
    rospy.loginfo("Current location: " + str(feedback))


def done_cb(status, result):
    if status == 3:
        rospy.loginfo("Goal reached")
    if status == 2 or status == 8:
        rospy.loginfo("Goal cancelled")
    if status == 4:
        rospy.loginfo("Goal aborted")


def set_goal():
    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.header.stamp = rospy.Time.now()

    goal.pose.position.x = pos_x
    goal.pose.position.y = pos_y
    goal.pose.position.z = pos_z
    goal.pose.orientation.x = rot_x
    goal.pose.orientation.y = rot_y
    goal.pose.orientation.z = rot_z
    goal.pose.orientation.w = rot_w

    rospy.sleep(1)
    rospy.loginfo("Sending goal to navigation stack.")
    pub.publish(goal)
    rospy.loginfo("Goal sent to navigation stack.")


def goal_handler:
    rospy.init_node("set_goal", anonymous=True)
    pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
    pos_x = rospy.get_param('/pos_x', -5.579485893249512)
    pos_y = rospy.get_param('/pos_y', 3.488150119781494)
    pos_z = rospy.get_param('/pos_z', 0.0)
    rot_x = rospy.get_param('/rot_x', 0.0)
    rot_y = rospy.get_param('/rot_y', 0.0)
    rot_z = rospy.get_param('/rot_z', 0.9751426556251712)
    rot_w = rospy.get_param('/rot_w', 0.22157797990840336)

    rospy.spin()
# navclient.send_goal(goal, done_cb, active_cb, feedback_cb)
# finished = navclient.wait_for_result()
