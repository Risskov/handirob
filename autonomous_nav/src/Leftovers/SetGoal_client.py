#!/usr/bin/env python3

import rospy
from autonomous_nav.srv import *
from geometry_msgs.msg import PoseStamped
import sys


def set_goal_client(px, py, pz, rx, ry, rz, rw):
    rospy.wait_for_service('set_goal')

    # set_goal(px, py, pz, rx, ry, rz, rw)
    try:
        print("setting goal")
        set_goal = rospy.ServiceProxy('set_goal', SetGoal)
        resp1 = set_goal.call(SetGoalRequest(px, py, pz, rx, ry, rz, rw))
        # print(resp1.response)
        return resp1.response
    except rospy.ServiceException as exc:
        rospy.logwarn("Service did not process request: %s", str(exc))

#    except rospy.ServiceException as e:
#        print("Service call failed: %s"%e)

def usage():
    return "%s [pose x, pose y, pose z, orr x, orr y, orr z, orr w]" % sys.argv[0]


if __name__ == "__main__":

    argv = rospy.myargv()
    if len(sys.argv) == 8:
        px = float(sys.argv[1])
        py = float(sys.argv[2])
        pz = float(sys.argv[3])
        rx = float(sys.argv[4])
        ry = float(sys.argv[5])
        rz = float(sys.argv[6])
        rw = float(sys.argv[7])
    else:
        print(usage())
        sys.exit(1)
    print("Requesting poses")
    set_goal_client(px, py, pz, rx, ry, rz, rw)
    #print("Requesting %s,%s,%s,%s,%s,%s,%s,%s" % (
     #   px, py, pz, rx, ry, rz, rw, set_goal_client(px, py, pz, rx, ry, rz, rw)))