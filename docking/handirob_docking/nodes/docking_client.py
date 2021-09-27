#! /usr/bin/env python
from __future__ import print_function
import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import handirob_docking.msg

def docking_client():
    # Creates the SimpleActionClient, passing the type of the action
    client = actionlib.SimpleActionClient('docking_server', handirob_docking.msg.DockingAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()
    # Creates a goal to send to the action server.
    goal = handirob_docking.msg.DockingGoal(start=True)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('docking_client_py')
        result = docking_client()
        print("Result:", result.result)
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)

