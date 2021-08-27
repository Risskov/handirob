#! /usr/bin/env python3
import rospy
import lifting_mechanism


rospy.init_node("Lifting_test_node")
lifting = lifting_mechanism.lifting_mechanism()

print(lifting.check_stand())