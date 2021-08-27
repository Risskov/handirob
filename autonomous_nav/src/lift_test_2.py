#! /usr/bin/env python3
import rospy
import lifting_mechanism


rospy.init_node("Lifting_test_node")
lifting = lifting_mechanism.lifting_mechanism()

while True:
    print(lifting.check_stand())
    
    rospy.sleep(1)

rospy.spin()