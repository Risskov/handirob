#!/usr/bin/env python3

import rospy
import numpy
from visualization_msgs.msg import MarkerArray
from object_msgs.msg import Objects, Object, PoseWithLabel
from geometry_msgs.msg import Pose
from stand_behavior import *
class people_behavior:
    def __init__(self):
        self.person_pos = Pose
        self.person_quat = Pose
        self.goals = GoalClient()
        self.base_pos = Pose
        self.callbackNo = False
        self.object_pose_sub = rospy.Subscriber("/people_projector/objects/transformed", Objects, self.callback)
        self.frameID = "camera_color_optical_frame"

    def callback(self, msg):
        #self.voltageMsg = msg.voltage
        self.person_pos = msg.objects[0].poses[0].pose
        #print(msg.markers[1].header.seq)
        print(self.person_pos)
        #rospy.loginfo("callback: %s", self.person_pos)
        if not self.callbackNo:
            self.callbackNo = True

    
    def calcfunc(self):
        while not self.callbackNo:
            rospy.sleep(0.1)
        self.base_pos = get_base_pose()
        self.person_pos.position.z = 0
        self.person_pos.orientation = self.base_pos.orientation
        self.person_quat = self.goals.calc_object_orientation(self.base_pos, self.person_pos)

    def gotoPerson(self):
        self.calcfunc()
        self.goals.update_goal(self.person_pos)
        self.goals.send_goal()
        #rospy.loginfo("goal sent to pose: %s", self.person_quat)

    def get_pose(self):
        return 1



def main():
    rospy.init_node("people_behavior")
    peoples = people_behavior() 
    r = rospy.Rate(1)
    
    while not rospy.is_shutdown():
        peoples.gotoPerson()
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
