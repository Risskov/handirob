#!/usr/bin/env python3

import rospy
import numpy
from visualization_msgs.msg import MarkerArray
from object_msgs.msg import Objects, Object, PoseWithLabel
from geometry_msgs.msg import Pose
from stand_behavior import *
class object_behavior:
    def __init__(self, _object_topic, _frameID):
        self.object_pos = Pose
        #self.object_quat = Pose
        self.goals = GoalClient()
        self.base_pos = Pose
        self.callbackNULL = False
        self.object_pose_sub = rospy.Subscriber("/" + _object_topic + "/objects/transformed", Objects, self.callback)
        if _object_topic == "stand":
            self.dock = True
        self.frameID = _frameID

    def callback(self, msg):
        self.object_pos = msg.objects[0].poses[0].pose
        if not self.callbackNULL:
            self.callbackNULL = True

    
    def calcfunc(self):
        while not self.callbackNULL:
            rospy.sleep(0.1)
        self.base_pos = get_base_pose()
        self.object_pos.position.z = 0
        self.object_pos.orientation = self.base_pos.orientation
        #self.object_quat = self.goals.calc_object_orientation(self.base_pos, self.object_pos)

    def gotoObject(self):
        self.calcfunc()
        self.goals.update_goal(self.object_pos)
        self.goals.send_goal()
        #rospy.loginfo("goal sent to pose: %s", self.object_quat)

    def get_pose(self):
        return 1



def main():
    Namespace = rospy.get_param("namespace", "object")
    object_topic = rospy.get_param("object_topic", "object")
    frameID = rospy.get_param("object_frame", "camera_color_optical_frame")
    rospy.init_node(Namespace + "_behavior")
    object = object_behavior(object_topic, frameID) 
    r = rospy.Rate(1)
    
    while not rospy.is_shutdown():
        object.gotoObject()
        r.sleep()

if __name__ == '__main__':
    main()
