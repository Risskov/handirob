#!/usr/bin/env python3
from os import ST_APPEND
import rospy

from geometry_msgs.msg import Pose
from stand_pose.msg import Stand

import json
import rospkg

####### Note: 31/01/2022
# Currently a copy of update_init_pose.py
# Needs solution specific code.
# Look into using a dict to keep position and such memory.. Or just update JSON / read JSON every time idc...
# Good luck. 



class StandManager():

    def __init__(self, _dir):
        #self.load_pose()
        self.dir = _dir
        self.pose = Pose
        self.save_pose()
        #self.pose_dict = {'pos' : list([0, 0, 0]), 'ori' : list([0, 0, 0, 0])}

    def pose_callback(self, msg):
        self.pose = msg.pose.pose

    def save_pose(self):
        file = open(self.dir + "/init_pose.json", "w")
        print(self.dir)
        dict = {'pos' : list([0, 0, 0]), 'ori' : list([0, 0, 0, 0])}
        file.write(json.dumps(dict))

    def load_pose(self):
        file = open("init_pose.json", "r")
        self.pose_dict = json.load(file)
        file.close()

    def dict_to_msg(self):
        self.initpose_msg.pose.position.x = self.pose_dict["pos"][0]
        self.initpose_msg.pose.position.y = self.pose_dict["pos"][1]
        self.initpose_msg.pose.position.z = self.pose_dict["pos"][2]

        self.initpose_msg.pose.orientation.x = self.pose_dict["ori"][0]
        self.initpose_msg.pose.orientation.y = self.pose_dict["ori"][1]
        self.initpose_msg.pose.orientation.z = self.pose_dict["ori"][2]
        self.initpose_msg.pose.orientation.w = self.pose_dict["ori"][3]


    def msg_to_dict(self):
        self.pose_dict = {'pos' : list([self.initpose_msg.pose.position.x, self.initpose_msg.pose.position.y, self.initpose_msg.pose.position.z]), 
        'ori' : list([self.initpose_msg.pose.orientation.x, self.initpose_msg.pose.orientation.y, self.initpose_msg.pose.orientation.z, self.initpose_msg.pose.orientation.w])}    

    def set_pose(self):
        self.initpose_msg.header.frame_id = "map"

if __name__ == '__main__':
    rospy.init_node('stand_manager')
    StandManager()
    rospy.spin()