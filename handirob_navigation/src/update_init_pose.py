#!/usr/bin/python3

import rospy
import sys
import os
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
import json
import rospkg


class nonVolatilePose():

    def __init__(self, _dir):
        rospy.init_node('set_init_pose', anonymous=True)
        self.pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
        self.sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)
        self.initpose_msg = PoseWithCovarianceStamped()
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



def main():
    rospack = rospkg.RosPack()
    directory = rospy.get_param("directory", rospack.get_path('handirob_navigation'))
    try:
        poseclass = nonVolatilePose(directory)
        rospy.spin()
    except KeyboardInterrupt:
        print( "Shutting down module")
    
    

if __name__ == '__main__':
    main()