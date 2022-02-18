#!/usr/bin/python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
import rospkg
import numpy as np


class nonVolatilePose():

    def __init__(self, _dir):
        rospy.init_node('init_pose_manager', anonymous=False)
        self.dir = _dir
        self.pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
        #self.load_position()
        rospy.sleep(2)
        self.load_position()
        self.sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)
        self.pose = PoseWithCovarianceStamped()

    def pose_callback(self, msg):
        self.pose = msg
        np.save(self.dir + "/pose_data/init_pose.npy", self.pose)

    def load_position(self):
        init_pose = np.load(self.dir +"/pose_data/init_pose.npy", allow_pickle=True).item()
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = "map"
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose = init_pose.pose
        self.pub.publish(pose_msg)
        rospy.loginfo("published initial pose at: \n %s", pose_msg)



def main():
    rospack = rospkg.RosPack()
    directory = rospy.get_param("~directory", rospack.get_path('handirob_navigation'))
    try:
        poseclass = nonVolatilePose(directory)
        rospy.spin()
    except KeyboardInterrupt:
        print( "Shutting down module")
    
    

if __name__ == '__main__':
    main()