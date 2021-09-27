#!/usr/bin/python3
import time
import rospy
from object_msgs.msg import Objects, Object, PoseWithLabel
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import LaserScan
import laser_geometry.laser_geometry as lg
from ros_numpy import numpify, msgify
from scipy.cluster.hierarchy import fclusterdata
import numpy as np

class ModuleDetector:
    def __init__(self):
        self.sub = rospy.Subscriber('/scan_dock', LaserScan, callback=self.callback)
        self.pub_obj = rospy.Publisher('~objects', Objects, queue_size=1)
        self.pub_point = rospy.Publisher('~point', PointStamped, queue_size=1)
        self.lp = lg.LaserProjection()

    def callback(self, scan):
        cloud = self.lp.projectLaser(scan)
        cloud = numpify(cloud)
        cloud = np.asarray([cloud['x'], cloud['y']]).T
        clusters = fclusterdata(cloud, 0.05, criterion="distance")
        cloud = cloud[clusters == np.bincount(clusters).argmax()]

        point = PointStamped(header=scan.header)
        centroid = np.mean(cloud, axis=0)
        point.point.x = centroid[0]
        point.point.y = centroid[1]
        self.pub_point.publish(point)

        msg = Objects(header=scan.header)
        obj = Object(header=scan.header, label=100)
        pose = PoseWithLabel(label='Module')
        pose.pose.position = point.point
        obj.poses.append(pose)
        msg.objects.append(obj)
        self.pub_obj.publish(msg)

def main():
    rospy.init_node('ModuleDetector', anonymous=True, log_level=rospy.INFO)
    moduleDetector = ModuleDetector()
    rospy.spin()

if __name__ == '__main__':
    main()
