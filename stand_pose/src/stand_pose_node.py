#!/usr/bin/env python3
import rospy
import rospkg

# Because of transformations
import tf2_ros
import tf_conversions
import tf2_msgs.msg
import tf
import sys
import numpy as np
from scipy.spatial.transform import Rotation as R
import tf2_geometry_msgs
import geometry_msgs.msg
from std_msgs.msg import Int16
import json

import fiducial_msgs.msg as aruco

offset = np.array([0, 0, -0.1])
rate = 1 # in Hz  - Non static rate. The fastest rate allowed.

def get_stand_data(stand_id, path):
    f = open(path + '/stand_data/' + 'stand_' + str(stand_id) + '.json',"r")
    json_obj = json.load(f)
    f.close()
    print(json_obj["pos"]["x"])
    return json_obj

    

def calc_stand(data, path):

        for marker in data.transforms:

            #   Fiducial stuff
            id = marker.fiducial_id
            translation = marker.transform.translation
            t = np.array([translation.x, translation.y, translation.z])
            rotation = marker.transform.rotation
            #print("Stand with id:", trans.fiducial_id, "is found at a distance of about", np.sqrt(trans.transform.translation.z**2 + trans.transform.translation.x**2 + trans.transform.translation.y**2), "m")
            r = R.from_quat([rotation.x, rotation.y, rotation.z, rotation.w])
            stand = t + r.apply(offset)
            #print(stand)

            #   TF stuff
            pose_stamped = tf2_geometry_msgs.PoseStamped()
            pose_stamped.pose.position.x = stand[0]
            pose_stamped.pose.position.y = stand[1]
            pose_stamped.pose.position.z = stand[2]
            pose_stamped.pose.orientation = rotation
            pose_stamped.header.frame_id = data.header.frame_id
            pose_stamped.header.stamp = data.header.stamp
            pose_stamped.header.seq = data.header.seq

            #print(pose_stamped)
            try:
                output_pose_stamped = tf_buffer.transform(pose_stamped, "map", rospy.Duration(1))

                # python dictionary with key value pairs
                dict = {'pos' : list([output_pose_stamped.pose.position.x, output_pose_stamped.pose.position.y, output_pose_stamped.pose.position.z]), 'stamp' : {"secs" : data.header.stamp.secs, "nsecs" : data.header.stamp.nsecs} }
                rospy.loginfo("dict: %s", dict)

                # create json object from dictionary
                json_obj = json.dumps(dict)

                # Write to json file
                f = open(path + '/stand_data/' + 'stand_' + str(id) + '.json',"w")
                f.write(json_obj)
                f.close()
                pub_pose.publish(id)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                raise

            #try:
                # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
                #output_pose_stamped = tf_buffer.transform(pose_stamped, "camera_link", rospy.Duration(0.1))
                # Her skal den så sende posen lel
                #print(listener.lookupTransform("/fiducial_102", "/camera_link", rospy.Time(0)))

                #transform_stamped = geometry_msgs.msg.TransformStamped()
                #transform_stamped.header = output_pose_stamped.header
                #transform_stamped.transform.translation = output_pose_stamped.pose.position
                #transform_stamped.transform.rotation = output_pose_stamped.pose.orientation
                #transform_stamped.child_frame_id = "stand_" + str(id)
                #
                #tfm = tf2_msgs.msg.TFMessage([transform_stamped])

                #pub_tf.publish(tfm)

            #except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                #raise


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

    rospy.init_node('stand_pose', anonymous=False)
    global listener, tf_buffer, pub_pose
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    pub_pose = rospy.Publisher("/stand_pose_updated", Int16, queue_size=1)
    rospy.sleep(2)



    # get an instance of RosPack with the default search paths
    rospack = rospkg.RosPack()

    # get the file path for rospy_tutorials
    path = rospack.get_path('stand_pose')

    print(path)

    while not rospy.is_shutdown():
        # Every 1/hertz seconds this code should get latest /fiducial_transform, compare time to assure -
        # it is a new message, then update the respective json file in a predetermined folder as previous callback.
        data = rospy.wait_for_message("/fiducial_transforms", aruco.FiducialTransformArray) # Waits a little less than what the rate allows before timeout.
        calc_stand(data, path)

        rospy.sleep(1/rate)


    #rospy.Subscriber("/fiducial_transforms", aruco.FiducialTransformArray , callback, queue_size=1)
    #print("Subscriber created")

if __name__ == '__main__':
    listener()
