#!/usr/bin/env python3 

import rospy
import rospkg
import actionlib
import sys
import os
import RPi.GPIO as GPIO
import threading
#from actionlib_msgs.msg import GoalStatusArray
#from autonomous_nav.srv import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Int16
from fiducial_msgs.msg import FiducialArray
from scipy.spatial.transform import Rotation as R
import tf
from tf.transformations import *
import numpy as np
import json
import lifting_mechanism
# from set_goal import set_goal


def get_stand_data(stand_id, path):
    _path = path + '/stand_data/' + 'stand_' + str(stand_id) + '.json'
    if os.path.isfile(_path):
        f = open(_path,"r")
        #f.flush()
        json_obj = json.load(f)
        f.close()
        # print(json_obj["pos"]["x"])
        return json_obj
    else:
        rospy.logwarn("nonexisiting stand was requested")
        return 0

def calc_stand_orrientation(_base_pos, _stand_pos):
    v2 = np.array([1, 0, 0])
    v1 = (_stand_pos - _base_pos)
    q = np.cross(v2, v1)
    
    q = np.append(q, np.sqrt(np.linalg.norm(v1)**2) + np.dot(v1, v2))
    q = q / np.linalg.norm(q)
    return q

def goal_data(pose, orientation):

    _goal = MoveBaseGoal()
    # Init header
    _goal.target_pose.header.frame_id = "map"
    _goal.target_pose.header.stamp = rospy.Time.now()
    # Init goal
    _goal.target_pose.pose.position.x = pose[0]
    _goal.target_pose.pose.position.y = pose[1]
    _goal.target_pose.pose.position.z =  pose[2]
    _goal.target_pose.pose.orientation.x = orientation[0]
    _goal.target_pose.pose.orientation.y = orientation[1]
    _goal.target_pose.pose.orientation.z = orientation[2]
    _goal.target_pose.pose.orientation.w = orientation[3] 

    return _goal



def send_stand_goal(stand_number, stand_data):
    
    
    if stand_data:
        rospy.loginfo("Data for stand %i found. Setting goal.", stand_number)

        stand_pos = np.array([stand_data["pos"][0], stand_data["pos"][1], 0])
        
        #Getting base_footprint pose
        
        
        t = base_tf.getLatestCommonTime("/base_footprint", "/map")
        base_position, base_quaternion = base_tf.lookupTransform("/map", "/base_footprint", t)
        base_pos = np.array([base_position[0], base_position[1], 0])
        
        #Calculate direct orientation from base_footprint to stand
        q = calc_stand_orrientation(base_pos, stand_pos)
        
        pos_diff = (stand_pos - base_pos)
        stand_pos_approach = stand_pos - pos_diff/np.linalg.norm(pos_diff) # Setting goal 1 M away from the stand, in the immediate direction
        
        #Setting correct goal
        approach_goal = goal_data(stand_pos_approach, q)
        
        goal_dist = numpy.linalg.norm(stand_pos - base_pos)
        
        # rospy.loginfo("sending goal at: %s", goal.target_pose.pose)  
        client.send_goal(approach_goal)
        rospy.sleep(1)
        state = client.get_state()
        rospy.loginfo(client.get_goal_status_text())
        if state == 1 or state == 3:
            rospy.loginfo("Goal to stand %i sent successfully", stand_no)
            return goal_dist
        else:
            rospy.logwarn("Goal did not process correctly, state: %i", state)
            return 0
            
        
    else:
        rospy.logerr("Stand number %i does not exist in world", stand_number)
        return 0

def turn_to_lift2():

    t = base_tf.getLatestCommonTime("/base_footprint", "/map")

    base_position, base_quaternion = base_tf.lookupTransform("/map", "/base_footprint", t)
    base_pos = np.array([base_position[0], base_position[1], 0])
    
    stand_data = get_stand_data(stand_no, path)
    stand_pos = np.array([stand_data["pos"][0], stand_data["pos"][1], 0])
    
    stand_orr = calc_stand_orrientation(base_pos, stand_pos)

    r = R.from_euler('z', 180, degrees=True)
    r2 = R.from_quat([stand_orr[0], stand_orr[1], stand_orr[2], stand_orr[3]])
    quat = r2 * r
    quat = quat.as_quat()

    base_quaternion = base_quaternion * r.as_quat()

    rot_goal = goal_data(base_position, quat)
    client.send_goal_and_wait(rot_goal)

    if client.get_state() == 3:
        cmd_data = Twist()
        cmd_data.linear.x = -0.1
        cmd_data.linear.y = 0.0
        cmd_data.linear.z = 0.0
        cmd_data.angular.x  = 0.0
        cmd_data.angular.y = 0.0
        cmd_data.angular.y = 0.0
        #lifting = lifting_mechanism.lifting_mechanism()
        pub_rate = rospy.Rate(10)
        while not lifting.check_stand():
            cmd_publisher.publish(cmd_data)
            pub_rate.sleep()

        cmd_data.linear.x = 0
        cmd_publisher.publish(cmd_data)




def rotate_robot():
    global rotation_status
    cmd_data = Twist()
    cmd_data.linear.x = 0.0
    cmd_data.linear.y = 0.0
    cmd_data.linear.z = 0.0
    cmd_data.angular.x  = 0.0
    cmd_data.angular.y = 0.0
    rate = rospy.Rate(5)
    timer = 0
    rospy.loginfo("Starting to turn robot")
    while True:
        if rotation_status == 0:
            cmd_data.angular.z = 0.0
            cmd_publisher.publish(cmd_data)
            #rospy.loginfo("Stopping rotation")
            break

        elif rotation_status == 1: # slow turning
            #rospy.loginfo("Slowing down rotation")
            cmd_data.angular.z = 0.08
            cmd_publisher.publish(cmd_data)
            rate.sleep()
            if timer > 30:
                rotation_status = 0
            timer += 1

        elif rotation_status == 2: # Fast turning
            
            cmd_data.angular.z = 0.15
            cmd_publisher.publish(cmd_data)
            rate.sleep()
        else:
            rospy.logwarn("No speed definition given")
            break
        
    

def stand_search():
    rospy.loginfo("Starting to explore")
        #stand_explore = stand_search()
    global rotation_status, stand_no
    rotation_status = 2 # turn fast
    rotation_thread = threading.Thread(target=rotate_robot)
    rotation_thread.start()
    while True:
        #rospy.loginfo("waiting for fiducial data")
        fiducial_data = rospy.wait_for_message("/fiducial_vertices", FiducialArray)
        if len(fiducial_data.fiducials):
            stand_no = int(fiducial_data.fiducials[0].fiducial_id)
            rotation_status = 1 # turn slow
            print("fiducial_id no = ", stand_no)
            break
    
    try:
        if stand_no:
            rospy.loginfo("found stand: %i", stand_no)
            stand_found = rospy.wait_for_message("stand_pose_updated", Int16)
            rotation_status = 0 # stop turning
            stand_data = get_stand_data(stand_no, path)
            return stand_data
    except:
        rospy.logerr("No stand to be found")
        return 0
    #if stand_explore.get_stand():
    #    print(stand_explore.get_stand)


def fetch_stand():
    global stand_data

    if not stand_data:
        stand_data = stand_search()
        print("Sending goal to found stand")
    
    
    rospy.loginfo("Stan_data: %s", stand_data)
    old_stand_data = stand_data
    
    goal_dist = send_stand_goal(stand_no, stand_data)

    print("goal dist: ", goal_dist)
    rospy.loginfo("Sent first goal to stand %i", stand_no)
    if goal_dist != 0:
        control_rate = rospy.Rate(10)
        while client.get_state() != 3:
            stand_time = stand_data["stamp"]["secs"]

            stand_data = get_stand_data(stand_no, path)
            if stand_data != old_stand_data:
                rospy.loginfo("New stand data found")
                goal_dist = send_stand_goal(stand_no, stand_data)
                rospy.loginfo("Sent new goal to stand %i", stand_no)
            old_stand_data = stand_data
            if goal_dist < 1 and rospy.get_time() - stand_time < 10:
                rospy.loginfo("Cancelling goal")
                client.cancel_all_goals()
                break
            
            control_rate.sleep()
    
    #rospy.loginfo("distance to goal is: %f", goal_dist)

    turn_to_lift2()


def move_forward(time = 4, direction = True):
    cmd_data = Twist()
    if direction:
        cmd_data.linear.x = 0.1
    else:
        cmd_data.linear.x = -0.1
    cmd_data.linear.y = 0.0
    cmd_data.linear.z = 0.0
    cmd_data.angular.x  = 0.0
    cmd_data.angular.y = 0.0
    cmd_data.angular.y = 0.0
    #lifting = lifting_mechanism.lifting_mechanism()
    pub_rate = rospy.Rate(10)
    for _ in range (time*10):
        cmd_publisher.publish(cmd_data)
        pub_rate.sleep()

    cmd_data.linear.x = 0
    cmd_publisher.publish(cmd_data)

def usage():
    return "%s [stand number]" % sys.argv[0]

def delete_dir():
    for element in dir():
        if element[0:2] != "__":
            del globals()[element]
    print(dir())
    
if __name__ == "__main__":
    
    argv = rospy.myargv()
    if len(sys.argv) == 2:
        global stand_no
        stand_no = int(sys.argv[1])
    else:
        print(usage())
        print(len(sys.argv))
        sys.exit(1)
    rospy.loginfo("Requesting stand pose")
    
    rospy.init_node("stand_goal_client")
    

    #lifting = lifting_mechanism.lifting_mechanism()

    
    rospack = rospkg.RosPack()
    global path
    path = rospack.get_path('stand_pose')
    global stand_data
    
    stand_data = get_stand_data(stand_no, path)

    
    cmd_publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 10)

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("Waiting for Move Base server")
    client.wait_for_server()
    lifting = lifting_mechanism.lifting_mechanism()
    base_tf = tf.TransformListener()
    rospy.sleep(1)


    t = base_tf.getLatestCommonTime("/base_footprint", "/map")
    init_base_position, init_base_quaternion = base_tf.lookupTransform("/map", "/base_footprint", t)
    
    fetch_stand()

    #go to init pose
    lifting.lift_stand()
    rospy.sleep(5)

    move_forward(4, False)
    lifting.lower_stand()
    rospy.sleep(5)
    move_forward(5)

    init_goal = goal_data(init_base_position, init_base_quaternion)
    client.send_goal_and_wait(init_goal)

    stand_data = None
    fetch_stand()

    lifting.lift_stand()
    rospy.sleep(5)

    move_forward(4, False)
    lifting.lower_stand()
    rospy.sleep(5)
    move_forward(5)

    init_goal = goal_data(init_base_position, init_base_quaternion)
    client.send_goal_and_wait(init_goal)



    

            
rospy.spin()
            
    
