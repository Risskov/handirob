#!/usr/bin/env python
from __future__ import print_function
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
import tf2_ros as tf
#from tf2_ros.transformations import *
import numpy as np
import json
import lifting_mechanism
import handirob_docking.msg
import handirob_lifting.msg
# from set_goal import set_goal


def get_stand_data(stand_id, path):
    _path = path + '/stand_data/' + 'stand_' + str(stand_id) + '.json'
    if os.path.isfile(_path):
        f = open(_path,"r")
        f.flush()
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

def send_stand_goal(stand_number, stand_data):
    
    
    if stand_data:
        rospy.loginfo("Data for stand %i found. Setting goal.", stand_number)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        
        stand_pos = np.array([stand_data["pos"][0], stand_data["pos"][1], 0])
        
        #Getting base_footprint pose
        
        
        #t = base_tf.getLatestCommonTime("/base_footprint", "/map")
        base_position = tf_buffer.lookup_transform("map", "base_footprint", rospy.Time())
        base_pos = np.array([base_position.transform.translation.x, base_position.transform.translation.y, 0])
        print(base_pos)
        #Calculate direct orientation from base_footprint to stand
        q = calc_stand_orrientation(base_pos, stand_pos)

        pos_diff = (stand_pos - base_pos)
        stand_pos_approach = stand_pos - pos_diff/np.linalg.norm(pos_diff) # Setting goal 1 M away from the stand, in the immediate direction
        
        #Setting correct goal
        goal.target_pose.pose.position.x = stand_pos_approach[0]
        goal.target_pose.pose.position.y = stand_pos_approach[1]
        goal.target_pose.pose.position.z = stand_pos_approach[2]
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]
        
        goal_dist = np.linalg.norm(stand_pos - base_pos)
        
        # rospy.loginfo("sending goal at: %s", goal.target_pose.pose)  
        client.send_goal(goal)
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


def setup_lift_switch():

    GPIO_switch_pwr = 5
    
    GPIO.setmode(GPIO.BCM) # Broadcom pin-numbering scheme
    GPIO.setup(13, GPIO.IN)
    GPIO.setup(GPIO_switch_pwr, GPIO.OUT, initial=GPIO.HIGH)


def turn_to_lift2():

    #t = base_tf.getLatestCommonTime("/base_footprint", "/map")
    #base_tf.waitForTransform("/base_link", "/map", rospy.get_rostime(), rospy.Duration(10))
    base_position, base_quaternion = tf_buffer.lookup_transform("map", "base_footprint", rospy.Time())


    rot_goal = MoveBaseGoal()
    rot_goal.target_pose.header.frame_id = "map"
    rot_goal.target_pose.header.stamp = rospy.Time.now()

    rot_goal.target_pose.pose.position.x = base_position[0]
    rot_goal.target_pose.pose.position.y = base_position[1]
    rot_goal.target_pose.pose.position.z =  base_position[1]
    rot_goal.target_pose.pose.orientation.x = base_quaternion[0]
    rot_goal.target_pose.pose.orientation.y = base_quaternion[1]
    rot_goal.target_pose.pose.orientation.z = base_quaternion[3]
    rot_goal.target_pose.pose.orientation.w = base_quaternion[2]
    
    # rospy.loginfo("sending goal at: %s", goal.target_pose.pose)  
    client.send_goal(rot_goal)

    
def turn_to_lift():
    
        # Get position of robot base
    #base_tf = tf.TransformListener()
    #rospy.sleep(5)
    # t = base_tf.getLatestCommonTime("/base_footprint", "/map")
    #base_tf.waitForTransform("/base_link", "/map", rospy.get_rostime(), rospy.Duration(10))
    base_position = tf_buffer.lookup_transform("map", "base_footprint", rospy.Time())
    base_pos = np.array([base_position.transform.translation.x, base_position.transform.translation.y, 0])
    # base_position, base_quaternion = tf_buffer.lookup_transform("map", "base_footprint", rospy.Time())
    # base_pos = np.array([base_position[0], base_position[1], 0])
    
    stand_data = get_stand_data(stand_no, path)
    stand_pos = np.array([stand_data["pos"][0], stand_data["pos"][1], 0])
    
    stand_orr = calc_stand_orrientation(base_pos, stand_pos)
    
    cmd_data = Twist()
    cmd_data.linear.x = 0.0
    cmd_data.linear.y = 0.0
    cmd_data.linear.z = 0.0
    cmd_data.angular.x  = 0.0
    cmd_data.angular.y = 0.0
    

    
    # Start rotating robot
    angle_speed = 0.5
    diff = 5
    cmd_data.angular.z = angle_speed
    rospy.loginfo("Turning with 0.3 rad/s")

    goal_dist = np.linalg.norm(stand_pos - base_pos)


    for i in range (60):

        cmd_publisher.publish(cmd_data)
        rospy.sleep(0.1)

    angle_speed = 0
    cmd_data.angular.z = angle_speed
    rospy.loginfo("setting turning speed to: %f", angle_speed)
        
        
    #curr_speed = rospy.wait_for_message("cmd_vel", Twist, timeout=2)
    #if curr_speed.angular.z == 0:
        #get_switch_status(True)

        
        #while not get_switch_status(False):
        #    pass
        #rospy.sleep(4)

    return 1

def docking_client():
    # Creates the SimpleActionClient, passing the type of the action
    client = actionlib.SimpleActionClient('docking_server', handirob_docking.msg.DockingAction)

    # Waits until the action server has started up and started
    # listening for goals.
    print("waiting for server")
    client.wait_for_server()
    # Creates a goal to send to the action server.
    goal = handirob_docking.msg.DockingGoal(start=True)
    print("next")
    # Sends the goal to the action server.
    client.send_goal(goal)
    print("sent goal")
    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

def lifting_client():
    client = actionlib.SimpleActionClient('lifting_server', handirob_lifting.msg.LoweringAction)

    # Waits until the action server has started up and started
    # listening for goals.
    print("waiting for server")
    client.wait_for_server()
    # Creates a goal to send to the action server.
    goal = handirob_lifting.msg.LoweringGoal(start=True)
    print("next")
    # Sends the goal to the action server.
    client.send_goal(goal)
    print("sent goal")
    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult
class stand_search2:

    def __init__(self):
        self.stand_sub = rospy.Subscriber("/stand_pose_updated", Int16, self.stand_pose_cb)




    def stand_pose_cb(self, msg):

        while not msg.data:
             # Start turning and look for a stand
            cmd_data = Twist()
            cmd_data.linear.x = 0.0
            cmd_data.linear.y = 0.0
            cmd_data.linear.z = 0.0
            cmd_data.angular.x  = 0.0
            cmd_data.angular.y = 0.0
            cmd_data.angular.z = 0.4
            cmd_publisher.publish(cmd_data)
        
        
        cmd_data = Twist()
        cmd_data.linear.x = 0.0
        cmd_data.linear.y = 0.0
        cmd_data.linear.z = 0.0
        cmd_data.angular.x  = 0.0
        cmd_data.angular.y = 0.0
        cmd_data.angular.z = 0.0
        cmd_publisher.publish(cmd_data)
        self.msg = msg.data
        rospy.loginfo("Registered stand number: %s", self.msg)
        
    def get_stand(self):
        print(self.msg)
        return self.msg

def rotate_robot():
    global rotation_status
    cmd_data = Twist()
    cmd_data.linear.x = 0.0
    cmd_data.linear.y = 0.0
    cmd_data.linear.z = 0.0
    cmd_data.angular.x  = 0.0
    cmd_data.angular.y = 0.0
    rate = rospy.Rate(5)
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

        elif rotation_status == 2: # Fast turning
            
            cmd_data.angular.z = 0.23
            cmd_publisher.publish(cmd_data)
            rate.sleep()
        else:
            rospy.logwarn("No speed definition given")
            break
        

class get_fiducial_id:

    def __init__(self):
        self.fiducial_sub = rospy.Subscriber("/fiducial_verticies", FiducialArray, self.callback)

    def callback(self, msg):
        self.msg = msg

    def get_fiducial(self):
        return(self.fiducials.fiducial_id)

    

def stand_search():
    rospy.loginfo("Starting to explore")
        #stand_explore = stand_search()
    global rotation_status, stand_no
    rotation_status = 2 # turn fast
    rotation_thread = threading.Thread(target=rotate_robot)
    rotation_thread.start()
    #find_fiducial = get_fiducial_id()
    #while True:
    #    fiducial_id = find_fiducial.get_fiducial()
    #    print("Fiducial id: ", fiducial_id)
    #    if fiducial_id:
    #        stand_no = fiducial_id
    #        break
    while True:
        #rospy.loginfo("waiting for fiducial data")
        fiducial_data = rospy.wait_for_message("/fiducial_vertices", FiducialArray)
        #print(fiducial_data)
        #print(fiducial_data.fiducials)
        if len(fiducial_data.fiducials):
            
            stand_no = fiducial_data.fiducials[0].fiducial_id
            rotation_status = 1 # turn slow
            print("fiducial_id no = ", stand_no)
            #rotation_thread.join()
            break
    
    try:
        if stand_no:
            rospy.loginfo("found stand: %i", stand_no)
            stand_found = rospy.wait_for_message("stand_pose_updated", Int16)
            rotation_status = 0 # stop turning
            #rotation_thread.join()
            stand_data = get_stand_data(stand_no, path)
            return stand_data
    except:
        rospy.logerr("No stand to be found")
        return 0
    #if stand_explore.get_stand():
    #    print(stand_explore.get_stand)


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
    

    # lifting = lifting_mechanism.lifting_mechanism()

    # print(lifting.check_stand())
    # print(client.get_state())

    rospack = rospkg.RosPack()
    path = rospack.get_path('stand_pose')
    global stand_data
    
    stand_data = get_stand_data(stand_no, path)

    cmd_publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 10)

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("Waiting for Move Base server")
    client.wait_for_server()
    tf_buffer = tf.Buffer(rospy.Duration(1200.0))
    base_tf = tf.TransformListener(tf_buffer)
    rospy.sleep(1)

    if not stand_data:
        stand_data = stand_search()
        print("Sending goal to found stand")
    
    if stand_data:
        old_stand_data = stand_data
        
        goal_dist = send_stand_goal(stand_no, stand_data)

        print("goal dist: ", goal_dist)
        rospy.loginfo("Sent first goal to stand %i", stand_no)
            #while client.get_state() != 3:
        if goal_dist != 0:
            while True:
                stand_time = stand_data["stamp"]["secs"]

                stand_data = get_stand_data(stand_no, path)
                #rospy.Subscriber("stand_pose_updated", stand_pose_cb)
                rospy.loginfo("distance to goal is: %f", goal_dist)
                if stand_data != old_stand_data:
                    rospy.loginfo("New stand data found")
                    goal_dist = send_stand_goal(stand_no, stand_data)
                    rospy.loginfo("Sent new goal to stand %i", stand_no)
                old_stand_data = stand_data
                if goal_dist < 0.2 and rospy.get_time() - stand_time < 10 or client.get_state() == 3:
                    break
                        
            rospy.loginfo("distance to goal is: %f", goal_dist)
            rospy.loginfo("Cancelling goal")
            client.cancel_all_goals()
            #delete_dir()
            turn_to_lift()
            result = docking_client()
            stand_data = get_stand_data(108, path)
            send_stand_goal(108, stand_data)
            lifting_client()
            rospy.sleep(4)
            send_stand_goal(stand_no, stand_data)

            
rospy.spin()
            
    
