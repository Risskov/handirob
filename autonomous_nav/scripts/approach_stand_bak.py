#!/usr/bin/env python3 

import rospy
import rospkg
import actionlib
import sys
import os
import RPi.GPIO as GPIO
#from actionlib_msgs.msg import GoalStatusArray
#from autonomous_nav.srv import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Int16
import tf
from tf.transformations import *
import numpy as np
import json
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
    
def feedbackCb():
    rospy.loginfo(MoveBaseFeedback)

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
        
        
        t = base_tf.getLatestCommonTime("/base_footprint", "/map")
        base_position, base_quaternion = base_tf.lookupTransform("/map", "/base_footprint", t)
        base_pos = np.array([base_position[0], base_position[1], 0])
        
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
        
        goal_dist = numpy.linalg.norm(stand_pos - base_pos)
        
        rospy.loginfo("sending goal at: %s", goal.target_pose.pose)  
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



def turn_to_lift():
    
        # Get position of robot base
    #base_tf = tf.TransformListener()
    #rospy.sleep(5)
    t = base_tf.getLatestCommonTime("/base_footprint", "/map")
    #base_tf.waitForTransform("/base_link", "/map", rospy.get_rostime(), rospy.Duration(10))
    base_position, base_quaternion = base_tf.lookupTransform("/map", "/base_footprint", t)
    base_pos = np.array([base_position[0], base_position[1], 0])
    
    stand_data = get_stand_data(stand_no, path)
    stand_pos = np.array([stand_data["pos"][0], stand_data["pos"][1], 0])
    
    stand_orr = calc_stand_orrientation(base_pos, stand_pos)
    
    cmd_publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
    
    cmd_data = Twist()
    cmd_data.linear.x = 0.0
    cmd_data.linear.y = 0.0
    cmd_data.linear.z = 0.0
    cmd_data.angular.x  = 0.0
    cmd_data.angular.y = 0.0
    

    
    # Start rotating robot
    diff = 0.3
    cmd_data.angular.z = diff
    #while True:
    rospy.loginfo("Turning with 0.3 rad/s")
    while diff > 0:
        
        t = base_tf.getLatestCommonTime("/base_footprint", "/map")
        base_position_new, base_quaternion_new = base_tf.lookupTransform("/map", "/base_footprint", t)
        # Kan også løses med et for loop, hvor diffen og hastigheden skaleres
        #for i in range(3,0)
        #rospy.loginfo("og_orrientation: %s, new_orientation: %s", base_quaternion_og, base_quaternion_new)
        #rospy.sleep(1)
        #print(cmd_data)
        #print(stand_orr)
        print("Q_diff1: ", base_quaternion_new[2], " - ", stand_orr[3], " = ", base_quaternion_new[2] - stand_orr[3])
        print("Q_diff2: ", base_quaternion_new[3], " - ", stand_orr[2], " = ", base_quaternion_new[3] - stand_orr[2])
        if abs(base_quaternion_new[2] - stand_orr[3]) < 0.05 and abs(base_quaternion_new[3] - stand_orr[2]) > 1.995:
        
            #decresse speed when nearing half rotation, and stop when Base is rotated 180 degrees
            rospy.sleep(2.0)
            diff = 0.0
            cmd_data.angular.z = diff
            
            
            rospy.loginfo("setting turning speed to: %f", diff)
        
        cmd_publisher.publish(cmd_data)
        rospy.sleep(0.1)
        
        
    #curr_speed = rospy.wait_for_message("cmd_vel", Twist, timeout=2)
    #if curr_speed.angular.z == 0:
    if not diff:
        rospy.loginfo("turning complete, approaching stand with 0.1 m/s")
        setup_lift_switch()
        GPIO_switch_in = 13
        try:
            cmd_data.linear.x = -0.1
            curr_time = rospy.get_rostime()
            while not GPIO.input(GPIO_switch_in): # or curr_time - new_time < 5:
                cmd_publisher.publish(cmd_data)
                    #rospy.loginfo("cmd_data: %s", cmd_data)
                rospy.sleep(0.1)
                new_time = rospy.get_rostime()
                
            rospy.loginfo("Stand detected")
            #client.cancel_all_goals()
            cmd_data.linear.x = 0.0; cmd_data.linear.y = 0.0; cmd_data.linear.z = 0.0
            cmd_publisher.publish(cmd_data)
            rospy.loginfo("Stopping robot")

        except KeyboardInterrupt:
            print("stupid")
        
        finally:
            twist = Twist()
            twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
            cmd_publisher.publish(twist)
        #get_switch_status(True)

        
        #while not get_switch_status(False):
        #    pass
        #rospy.sleep(4)

    return 1
    
def stand_pose_cb(msg):
    stand_update = msg.data

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
        stand_no = int(sys.argv[1])
    else:
        print(usage())
        print(len(sys.argv))
        sys.exit(1)
    rospy.loginfo("Requesting stand pose")
    
    rospy.init_node("stand_goal_client", anonymous = True)
    
    
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("Waiting for Move Base server")
    client.wait_for_server()
    
    base_tf = tf.TransformListener()
    rospy.sleep(1)
    
    # print(client.get_state())

    
    rospack = rospkg.RosPack()
    path = rospack.get_path('stand_pose')
    stand_data = get_stand_data(stand_no, path)
    
    old_stand_data = stand_data
    
    goal_dist = send_stand_goal(stand_no, stand_data)
    #goal_dist = 1.0
    print("goal dist: ", goal_dist)
    rospy.loginfo("Sent first goal to stand %i", stand_no)
        #while client.get_state() != 3:
    if goal_dist != 0:
        while goal_dist > 1.5:
            stand_time = rospy.Time(stand_data["stamp"]["secs"], stand_data["stamp"]["nsecs"])

            stand_data = get_stand_data(stand_no, path)
            #rospy.Subscriber("stand_pose_updated", stand_pose_cb)
            rospy.loginfo("distance to goal is: %f", goal_dist)
            if stand_data != old_stand_data:
                rospy.loginfo("New stand data found")
                goal_dist = send_stand_goal(stand_no, stand_data)
                rospy.loginfo("Sent new goal to stand %i", stand_no)
            old_stand_data = stand_data
                    
        rospy.loginfo("distance to goal is: %f", goal_dist)
        rospy.loginfo("Cancelling goal")
        client.cancel_all_goals()
        #delete_dir()
        turn_to_lift()

    
    else:
        rospy.signal_shutdown("Stand number %i does not exist in world", stand_number)
            
            
            
            
            
            
            
            
            
    
