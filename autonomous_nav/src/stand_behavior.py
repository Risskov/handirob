#!/usr/bin/env python3 

import rospy
import rospkg
import actionlib
import sys
# import tf
import numpy as np
import handirob_docking.msg
#import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from scipy.spatial.transform import Rotation as R


class stands:
    def __init__(self):
        self.stand_msgs = MarkerArray()
        self.stand_pose = Pose()
        #self.stand_sub = rospy.Subscriber("/object_tracker/objects/markers", MarkerArray, self._callback)
        #self.stand_sub = rospy.Subscriber("/stand_detector/objects/markers", MarkerArray, self._callback)

    def _callback(self, marker_msg):
        self.stand_msgs = marker_msg.markers


    def get_pose(self, id=0):
        self.get_msgs()
        self.stand_pose = self.stand_msgs.markers[1].pose #change to [2] for object tracker
        return self.stand_pose
    
    def get_msgs(self):
        self.stand_msgs = rospy.wait_for_message("/object_tracker/objects/markers", MarkerArray, timeout=5)
        #self.stand_msgs = rospy.wait_for_message("/stand_detector/objects/markers", MarkerArray, timeout=5)



class GoalClient:
    def __init__(self):
        self.goal_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.goal_client.wait_for_server()
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = "map"
        self.goal.target_pose.header.stamp = rospy.Time.now()

    def calc_object_orientation(self, _base_pos, _object_pos):
        #nt(_object_pos)
        #rospy.loginfo("objectorient: %s", _object_pos.orientation.x)
        bp = np.array([int(_base_pos.position.x), int(_base_pos.position.y), int(_base_pos.position.z)])
        sp = np.array([_object_pos.position.x, _object_pos.position.y, _object_pos.position.z])
        v2 = np.array([1, 0, 0])
        v1 = (sp - bp)
        q = np.cross(v2, v1)
        
        q = np.append(q, np.sqrt(np.linalg.norm(v1)**2) + np.dot(v1, v2))
        q = q / np.linalg.norm(q)

        r = R.from_euler('z', 180, degrees=True)
        r2 = R.from_quat([q[0], q[1], q[2], q[3]])
        rot = r2 * r
        rot = rot.as_quat()
        return array_to_orr(_object_pos, rot)

    def update_goal(self, Pose):
        # Init goal        
        self.goal.target_pose.pose = Pose
        #rospy.loginfo("Goal updated to %s", self.goal.target_pose.pose)



    def send_goal(self, Pose=0, wait_for_result=False):
        if Pose:
            self.update_goal(Pose)

        if wait_for_result:
            self.goal_client.send_goal_and_wait(self.goal)
        else:
        #rospy.loginfo("sending goal to: %s", self.goal.pose.position)
            self.goal_client.send_goal(self.goal)
    
    def cancel_goal(self):
        self.goal_client.cancel_all_goals()
    
    def goal_status(self):
        return self.goal_client.get_state

class DockingClient:
    def __init__(self):
        self.docking_client = actionlib.SimpleActionClient('docking_server', handirob_docking.msg.DockingAction)
        self.docking_client.wait_for_server()

    def start(self):
        goal = handirob_docking.msg.DockingGoal(start=True)
        self.docking_client.send_goal(goal)
        self.docking_client.wait_for_result()
        return self.docking_client.get_result()

def pose_to_array(pose):
    #posx = pose.__dict__
    #print("this is position x ", posx)
    return np.array([pose.position.x, pose.position.y, pose.position.z])

def array_to_pos(pose, pose_array):
    pose.position.x = pose_array[0]
    pose.position.y = pose_array[1]
    pose.position.z = pose_array[2]
    return pose

def array_to_orr(pose, orr_array):
    pose.orientation.x = orr_array[0]
    pose.orientation.y = orr_array[1]
    pose.orientation.z = orr_array[2]
    pose.orientation.w = orr_array[3]
    return pose

def goto_stand(_base_pos, _stand_pos):
    sp = pose_to_array(_stand_pos)
    pos_diff = ( sp - pose_to_array(_base_pos))
    stand_pos_approach = sp - pos_diff/np.linalg.norm(pos_diff) # Setting goal 1 M away from the stand, in the immediate direction
    _base_pos = array_to_pos(_base_pos, stand_pos_approach)
    stand_or = calc_object_orientation(_base_pos, _stand_pos)
    _base_pos.orientation.z = stand_or[2]
    _base_pos.orientation.w = stand_or[3]
    print(_base_pos)

    goal_client.send_goal(_base_pos, wait_for_result=True)
    status = goal_client.goal_status()
    return status




def get_base_pose():
    pose_msg = rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped, timeout=10)
    return pose_msg.pose.pose

def calc_object_orientation(_base_pos, _object_pos):
    print(_object_pos.position.x)
    bp = np.array([_base_pos.position.x, _base_pos.position.y, _base_pos.position.z])
    sp = np.array([_object_pos.position.x, _object_pos.position.y, _object_pos.position.z])
    v2 = np.array([1, 0, 0])
    v1 = (sp - bp)
    q = np.cross(v2, v1)
    
    q = np.append(q, np.sqrt(np.linalg.norm(v1)**2) + np.dot(v1, v2))
    q = q / np.linalg.norm(q)

    r = R.from_euler('z', 180, degrees=True)
    r2 = R.from_quat([q[0], q[1], q[2], q[3]])
    rot = r2 * r
    rot = rot.as_quat()
    return rot



def turn_to_dock(base_pos, stand_pos):

    stand_rot = calc_object_orientation(base_pos, stand_pos)
    # base_rot = R.from_quat([0, 0, np.sin(np.pi/4), np.cos(np.pi/4)])
    #r = R.from_euler('z', 180, degrees=True)
    #r2 = R.from_quat([stand_or[0], stand_or[1], stand_or[2], stand_or[3]])
    # r2 = R.from_quat([base_pos.orientation.x, base_pos.orientation.y, base_pos.orientation.z, base_pos.orientation.w])

    #stand_rot = r2 * r
    #stand_rot = stand_rot.as_quat()
    #print(stand_or)
    
    base_pos.orientation.z = stand_rot[2]
    base_pos.orientation.w = stand_rot[3]

    # print(base_pos)

    goal_client.update_goal(base_pos)
    goal_client.send_goal(wait_for_result=True)
    return goal_client.goal_status()

def main():
    rospy.init_node("simple_stand_behavior")
    global goal_client
    goal_client = GoalClient()
    docking = DockingClient()
    stand1 = stands()

    if goto_stand(get_base_pose(), stand1.get_pose()) != 3:
        rospy.logerr("Did not reach goal")

    turn_to_dock(get_base_pose(), stand1.get_pose())
    docking.start()




    # base_pos = get_base_pose()
    # print(base_pos)
    # stand1 = stands()
    
    # pose = stand1.get_pose()

    # goal_client.update_goal(pose)
    # goal_client.send_goal()
    # rospy.sleep(2)
    #goal_client.cancel_goal()
    



    
#main()