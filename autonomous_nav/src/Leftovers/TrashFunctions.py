    
    
    
    
    
    
def send_stand_goal(stand_number, stand_data):
    
    
    if stand_data:
        rospy.loginfo("Data for stand %i found. Setting goal.", stand_number)

        stand_pos = np.array([stand_data["pos"][0], stand_data["pos"][1], 0])
        
        #Getting base_footprint pose
        
        
        t = base_tf.getLatestCommonTime("/base_footprint", "/map")
        base_position, base_quaternion = base_tf.lookupTransform("/map", "/base_footprint", t)
        base_pos = np.array([base_position[0], base_position[1], 0])
        
        #Calculate direct orientation from base_footprint to stand
        q = calc_stand_orientation(base_pos, stand_pos)
        
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


class base_tf:
    def __init__(self):
        self.base_tf_listener = tf.TransformListener()
        self.base_pos = 0
        self.base_quart = 0

    def get_latest_pose(self, target_frame="/map"):
        t = base_tf.getLatestCommonTime("/base_link", "/map")
        self.base_pos, self.base_quart = self.base_tf_listener.lookupTransform(target_frame, "/base_footprint", t)
        return self.base_pos, self.base_quart
    
    def get_pos(self, target_frame="/map"):
        self.base_pos, self.base_quart = self.get_latest_pose(target_frame)
        return self.base_pos
    
    def get_quart(self, target_frame="/map"):
        self.base_pos, self.base_quart = self.get_latest_pose(target_frame)
        return self.base_quart