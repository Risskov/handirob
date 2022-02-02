from autonomous_nav import approach_stand




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
    
    cmd_data = Twist()
    cmd_data.linear.x = 0.0
    cmd_data.linear.y = 0.0
    cmd_data.linear.z = 0.0
    cmd_data.angular.x  = 0.0
    cmd_data.angular.y = 0.0
    

    
    # Start rotating robot
    angle_speed = 0.3
    diff = 5
    cmd_data.angular.z = angle_speed
    rospy.loginfo("Turning with 0.3 rad/s")
    while True:
        
        t = base_tf.getLatestCommonTime("/base_footprint", "/map")
        base_position_new, base_quaternion_new = base_tf.lookupTransform("/map", "/base_footprint", t)
        # Kan også løses med et for loop, hvor diffen og hastigheden skaleres
        #for i in range(3,0)
        print("Q_diff1: ", base_quaternion_new[2], " - ", stand_orr[3], " = ", base_quaternion_new[2] - stand_orr[3])
        print("Q_diff2: ", base_quaternion_new[3], " - ", stand_orr[2], " = ", base_quaternion_new[3] - stand_orr[2])
        if (base_quaternion_new[2] - stand_orr[3]) < -0.1 and (base_quaternion_new[3] - stand_orr[2]) > 1.9:
        
            #decresse speed when nearing half rotation, and stop when Base is rotated 180 degrees
            diff -= 1
            angle_speed = 0.2
            cmd_data.angular.z = angle_speed
            rospy.loginfo("setting turning speed to: %f", angle_speed)
        
        if (base_quaternion_new[2] - stand_orr[3]) > 0 :
            
            angle_speed = 0
            cmd_data.angular.z = angle_speed
            rospy.loginfo("setting turning speed to: %f", angle_speed)

        if angle_speed <= 0:
            break
        
        cmd_publisher.publish(cmd_data)
        rospy.sleep(0.1)
        
        
    #curr_speed = rospy.wait_for_message("cmd_vel", Twist, timeout=2)
    #if curr_speed.angular.z == 0:
    if angle_speed <= 0:
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

class stand_search:

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

def find_stand():
    cmd_data = Twist()
    cmd_data.linear.x = 0.0
    cmd_data.linear.y = 0.0
    cmd_data.linear.z = 0.0
    cmd_data.angular.x  = 0.0
    cmd_data.angular.y = 0.0
    rate = rospy.Rate(5)
    while True:
        cmd_data.angular.z = 0.15
        cmd_publisher.publish(cmd_data)
        rate.sleep()
        if stop_thread:
            cmd_data.angular.z = 0.0
            cmd_publisher.publish(cmd_data)
            break

class get_fiducial_id:

    def __init__(self):
        self.fiducial_sub = rospy.Subscriber("/fiducial_verticies", FiducialTransformArray, self.callback)

    def callback(self, msg):
        self.msg = msg

    def get_fiducial(self):
        return(self.fiducials.fiducial_id)

    