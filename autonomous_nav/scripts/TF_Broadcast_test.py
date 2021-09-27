#!/usr/bin/env python
# import roslib
# roslib.load_manifest('learning_tf')

import rospy
import tf
from geometry_msgs.msg import PoseStamped, Twist
import RPi.GPIO as GPIO



def setup_lift_switch():
    
    GPIO.setwarnings(False)
    GPIO_switch_pwr = 5
    
    GPIO.setmode(GPIO.BCM) # Broadcom pin-numbering scheme
    GPIO.setup(13, GPIO.IN)
    GPIO.setup(GPIO_switch_pwr, GPIO.OUT, initial=GPIO.HIGH)



def turn_to_lift():
    cmd_publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
    cmd_data = Twist()
    cmd_data.linear.x = 0.0
    cmd_data.linear.y = 0.0
    cmd_data.linear.z = 0.0
    cmd_data.angular.x  = 0.0
    cmd_data.angular.y = 0.0
    
    base_tf = tf.TransformListener()
    rospy.sleep(1)
    t = base_tf.getLatestCommonTime("/base_link", "/map")
    base_position_og, base_quaternion_og = base_tf.lookupTransform("/map", "/base_link", t)
    
    #if abs(base_quaternion[2]) != base_quaternion[3]+0.01 && abs(base_quaternion[3]) != base_quaternion[2]+0.01:
    # Start rotating robot
    #cmd_data.angular.z = 0.3
    #cmd_publisher.publish(cmd_data)
    
    diff = 0.3
    cmd_data.angular.z = diff
    #while True:
    rospy.loginfo("Turning with 0.3 rad/s")
    while diff > 0:
        
        t = base_tf.getLatestCommonTime("/base_link", "/map")
        base_position_new, base_quaternion_new = base_tf.lookupTransform("/map", "/base_link", t)
        # Kan også løses med et for loop, hvor diffen og hastigheden skaleres
        #for i in range(3,0)
        #rospy.loginfo("og_orrientation: %s, new_orientation: %s", base_quaternion_og, base_quaternion_new)
        #rospy.sleep(1)
        if abs(base_quaternion_og[2] - base_quaternion_new[3]) < 0.01 and abs(base_quaternion_og[3] - base_quaternion_new[2]) < 0.01:
        
            #decresse speed when nearing half rotation, and stop when Base is rotated 180 degrees
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
            while not GPIO.input(GPIO_switch_in):
                cmd_publisher.publish(cmd_data)
                    #rospy.loginfo("cmd_data: %s", cmd_data)
                rospy.sleep(0.1)
                
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

if __name__ == '__main__':
    rospy.init_node('turn_to_lift')
    turn_to_lift()
    rospy.signal_shutdown("test")
