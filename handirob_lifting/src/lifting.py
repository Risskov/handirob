#! /usr/bin/env python3
import rospy
import actionlib
import RPi.GPIO as GPIO
import time
from handirob_lifting.msg import *
from std_msgs.msg import String

GPIO_lift_1 = 6
GPIO_lift_2 = 12
GPIO_switch_pwr = 5
GPIO_switch_in = 13

GPIO.setmode(GPIO.BCM) # Broadcom pin-numbering scheme

GPIO.setwarnings(False)

GPIO.setup(GPIO_lift_1, GPIO.OUT, initial=GPIO.HIGH)
GPIO.setup(GPIO_lift_2, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(GPIO_switch_in, GPIO.IN)
GPIO.setup(GPIO_switch_pwr, GPIO.OUT, initial=GPIO.HIGH)

# GPIO.setup(GPIO_switch_pwr, GPIO.HIGH)


    

def lower_stand():
    GPIO.output(GPIO_lift_1, GPIO.HIGH)
    GPIO.output(GPIO_lift_2, GPIO.LOW)

def lift_stand():
    for k in range(2):
        if GPIO.input(GPIO_switch_in):
            rospy.sleep(1)
            if GPIO.input(GPIO_switch_in):
                GPIO.output(GPIO_lift_1, GPIO.LOW)
                GPIO.output(GPIO_lift_2, GPIO.HIGH)
                rospy.loginfo("Stand is attached, and robot will start lifting.")
                return 1
        else:
            rospy.logwarn("Stand is not placed in the lift, waiting..")
            for i in range(30):
                if GPIO.input(GPIO_switch_in):
                    break
                rospy.sleep(0.5)

    rospy.logerr("Stand was never placed in the lift, aborting..")
    return 0

def get_pos():
    if GPIO.input(GPIO_lift_1) == 0 and GPIO.input(GPIO_lift_2) == 1:
        return 1 # if lifted
    if GPIO.input(GPIO_lift_1) == GPIO.input(GPIO_lift_2):
        rospy.logwarn("Lifting mechanism inactive. Lowering lift.")
        lower_stand()
    return 0 # if lowered


class LiftingServer():
    
    def __init__(self):
        self.server = actionlib.SimpleActionServer('lift_stand', liftAction, self.lift, False)
        
        self.server.start()
    
    def lift_activity_check(self):
        start_time = rospy.get_time()
        while not self.server.is_new_goal_available():
            if not GPIO.input(GPIO_switch_in):
                self.server.publish_feedback(liftFeedback(0, 0))
                self.server.set_aborted(text="Stand is lost")
                lower_stand()
                while not GPIO.input(GPIO_switch_in):
                    pass
            if start_time < (rospy.get_time() - 5):
                self.server.publish_feedback(liftFeedback(1,1))
        return self.server.accept_new_goal()
        

    def lift(self, goal):
        if goal.lift == 1: # Lift the stand
            if not get_pos():
                if lift_stand():
                    
                    goal.lift = self.lift_activity_check()
                    
                else:
                    self.server.set_preempted()
                    self.server.set_aborted(text="Unable to lift stand")
            else:
                rospy.logwarn("Stand is already lifted")
                goal.lift = self.lift_activity_check()

        
                
        print("testing goal again, goal = ", goal.lift)
        if goal.lift == 0: # Lower the stand
            print("start lowering after lifting")
            if get_pos():
                lower_stand()
                rospy.sleep(5)
                self._result.lifting_completed = True
                rospy.loginfo("Lowereing stand")
                self.server.set_succeeded(self._result)
                
                
            else:
                self._result.lifting_completed = True
                rospy.logwarn("Lift already lowered")
                self.server.set_succeeded(True)
        



if __name__ == "__main__":
    
    rospy.init_node('lifting_mechanism')
    #pub_status = rospy.Publisher('lifting/status', String, queue_size=10)
    #pub_error = rospy.Publisher('lifting/error', String, queue_size=10)
    
    #pub_status.publish(String("down"))
    
    server = LiftingServer()
    rospy.spin()

#while 1:
    
    #lift_stand
    #if GPIO.input(GPIO_switch_in):
    #    GPIO.output(GPIO_lift_1, GPIO.LOW)
    #    GPIO.output(GPIO_lift_2, GPIO.HIGH)
    #else:
    #    GPIO.output(GPIO_lift_1, GPIO.HIGH)
    #    GPIO.output(GPIO_lift_2, GPIO.LOW)
