#! /usr/bin/env python3
import rospy
import actionlib
import RPi.GPIO as GPIO
import time
import threading
from handirob_lifting.msg import *
from std_msgs.msg import String

class lifting_mechanism:

    __GPIO_lift_1 = 6
    __GPIO_lift_2 = 12
    __GPIO_switch_pwr = 5
    __GPIO_switch_in = 13
    __GPIO_switch_lowered = 26
    
    def __init__(self):


        GPIO.setmode(GPIO.BCM) # Broadcom pin-numbering scheme

        GPIO.setwarnings(False)

        GPIO.setup(self.__GPIO_lift_1, GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(self.__GPIO_lift_2, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.__GPIO_switch_in, GPIO.IN)
        GPIO.setup(self.__GPIO_switch_pwr, GPIO.OUT, initial=GPIO.HIGH)

        
    
    def check_stand(self):
        if GPIO.input(self.__GPIO_switch_in) == 1:
            return True # Stand is attatched
            
        return False # Stand is not in attached


    def lower_stand(self):
        if self.get_pos():
            GPIO.output(self.__GPIO_lift_1, GPIO.HIGH)
            GPIO.output(self.__GPIO_lift_2, GPIO.LOW)

    def lift_stand(self):
        if self.check_stand():
            GPIO.output(self.__GPIO_lift_1, GPIO.LOW)
            GPIO.output(self.__GPIO_lift_2, GPIO.HIGH)
            return True
        else:
            rospy.logerr("Stand is not detected")
            return False

    def get_pos(self):
        
        if GPIO.input(self.__GPIO_switch_lowered) == 1:
            return True # Stand is in the lowered position
        else:
            return False # Stand is not in the lowered position

        if GPIO.input(self.__GPIO_lift_1) == 0 and GPIO.input(self.__GPIO_lift_2) == 1:
            return 1 # if lifted
        if GPIO.input(self.__GPIO_lift_1) == GPIO.input(self.__GPIO_lift_2):
            rospy.logwarn("Lifting mechanism inactive. Lowering lift.")
            self.lower_stand()
        return 0

