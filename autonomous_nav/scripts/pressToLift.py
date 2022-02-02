#! /usr/bin/env python3
### BEGIN INIT INFO
# Provides:          pressToLift.py
# Required-Start:    $remote_fs $syslog
# Required-Stop:     $remote_fs $syslog
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: Start daemon at boot time
# Description:       Enable service provided by daemon.
### END INIT INFO

import RPi.GPIO as GPIO
import time
import rospy
import sys



__GPIO_lift_1 = 6
__GPIO_lift_2 = 12
__GPIO_switch_pwr = 5
__GPIO_switch_in = 13
__GPIO_switch_lowered = 26


GPIO.setmode(GPIO.BCM) # Broadcom pin-numbering scheme

GPIO.setwarnings(False)

GPIO.setup(__GPIO_lift_1, GPIO.OUT, initial=GPIO.HIGH)
GPIO.setup(__GPIO_lift_2, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(__GPIO_switch_in, GPIO.IN)
GPIO.setup(__GPIO_switch_lowered, GPIO.IN)
GPIO.setup(__GPIO_switch_pwr, GPIO.OUT, initial=GPIO.HIGH)

def _lift_stand():
    GPIO.output(__GPIO_lift_1, GPIO.LOW)
    GPIO.output(__GPIO_lift_2, GPIO.HIGH)

def _lower_stand():
    GPIO.output(__GPIO_lift_1, GPIO.HIGH)
    GPIO.output(__GPIO_lift_2, GPIO.LOW)

if __name__=="__main__":

    rospy.init_node("Press_to_lift_node")
    rospy.loginfo("Simple lifting node started")
    rospy.loginfo("Press the switch to start lifting")
    rospy.loginfo("Lift will lower when switch is released.")

    while not rospy.is_shutdown():
        GPIO.wait_for_edge(__GPIO_switch_in, GPIO.BOTH)
        #if GPIO.input(__GPIO_switch_in):
        #    time.sleep(0.5)
        print(GPIO.input(__GPIO_switch_in))
        if GPIO.input(__GPIO_switch_in):
            rospy.loginfo("Lifting")
            _lift_stand()
        else:
            time.sleep(0.2)
            rospy.loginfo("Lowering")
            _lower_stand()
