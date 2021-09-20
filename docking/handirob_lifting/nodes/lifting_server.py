#!/usr/bin/env python
import time
import rospy
import actionlib
import Jetson.GPIO as GPIO
from std_srvs.srv import Trigger, TriggerResponse
from smooth_lifting.msg import LiftingAction, LiftingFeedback, LiftingResult
from smooth_lifting.msg import LoweringAction, LoweringFeedback, LoweringResult


class LiftingMechanism:
    _lift_feedback = LiftingFeedback()
    _lift_result = LiftingResult()
    _lower_feedback = LoweringFeedback()
    _lower_result = LoweringResult()

    def __init__(self):
        self.lift_service = rospy.Service('~lift_module', Trigger, self._lift_srv)
        self.lower_service = rospy.Service('~lower_module', Trigger, self._lower_srv)
        self._as_lift = actionlib.SimpleActionServer(rospy.get_name(), LiftingAction, execute_cb=self._lift_action, auto_start=False)
        self._as_lower = actionlib.SimpleActionServer(rospy.get_name(), LoweringAction, execute_cb=self._lower_action, auto_start=False)
        GPIO.setmode(GPIO.BOARD)
        self.pins = [6, 12]
        GPIO.setup(self.pins, GPIO.OUT)
        self.sleep_time = 12
        self.state = 'LOW'
        self._lower()
        self._as_lift.start()
        self._as_lower.start()

    def _wait(self):
        time.sleep(self.sleep_time)
    
    def _lower(self):
        GPIO.output(self.pins, (GPIO.LOW, GPIO.LOW))
        print('Lowering module')

    def _lift(self):
        GPIO.output(self.pins, (GPIO.HIGH, GPIO.HIGH))
        print('Lifting module')

    # def _reset(self, mode):
    #     GPIO.output(self.pins, (GPIO.LOW, GPIO.LOW))
    #     print('{} module'.format(mode))

    def _lift_srv(self, req):
        if self.state == 'LOW':
            self._lift()
            self._wait()
            # self._reset('Lifted')
            self.state = 'HIGH'
            return TriggerResponse(True, self.state)
        return TriggerResponse(False, self.state)

    def _lower_srv(self, req):
        if self.state == 'HIGH':
            self._lower()
            self._wait()
            # self._reset('Lowered')
            self.state = 'LOW'
            return TriggerResponse(True, self.state)
        return TriggerResponse(False, self.state)

    def _lift_action(self, goal):
        if self.state == 'HIGH':
            self._lift_result.result = False
            self._as_lift.set_succeeded(self._lift_result)
        else:
            r = rospy.Rate(1)
            time_remaining = self.sleep_time
            self._lift()
            while time_remaining:
                self._lift_feedback.time_left = time_remaining
                self._as_lift.publish_feedback(self._lift_feedback)
                time_remaining -= 1
                print(time_remaining)
                r.sleep()
            self._reset('Lifted')
            self.state = 'HIGH'
            self._lift_result.result = True
            self._as_lift.set_succeeded(self._lift_result)

    def _lower_action(self, goal):
        if self.state == 'LOW':
            self._lower_result.result = False
            self._as_lower.set_succeeded(self._lower_result)
        else:
            r = rospy.Rate(1)
            time_remaining = self.sleep_time
            self._lower()
            while time_remaining:
                self._lower_feedback.time_left = time_remaining
                self._as_lower.publish_feedback(self._lower_feedback)
                time_remaining -= 1
                print(time_remaining)
                r.sleep()
            self._reset('Lowered')
            self.state = 'LOW'
            self._lower_result.result = True
            self._as_lower.set_succeeded(self._lower_result)


    def shutdown(self):
        GPIO.cleanup(self.pins)

def main():
    rospy.init_node('LiftingMechanism', anonymous=True, log_level=rospy.INFO)
    liftingMechanism = LiftingMechanism()
    rospy.on_shutdown(liftingMechanism.shutdown)
    rospy.spin()

if __name__ == '__main__':
    main()
