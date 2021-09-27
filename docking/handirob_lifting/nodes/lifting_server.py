#!/usr/bin/env python
import time
import rospy
import actionlib
import Jetson.GPIO as GPIO
from std_srvs.srv import Trigger, TriggerResponse
from handirob_lifting.msg import LiftingAction, LiftingFeedback, LiftingResult
from handirob_lifting.msg import LoweringAction, LoweringFeedback, LoweringResult


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
        GPIO.setmode(GPIO.BCM)
        self.lift = [6, 12]
        GPIO.setup(self.lift, GPIO.OUT, initial=(GPIO.HIGH, GPIO.LOW))
        self.switch_pwr = 5
        GPIO.setup(self.switch_pwr, GPIO.OUT, initial=GPIO.HIGH)
        self.switch_in = 13
        self.switch_low = 26
        GPIO.setup((self.switch_in, self.switch_low), GPIO.IN)
        self.sleep_time = 3
        self.state = 'LOW'
        self._lower()
        self._as_lift.start()
        self._as_lower.start()

    def _wait(self):
        time.sleep(self.sleep_time)
    
    def _lower(self):
        GPIO.output(self.lift, (GPIO.HIGH, GPIO.LOW))
        print('Lowering module')

    def _lift(self):
        GPIO.output(self.lift, (GPIO.LOW, GPIO.HIGH))
        print('Lifting module')

    def _is_low(self):
        return bool(GPIO.input(self.switch_low))

    def _is_in(self):
        return bool(GPIO.input(self.switch_in))

    def _lift_srv(self, req):
        if self._is_in() and self._is_low():
            self._lift()
            self._wait()
            return TriggerResponse(True, 'Module lifted')
        elif self._is_in():
            return TriggerResponse(True, 'Module already lifted')
        elif not self._is_low():
            return TriggerResponse(False, 'Lifter alreay lifted')
        return TriggerResponse(False, 'Module not docked')
        
    def _lower_srv(self, req):
        if not self._is_low():
            self._lower()
            self._wait()
            if self._is_in() and self._is_low():
                return TriggerResponse(True, 'Module lowered')
            elif self._is_low():
                return TriggerResponse(True, 'Lifter lowered')
            return TriggerResponse(False, 'Lowering failed')
        elif self._is_in():
            return TriggerResponse(True, 'Module already lowered')
        return TriggerResponse(True, 'Lifter already lowered')


    def _lift_action(self, goal):
        def set_result(result, message):
            self._lift_result.result = result
            self._lift_result.msg = message
            self._as_lift.set_succeeded(self._lift_result)

        if self._is_in() and self._is_low():
            r = rospy.Rate(1)
            time_remaining = self.sleep_time
            self._lift()
            while time_remaining:
                self._lift_feedback.time_left = time_remaining
                self._as_lift.publish_feedback(self._lift_feedback)
                time_remaining -= 1
                print(time_remaining)
                r.sleep()
            set_result(True, 'Module lifted')
        elif self._is_in():
            set_result(True, 'Module already lifted')
        elif not self._is_low():
            set_result(False, 'Lifter already lifted')
        else:
            set_result(False, 'Module not docked')

    def _lower_action(self, goal):
        def set_result(result, message):
            self._lower_result.result = result
            self._lower_result.msg = message
            self._as_lower.set_succeeded(self._lower_result)

        if not self._is_low():
            r = rospy.Rate(1)
            time_remaining = self.sleep_time
            self._lower()
            while time_remaining:
                self._lower_feedback.time_left = time_remaining
                self._as_lower.publish_feedback(self._lower_feedback)
                time_remaining -= 1
                print(time_remaining)
                r.sleep()
            if self._is_in() and self._is_low():
                set_result(True, 'Module lowered')
            elif self._is_low():
                set_result(True, 'Lifter lowered')
            else:
                set_result(False, 'Lowering failed')
        elif self._is_in():
            set_result(True, 'Module already lowered')
        else:
            set_result(True, 'Lifter already lowered')

    def shutdown(self):
        GPIO.cleanup(self.lift)
        GPIO.cleanup(self.switch_pwr)
        GPIO.cleanup(self.switch_in)

def main():
    rospy.init_node('LiftingMechanism', anonymous=True, log_level=rospy.INFO)
    liftingMechanism = LiftingMechanism()
    rospy.on_shutdown(liftingMechanism.shutdown)
    rospy.spin()

if __name__ == '__main__':
    main()
