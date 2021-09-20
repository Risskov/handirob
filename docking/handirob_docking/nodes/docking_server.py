#!/usr/bin/env python
import math
import rospy
import actionlib
from geometry_msgs.msg import PoseStamped, Twist
from object_msgs.msg import Objects
from smooth_docking.msg import DockingAction, DockingFeedback, DockingResult
from std_srvs.srv import Trigger
from scipy.spatial.transform import Rotation as R
import tf2_ros
import tf2_geometry_msgs

class DockingServer():
    _feedback = DockingFeedback()
    _result = DockingResult()

    def __init__(self, name):
        self.camera_frame = rospy.get_param('~camera_frame')
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self._as = actionlib.SimpleActionServer(name, DockingAction, execute_cb=self._docking_behavior, auto_start=False)
        self.lift_bin_srv = rospy.ServiceProxy('/lifting_server/lift_module', Trigger)
        cmd_vel_topic = rospy.get_param('~cmd_vel_topic')
        self.pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size = 10)
        object_topic = rospy.get_param('~object_topic')
        self.object_sub = rospy.Subscriber(object_topic, Objects, self._object_callback)
        self._restart_signals()

        self.linear_speed = 0.1
        self.k0 = 0.3
        self.k1 = 1.0
        self.k2 = 0.25
        self.docking_tolerance = 0.25
        self.wall_tolerance = 0.24

        self._as.start()
        # self._docking_behavior()

    def _restart_signals(self):
        self.plane_dist = None
        self.plane_angle = None
        self.object_dx = None
        self.object_dy = None
        self.object_dist = None
        self.object_angle = None

    def _transform_pose(self, header, pose):
        pose_stamped = PoseStamped(pose=pose, header=header)
        try:
            transform = self.tf_buffer.lookup_transform(self.camera_frame,
                                                        header.frame_id,
                                                        header.stamp,
                                                        rospy.Duration(0.2))
            return tf2_geometry_msgs.do_transform_pose(pose_stamped, transform).pose
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return None

    def _object_callback(self, objects):
        min_object_dist = None
        for object_msg in objects.objects:
            if object_msg.label == 100:
                pose = object_msg.poses[0].pose
                self.bin_pos = pose.position
                pose = self._transform_pose(object_msg.header, pose)
                if pose is not None:
                    object_dist = math.hypot(pose.position.x, pose.position.y)
                    if min_object_dist is None or object_dist < min_object_dist:
                        min_object_dist = object_dist
                        self.object_dist = object_dist
                        self.object_dx = pose.position.x
                        self.object_dy = pose.position.y
                        quat = pose.orientation
                        self.object_angle = R.from_quat([quat.x, quat.y, quat.z, quat.w]).as_euler('xyz')[2]

    def _preempt(self):
        self._send_cmd_vel(0, 0)
        self._as.set_preempted()
        self._restart_signals()

    def _publish_feedback(self, distance, module_lifted):
        self._feedback.distance = distance
        self._feedback.module_lifted = module_lifted
        self._as.publish_feedback(self._feedback)

    def _wait_for_object(self, r):
        while self.object_dist is None or self.plane_angle is None:
            if self._as.is_preempt_requested():
                self._preempt()
                return False
            r.sleep()
        return True

    def _initial_rotation(self, r):
        off_dist = self.object_dist * math.sin(self.object_angle - math.atan2(self.object_dy, self.object_dx))
        while round(self.object_dy + off_dist * self.k1, 1) != 0:
            if self._as.is_preempt_requested():
                self._preempt()
                return False
            angular_vel = self.k0 * (self.object_dy + off_dist * self.k1)
            self._send_cmd_vel(0, angular_vel)
            self._publish_feedback(self.object_dist, False)
            r.sleep()
        return True

    def _docking(self, r):
        robot_pos = self.tf_buffer.lookup_transform("map", self.camera_frame, rospy.Time.now(), rospy.Duration(0.1)).transform.translation
        dist = math.hypot(self.bin_pos.x - robot_pos.x, self.bin_pos.y - robot_pos.y)
        while (dist > self.docking_tolerance) and self.k2 * math.tan(self.plane_angle) - self.plane_dist > self.wall_tolerance:
            if self._as.is_preempt_requested():
                self._preempt()
                return False
            self._send_cmd_vel(-1 * self.linear_speed, self.k0 * self.object_dy)
            robot_pos = self.tf_buffer.lookup_transform("map", self.camera_frame, rospy.Time.now(), rospy.Duration(0.1)).transform.translation
            dist = math.hypot(self.bin_pos.x - robot_pos.x, self.bin_pos.y - robot_pos.y)
            self._publish_feedback(self.object_dist, False)
            r.sleep()
        self._send_cmd_vel(0, 0)
        return True

    def _clear_wall(self, r):
        while self.plane_dist > -0.6:
            if self._as.is_preempt_requested():
                self._preempt()
                return False
            self._send_cmd_vel(self.linear_speed, self.k0 * self.plane_angle)
            self._publish_feedback(self.plane_dist, True)
            r.sleep()
        return True

    def _docking_behavior(self, req):
        self._restart_signals()
        r = rospy.Rate(10)
        print('waiting')
        if not self._wait_for_object(r):
            return
        print('rotating')
        if not self._initial_rotation(r):
            return
        print('docking')
        if not self._docking(r):
            return
        self._lift_module()
        if not self._clear_wall(r):
            return

        self._result.result = True
        self._as.set_succeeded(self._result)

    def _send_cmd_vel(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.pub.publish(msg)

    def _lift_module(self):
        print('Lifting')
        resp = self.lift_bin_srv()
        return resp.success


if __name__ == "__main__":
    rospy.init_node('Docking')
    server = DockingServer(rospy.get_name())
    rospy.spin()
