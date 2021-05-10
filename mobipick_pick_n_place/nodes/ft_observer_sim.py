#! /usr/bin/env python3

import rospy

import actionlib

import geometry_msgs.msg._WrenchStamped
from mobipick_pick_n_place.msg import FtObserverAction, FtObserverActionFeedback, FtObserverActionGoal, FtObserverActionResult
from geometry_msgs.msg import WrenchStamped
from robotiq_ft_sensor.srv import sensor_accessor, sensor_accessorRequest
from std_srvs.srv import Empty

class ForceTorqueObserver(object):
    _feedback = FtObserverActionFeedback
    _result = FtObserverActionResult
    _wrench = WrenchStamped
    _actual_force = 0.0
    _timer_is_running=False
    _user_interaction_simulated=False

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, FtObserverAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        self._srv = rospy.Service("user_interaction", Empty, self.interact_cb)
        rospy.loginfo("ft observer action server in simulation mode started")

    def execute_cb(self, goal):
        r = rospy.Rate(1)
        self._user_interaction_simulated = False
        rospy.loginfo("Optionally call service user_interaction to simulate user interaction")
        rospy.Timer(rospy.Duration(goal.timeout), self.timer_cb, True)
        self._timer_is_running = True
        # start executing the action
        while self._timer_is_running:
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()

            # publish the feedback
            #self._as.publish_feedback(self._feedback)
            # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
            r.sleep()
        if self._user_interaction_simulated:
            self._result.catched = True
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
        else:
            self._as.set_aborted()

    def timer_cb(self, time):
        self._timer_is_running=False

    def interact_cb(self, req):
        if not self._user_interaction_simulated and self._timer_is_running:
            self._user_interaction_simulated = True
            self._timer_is_running=False
            rospy.loginfo("User interaction simulated")
        return []


if __name__ == '__main__':
    rospy.init_node('ft_observer')
    server = ForceTorqueObserver(rospy.get_name())
    rospy.spin()
