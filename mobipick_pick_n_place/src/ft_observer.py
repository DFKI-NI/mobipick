#! /usr/bin/env python

import rospy

import actionlib

import geometry_msgs.msg._WrenchStamped
from mobipick_pick_n_place.msg import FtObserverAction, FtObserverActionFeedback, FtObserverActionGoal, FtObserverActionResult
from geometry_msgs.msg import WrenchStamped
from robotiq_ft_sensor.srv import sensor_accessor, sensor_accessorRequest

class ForceTorqueObserver(object):
    _feedback = FtObserverActionFeedback
    _result = FtObserverActionResult
    _wrench = WrenchStamped
    _actual_force = 0.0
    _timer_is_running=False

    def __init__(self, name):
        rospy.wait_for_service('/mobipick/ft_sensor/sensor_acc')
        rospy.loginfo("connected to sensor accessor service")    
        rospy.Subscriber("/mobipick/ft_sensor/wrench", WrenchStamped, self.wrench_cb)
        rospy.loginfo("Subscribed to ft_sensor/wrench")
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, FtObserverAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
      
    def execute_cb(self, goal):
        success = False
        rospy.loginfo("New goal, try to set ft sensor to zero!")
        try:
            sensor_acc = rospy.ServiceProxy('/mobipick/ft_sensor/sensor_acc', sensor_accessor)
            acc_srv = sensor_accessorRequest()
            acc_srv.command_id=8
            respond = sensor_acc(acc_srv)
            if respond.success:
                rospy.loginfo("ft sensor set to zero")
                wait_for_detection = True
                rospy.sleep(5)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        
        # helper variables
        r = rospy.Rate(1)
        rospy.Timer(rospy.Duration(goal.timeout), self.timer_cb, True)
        self._timer_is_running = True
        # start executing the action
        while wait_for_detection and self._timer_is_running:
            if self._actual_force > goal.threshold:
                success=True
                wait_for_detection = False
                self._feedback.actualForce=self._actual_force
            rospy.loginfo("actual force: %f" % self._actual_force)

            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()

            
            # publish the feedback
            #self._as.publish_feedback(self._feedback)
            # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
            r.sleep()
          
        if success:
            self._result.catched = True
            rospy.loginfo('%s: Succeeded' % self._action_name)

            self._as.set_succeeded(self._result)
        else:
            self._as.set_aborted()

    def wrench_cb(self, data):
        self._wrench = data.wrench
        self._actual_force = data.wrench.force.y

    def timer_cb(self, time):
        self._timer_is_running=False


if __name__ == '__main__':
    rospy.init_node('ft_observer')
    server = ForceTorqueObserver(rospy.get_name())
    rospy.spin()
