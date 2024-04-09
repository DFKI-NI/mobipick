#!/usr/bin/env python3

"""This node provides a joint trajectory action server that forwards the trajectory to Rock for execution."""

import sys
import rospy
import actionlib
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
    FollowJointTrajectoryResult,
    FollowJointTrajectoryFeedback,
)
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Int32


class JointTrajectoryForwarder(object):
    # create messages that are used to publish feedback/result
    _feedback = FollowJointTrajectoryFeedback()
    _result = FollowJointTrajectoryResult()

    def __init__(self, name):
        self._pub = rospy.Publisher('~trajectory', JointTrajectory, queue_size=1)
        action_name = name + '/follow_joint_trajectory'
        self._as = actionlib.SimpleActionServer(
            action_name, FollowJointTrajectoryAction, execute_cb=self.execute_cb, auto_start=False
        )

        if name == 'arm_controller':
            topic_name = 'arm'
        elif name == 'gripper_controller':
            topic_name = 'gripper'
        self._sub_errorcode = rospy.Subscriber(
            '/mobipick/' + topic_name + '_joint_trajectory_interface/error_code', Int32, self.callback_errorcode
        )
        self._sub_status = rospy.Subscriber(
            '/mobipick/' + topic_name + '_joint_trajectory_interface/status', Int32, self.callback_status
        )
        self._sub_preempt_ext = rospy.Subscriber(
            '/mobipick/' + topic_name + '_joint_trajectory_interface/cancel_goal', Int32, self.callback_preempt_external
        )

        self.error_code = 1
        self.status = 0
        self._pub_cancel = rospy.Publisher('~cancel', Int32, queue_size=1)

        self._as.start()

    def callback_status(self, data):
        rospy.logdebug('%s: Received new status code: %d', rospy.get_name(), data.data)
        self.status = data.data

        if self.status == 0:
            rospy.logdebug('%s: Executing trajectory', rospy.get_name())
        elif self.status == 1:
            rospy.loginfo('%s: Reached intermediate waypoint', rospy.get_name())
        elif self.status == 2:
            rospy.loginfo('%s: Reached final waypoint. Trajectory completed', rospy.get_name())
        elif self.status == 3:
            rospy.loginfo('%s: Ready to execute next trajectory', rospy.get_name())
        else:
            rospy.loginfo('%s: Error code: %d', rospy.get_name(), self.status)

    def callback_preempt_external(self, data):
        rospy.loginfo('%s: Received explicit request for preempting the goal- %d', rospy.get_name(), data.data)
        if data.data == 1:
            rospy.loginfo('%s: Received explicit request for preempting the goal- %d', rospy.get_name(), data.data)
            self._as.preempt_request = True  # TODO: `self._as.set_preempted()` would probably be better

    def callback_errorcode(self, data):
        rospy.loginfo('%s: Received new error code- %d', rospy.get_name(), data.data)
        self.error_code = data.data

    def execute_cb(self, goal: FollowJointTrajectoryGoal):
        """
        Process a new goal (i.e., execute the joint trajectory contained in the goal).

        :param goal: The goal to be executed.
        """
        rospy.loginfo('%s: Received new goal with %d points', rospy.get_name(), len(goal.trajectory.points))

        # start executing the action
        rospy.loginfo('%s: Executing trajectory', rospy.get_name())
        self.error_code = 1
        self._pub.publish(goal.trajectory)

        # the Rock side should now have received the trajectory and started executing it
        # we'll wait for the trajectory to finish here

        # prepare the feedback message that is continually published back
        self._feedback.header.frame_id = goal.trajectory.header.frame_id
        self._feedback.joint_names = goal.trajectory.joint_names

        # store traj_start_time for debug output
        traj_start_time = rospy.Time.now()
        rospy.logdebug("Planned trajectory duration: %f s", goal.trajectory.points[-1].time_from_start.to_sec())

        # publish feedback at 100 Hz
        r = rospy.Rate(100)

        while not rospy.is_shutdown():
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % rospy.get_name())
                self._pub_cancel.publish(1)
                self._as.set_preempted()
                self._as.preempt_request = False  # TODO: not necessary with `self._as.set_preempted()` above
                return  # return instead of break so that we don't call set_succeeded/set_aborted

            # check if the trajectory is finished
            if self.error_code <= 0:
                rospy.loginfo(
                    '%s: Trajectory completed (planned duration: %f s, actual duration: %f s)',
                    rospy.get_name(),
                    goal.trajectory.points[-1].time_from_start.to_sec(),
                    (rospy.Time.now() - traj_start_time).to_sec(),
                )
                break

            # publish the feedback
            self._feedback.header.stamp = rospy.Time.now()
            # TODO: set self._feedback.{actual, desired, error} here
            self._as.publish_feedback(self._feedback)

            r.sleep()

        if self.error_code == 0:
            rospy.loginfo('%s: Succeeded' % rospy.get_name())
            self._result.error_code = FollowJointTrajectoryResult.SUCCESSFUL
            self._result.error_string = "trajectory successfully executed"
            self._as.set_succeeded(self._result)
        else:
            rospy.loginfo('%s: Failed' % rospy.get_name())
            self._result.error_code = self.error_code
            self._result.error_string = "trajectory execution failed, check rock side"
            self._as.set_aborted(self._result)


def main():
    rospy.init_node('joint_trajectory_forwarder')
    try:
        controller_name = rospy.get_param('~controller_name')  # for example 'arm_controller' or 'gripper_controller'
    except KeyError:
        rospy.logfatal('Parameter ~controller_name was not set!')
        sys.exit(1)
    _ = JointTrajectoryForwarder(controller_name)
    rospy.spin()


if __name__ == '__main__':
    main()
