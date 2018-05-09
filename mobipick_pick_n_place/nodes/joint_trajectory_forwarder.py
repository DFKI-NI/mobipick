#!/usr/bin/env python

"""
This node provides a joint trajectory action server that forwards the trajectory to Rock for execution.
"""

import rospy
import actionlib
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
    FollowJointTrajectoryResult,
    FollowJointTrajectoryFeedback)
from trajectory_msgs.msg import JointTrajectory


class JointTrajectoryForwarder(object):
    # create messages that are used to publish feedback/result
    _feedback = FollowJointTrajectoryFeedback()
    _result = FollowJointTrajectoryResult()

    def __init__(self, name):
        self._action_name = name
        self._pub = rospy.Publisher('joint_trajectory_forwarder/trajectory', JointTrajectory, queue_size=1)
        self._as = actionlib.SimpleActionServer(self._action_name, FollowJointTrajectoryAction,
                                                execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def execute_cb(self, goal):
        """
        Process a new goal (i.e., execute the joint trajectory contained in the goal).

        :param goal: The goal to be executed.
        :type goal FollowJointTrajectoryGoal
        """

        rospy.loginfo('%s: Received new goal with %d points', self._action_name, len(goal.trajectory.points))

        # start executing the action
        rospy.loginfo('%s: Executing trajectory', self._action_name)
        self._pub.publish(goal.trajectory)

        # the Rock side should now have received the trajectory and started executing it
        # we'll wait for the trajectory to finish here

        # prepare the feedback message that is continually published back
        self._feedback.header.frame_id = goal.trajectory.header.frame_id
        self._feedback.joint_names = goal.trajectory.joint_names

        # TODO: actually receive some sort of feedback from the Rock side when the trajectory has finished
        # we fake this here by sleeping until the nominal end of the trajectory
        if goal.trajectory.header.stamp.is_zero():
            traj_end_time = rospy.Time.now() + goal.trajectory.points[-1].time_from_start
        else:
            traj_end_time = goal.trajectory.header.stamp + goal.trajectory.points[-1].time_from_start


        # publish feedback at 100 Hz
        r = rospy.Rate(100)

        while not rospy.is_shutdown():
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                return  # return instead of break so that we don't call set_succeeded/set_aborted

            # check if the trajectory is supposed to have finished
            if rospy.Time.now() >= traj_end_time:
                rospy.loginfo('%s: Trajectory end time reached' % self._action_name)
                break

            # TODO: check somehow if the Rock side has finished and break if yes

            # publish the feedback
            self._feedback.header.stamp = rospy.Time.now()
            # TODO: set self._feedback.{actual, desired, error} here
            self._as.publish_feedback(self._feedback)

            r.sleep()

        success = True  # TODO: actually determine this from the Rock side

        if success:
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._result.error_code = FollowJointTrajectoryResult.SUCCESSFUL
            self._result.error_string = "trajectory successfully executed"
            self._as.set_succeeded(self._result)
        else:
            rospy.loginfo('%s: Failed' % self._action_name)
            self._result.error_code = FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED  # TODO: or other error code
            self._result.error_string = "trajectory execution failed, because TODO"
            self._as.set_aborted(self._result)


def main():
    rospy.init_node('joint_trajectory_forwarder')
    server = JointTrajectoryForwarder('arm_controller/follow_joint_trajectory')
    rospy.spin()


if __name__ == '__main__':
    main()
