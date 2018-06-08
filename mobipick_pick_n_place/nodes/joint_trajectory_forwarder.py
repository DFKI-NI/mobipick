#!/usr/bin/env python

"""
This node provides a joint trajectory action server that forwards the trajectory to Rock for execution.
"""

import sys
import rospy
import actionlib
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
    FollowJointTrajectoryResult,
    FollowJointTrajectoryFeedback)
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Int32

class JointTrajectoryForwarder(object):
    # create messages that are used to publish feedback/result
    _feedback = FollowJointTrajectoryFeedback()
    _result = FollowJointTrajectoryResult()

    def __init__(self, name):
        self._pub = rospy.Publisher('~trajectory', JointTrajectory, queue_size=1)
        action_name = name + '/follow_joint_trajectory'
        self._as = actionlib.SimpleActionServer(action_name, FollowJointTrajectoryAction,
                                                execute_cb=self.execute_cb, auto_start=False)

        self._sub_errorcode = rospy.Subscriber('/mobipick/arm_joint_trajectory_interface/error_code', Int32, self.callback_errorcode)
        self._sub_status = rospy.Subscriber('/mobipick/arm_joint_trajectory_interface/status', Int32, self.callback_status)

        self.error_code = 0
        self.status = 0
        self.success = 0
        self._pub_cancel = rospy.Publisher('~cancel', Int32, queue_size=1)

        self._as.start()

    def callback_status(self, data):
        rospy.loginfo('%s: Received new status code- %d', rospy.get_name(), data.data)
        self.status = data.data

        if self.status == 0:
            rospy.loginfo('%s: Executing trajectory', rospy.get_name())
        elif self.status == 1:
            rospy.loginfo('%s: Reached intermediate waypoint', rospy.get_name())
        elif self.status == 2:
            rospy.loginfo('%s: Reached final waypoint. Trajectory completed', rospy.get_name())
        elif self.status == 3:
            rospy.loginfo('%s: Ready to execute next trajectory', rospy.get_name())
        else:
            rospy.loginfo('%s: Error. Aborting trajectory', rospy.get_name())
            self._pub_cancel = rospy.Publisher('~cancel', Int32, queue_size=1)

    def callback_errorcode(self, data):
        rospy.loginfo('%s: Received new error code- %d', rospy.get_name(), data.data)
        self.error_code = data.data

        if self.error_code == 0:
            self.success = True
        elif self.error_code <0:
            self.success = False


    def execute_cb(self, goal):
        """
        Process a new goal (i.e., execute the joint trajectory contained in the goal).

        :param goal: The goal to be executed.
        :type goal FollowJointTrajectoryGoal
        """

        rospy.loginfo('%s: Received new goal with %d points', rospy.get_name(), len(goal.trajectory.points))

        # start executing the action
        rospy.loginfo('%s: Executing trajectory', rospy.get_name())
        self._pub_cancel.publish(0);
        self.success = 0
        self._pub.publish(goal.trajectory)

        # the Rock side should now have received the trajectory and started executing it
        # we'll wait for the trajectory to finish here

        # prepare the feedback message that is continually published back
        self._feedback.header.frame_id = goal.trajectory.header.frame_id
        self._feedback.joint_names = goal.trajectory.joint_names

        # TODO: actually receive some sort of feedback from the Rock side when the trajectory has finished
        # we fake this here by sleeping until the nominal end of the trajectory
        if goal.trajectory.header.stamp.is_zero():
            traj_end_time = rospy.Time.now() + 2*(goal.trajectory.points[-1].time_from_start)
        else:
            traj_end_time = goal.trajectory.header.stamp + 2*(goal.trajectory.points[-1].time_from_start)


        # publish feedback at 100 Hz
        r = rospy.Rate(100)

        while not rospy.is_shutdown():
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % rospy.get_name())
                self._pub_cancel.publish(1);
                self._as.set_preempted()
                return  # return instead of break so that we don't call set_succeeded/set_aborted

            # check if the trajectory is supposed to have finished
            if self.success:
                rospy.loginfo('%s: Trajectory completed' % rospy.get_name())
                break
            if rospy.Time.now() >= traj_end_time:
                rospy.loginfo('%s: Trajectory end time reached' % rospy.get_name())
                if self.success == False:
                    rospy.loginfo('%s: Trajectory not completed' % rospy.get_name())
                    self._pub_cancel.publish(1);
                break

            # TODO: check somehow if the Rock side has finished and break if yes

            # publish the feedback
            self._feedback.header.stamp = rospy.Time.now()
            # TODO: set self._feedback.{actual, desired, error} here
            self._as.publish_feedback(self._feedback)

            r.sleep()

          # TODO: actually determine this from the Rock side

        if self.success:
            rospy.loginfo('%s: Succeeded' % rospy.get_name())
            self._result.error_code = FollowJointTrajectoryResult.SUCCESSFUL
            self._result.error_string = "trajectory successfully executed"
            self._as.set_succeeded(self._result)
        else:
            rospy.loginfo('%s: Failed' % rospy.get_name())
            self._result.error_code = FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED  # TODO: or other error code
            self._result.error_string = "trajectory execution failed, check rock side"
            self._as.set_aborted(self._result)


def main():
    rospy.init_node('joint_trajectory_forwarder')
    rospy.set_param('~controller_name', 'arm_controller')
    try:

        controller_name = rospy.get_param('~controller_name')   # for example 'arm_controller' or 'gripper_controller'
    except KeyError:
        rospy.logfatal('Parameter ~controller_name was not set!')
        sys.exit(1)
    server = JointTrajectoryForwarder(controller_name)
    rospy.spin()


if __name__ == '__main__':
    main()
