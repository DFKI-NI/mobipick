#! /usr/bin/env python3

import rospy

import actionlib

import control_msgs.msg

import trajectory_msgs.msg


class GripperBridgeAction(object):
    # create messages that are used to publish feedback/result
    _feedback = control_msgs.msg.GripperCommandFeedback()
    _result = control_msgs.msg.GripperCommandResult()

    def __init__(self, name, tf_prefix):
        self.tf_prefix = tf_prefix
        self.joint_trajectory_client()
        rospy.loginfo("Client connected")

        self._controller_sub = rospy.Subscriber(
            'gripper_controller/state', control_msgs.msg.JointTrajectoryControllerState, self.joint_state_cb
        )
        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name, control_msgs.msg.GripperCommandAction, execute_cb=self.execute_cb, auto_start=False
        )
        self._as.start()
        rospy.loginfo("Actionserver for gripper bridge started")

    def joint_state_cb(self, state):
        self._joint_state = state

    def joint_trajectory_client(self):
        self._jt_client = actionlib.SimpleActionClient(
            'gripper_controller/follow_joint_trajectory', control_msgs.msg.FollowJointTrajectoryAction
        )
        self._jt_client.wait_for_server()
        self._actual_goal = control_msgs.msg.FollowJointTrajectoryGoal()

    def set_jt_goal(self, position, effort=30.0):
        # self._actual_goal.header.stamp = rospy.Time.now()
        position = min(0.76, position)
        position = max(0.0, position)
        rospy.loginfo("Desired goal set to %.3f" % position)
        jt = trajectory_msgs.msg.JointTrajectory()
        jt_point = trajectory_msgs.msg.JointTrajectoryPoint()
        jt_point.positions.append(position)
        jt_point.time_from_start = rospy.Duration(2.0)
        jt_point.effort.append(effort)
        jt.points.append(jt_point)
        jt.joint_names.append(self.tf_prefix + 'gripper_finger_joint')
        self._actual_goal.trajectory = jt

    def execute_cb(self, goal):
        # helper variables
        rospy.loginfo(
            "Received gripper command with position=%.3f and effort=%.3f"
            % (goal.command.position, goal.command.max_effort)
        )
        r = rospy.Rate(10)
        success = True
        self.set_jt_goal(-0.76 / 0.14 * goal.command.position + 0.76, goal.command.max_effort)
        self._jt_client.send_goal(self._actual_goal)

        # Start executing the action
        while not rospy.is_shutdown():
            # Check if the action client has finished
            state = self._jt_client.get_state()
            if state in [
                actionlib.GoalStatus.SUCCEEDED,
                actionlib.GoalStatus.ABORTED,
                actionlib.GoalStatus.PREEMPTED,
                actionlib.GoalStatus.REJECTED,
                actionlib.GoalStatus.RECALLED,
            ]:
                break  # Stop if client reports the goal is already done

            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._jt_client.cancel_goal()  # Cancel the goal on the action client
                self._as.set_preempted()  # Set the action server to preempted
                self._feedback.stalled = True
                return  # Exit the execute_cb method

            # Update feedback
            self._feedback.reached_goal = False
            if hasattr(self, '_joint_state') and self._joint_state.actual.positions:
                self._feedback.position = 0.14 - 0.14 / 0.76 * self._joint_state.actual.positions[0]
            else:
                self._feedback.position = 0.0  # Default value if joint state is unavailable

            # Publish the feedback
            self._as.publish_feedback(self._feedback)
            r.sleep()

        self._jt_client.wait_for_result()

        # result handling
        result = self._jt_client.get_result()

        if result is None:
            rospy.logerr('%s: No result returned from action client', self._action_name)
            self._result.reached_goal = False
            self._as.set_aborted(self._result)
            return

        if result.error_code != 0:
            success = False

        if hasattr(self, '_joint_state') and self._joint_state.actual.positions:
            self._result.position = 0.14 - 0.14 / 0.76 * self._joint_state.actual.positions[0]
        else:
            self._result.position = 0.0  # Default value if joint state is unavailable

        if success:
            self._result.reached_goal = True
            rospy.loginfo('%s: Succeeded', self._action_name)
            self._as.set_succeeded(self._result)
        elif result.error_code == control_msgs.msg.FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED:
            self._result.reached_goal = False
            rospy.loginfo('%s: Gripper stalled (this is okay when grasping an object)', self._action_name)
            self._as.set_succeeded(self._result)
        else:
            self._result.reached_goal = False
            error_msg = error_code_to_string(result.error_code)
            rospy.logerr('%s: Failed with error code %s', self._action_name, error_msg)
            self._as.set_aborted(self._result)


def error_code_to_string(error_code):
    from control_msgs.msg import FollowJointTrajectoryResult as Res

    error_codes = {
        Res.SUCCESSFUL: 'SUCCESSFUL',
        Res.INVALID_GOAL: 'INVALID_GOAL',
        Res.INVALID_JOINTS: 'INVALID_JOINTS',
        Res.OLD_HEADER_TIMESTAMP: 'OLD_HEADER_TIMESTAMP',
        Res.PATH_TOLERANCE_VIOLATED: 'PATH_TOLERANCE_VIOLATED',
        Res.GOAL_TOLERANCE_VIOLATED: 'GOAL_TOLERANCE_VIOLATED',
    }

    return error_codes[error_code]


if __name__ == '__main__':
    rospy.init_node('gripper_command_bridge')

    param_path = rospy.search_param('tf_prefix')
    if param_path is not None:
        tf_prefix = rospy.get_param(param_path)
    else:
        tf_prefix = 'mobipick'

    #  ensure tf_prefix_ ends with exactly 1 '/' if nonempty, or "" if empty
    tf_prefix = tf_prefix.rstrip('/') + '/'
    if len(tf_prefix) == 1:
        tf_prefix = ""

    server = GripperBridgeAction(rospy.get_name(), tf_prefix)
    rospy.spin()
