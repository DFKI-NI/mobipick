#! /usr/bin/env python

import rospy

import actionlib

import control_msgs.msg

import trajectory_msgs.msg

class GripperBridgeAction(object):
    # create messages that are used to publish feedback/result
    _feedback = control_msgs.msg.GripperCommandFeedback()
    _result = control_msgs.msg.GripperCommandResult()

    def __init__(self, name):
        self.joint_trajectory_client()
        rospy.loginfo("Client connected")

        self._controller_sub = rospy.Subscriber('gripper_controller/state', control_msgs.msg.JointTrajectoryControllerState, self.joint_state_cb, )
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, control_msgs.msg.GripperCommandAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        rospy.loginfo("Actionserver for gripper bridge started")

    def joint_state_cb(self, state):
        self._joint_state = state

    def joint_trajectory_client(self):
        self._jt_client = actionlib.SimpleActionClient('gripper_controller/follow_joint_trajectory', control_msgs.msg.FollowJointTrajectoryAction)
        self._jt_client.wait_for_server()
        self._actual_goal = control_msgs.msg.FollowJointTrajectoryGoal()


    def set_jt_goal(self, position, effort = 30.0):
        #self._actual_goal.header.stamp = rospy.Time.now()
        position = min(0.76, position)
        position = max(0.0, position)
        rospy.loginfo("Desired goal set to %.3f" % position)
        jt = trajectory_msgs.msg.JointTrajectory()
        jt_point = trajectory_msgs.msg.JointTrajectoryPoint()
        jt_point.positions.append(position)
        jt_point.time_from_start=rospy.Duration(2.0)
        jt_point.effort.append(effort)
        jt.points.append(jt_point)
        jt.joint_names.append('mobipick/gripper_finger_joint')
        self._actual_goal.trajectory = jt

      
    def execute_cb(self, goal):
        # helper variables
        rospy.loginfo("Recieved gripper command with position=%.3f and effort=%.3f" % (goal.command.position, goal.command.max_effort))
        r = rospy.Rate(10)
        success = True
        self.set_jt_goal(-0.76/0.14*goal.command.position+0.76, goal.command.max_effort)
        self._jt_client.send_goal(self._actual_goal)
        # start executing the action
        while not self._jt_client.get_state()==actionlib.SimpleGoalState.ACTIVE:
        # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                self._feedback.stalled = true
            self._feedback.reached_goal=False
            self._feedback.position = (0.14-0.14/0.76*self._joint_state.actual.positions[0])
            # publish the feedback
            self._as.publish_feedback(self._feedback)
            # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
            r.sleep()
        if success:
            self._result = self._jt_client.get_result()
            self._result.result.position=(0.14-0.14/0.76*self._joint_state.actual.positions[0])
            self._result.result.reached_goal=True
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)

        
if __name__ == '__main__':
    rospy.init_node('gripper_command_bridge')
    server = GripperBridgeAction(rospy.get_name())
    rospy.spin()