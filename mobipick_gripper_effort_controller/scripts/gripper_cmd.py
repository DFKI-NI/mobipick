#!/usr/bin/env python3
"""Send one control_msgs/GripperCommand goal and print every action transition with timestamps.

Usage: rosrun mobipick_gripper_effort_controller gripper_cmd.py <gap_m> [max_effort] [_action:=/mobipick/gripper_hw]
Example: rosrun mobipick_gripper_effort_controller gripper_cmd.py 0.0 50

Like the real Robotiq action server, the controller only accepts gaps in -0.015..0.14 m and
max_effort in 30..100; max_effort defaults to 100 here.
"""
import sys
import time

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from control_msgs.msg import GripperCommandAction, GripperCommandGoal


def stamp():
    return '[wall %.3f | sim %.3f]' % (time.time(), rospy.Time.now().to_sec())


def main():
    rospy.init_node('gripper_cmd', anonymous=True)
    args = rospy.myargv(sys.argv)
    if len(args) < 2:
        print(__doc__)
        return 1
    gap = float(args[1])
    effort = float(args[2]) if len(args) > 2 else 100.0
    action = rospy.get_param('~action', '/mobipick/gripper_hw')
    timeout = rospy.get_param('~timeout', 15.0)

    client = actionlib.SimpleActionClient(action, GripperCommandAction)
    print('%s waiting for %s' % (stamp(), action))
    if not client.wait_for_server(rospy.Duration(5.0)):
        print('%s server %s not available' % (stamp(), action))
        return 2

    goal = GripperCommandGoal()
    goal.command.position = gap
    goal.command.max_effort = effort

    def active_cb():
        print('%s goal ACTIVE' % stamp())

    def feedback_cb(fb):
        print('%s feedback: gap %.4f m effort %.3f stalled %s reached %s'
              % (stamp(), fb.position, fb.effort, fb.stalled, fb.reached_goal))

    def done_cb(state, result):
        print('%s DONE state=%s (%d) result: gap %.4f m effort %.3f stalled %s reached %s'
              % (stamp(), client.get_goal_status_text(), state, result.position, result.effort,
                 result.stalled, result.reached_goal))

    print('%s sending gap %.4f m, max_effort %.1f' % (stamp(), gap, effort))
    client.send_goal(goal, done_cb=done_cb, active_cb=active_cb, feedback_cb=feedback_cb)
    finished = client.wait_for_result(rospy.Duration(timeout))
    if not finished:
        print('%s no result after %.1f s, client state %d; cancelling'
              % (stamp(), timeout, client.get_state()))
        client.cancel_goal()
        client.wait_for_result(rospy.Duration(3.0))
        print('%s state after cancel: %d (%s)' % (stamp(), client.get_state(), client.get_goal_status_text()))
        return 3
    return 0 if client.get_state() == GoalStatus.SUCCEEDED else 4


if __name__ == '__main__':
    sys.exit(main())
