#! /usr/bin/env python3

import rospy

import actionlib

from control_msgs.msg import GripperCommandAction, GripperCommandActionResult


class FakeGripperCommandNode(object):
    def __init__(self):
        self._as = actionlib.SimpleActionServer(
            'gripper_hw', GripperCommandAction, execute_cb=self.execute_cb, auto_start=False
        )
        self._as.start()
        rospy.loginfo('%s: Initialized' % rospy.get_name())

    def execute_cb(self, _):
        rospy.loginfo('%s: Succeeded' % rospy.get_name())
        self._as.set_succeeded(GripperCommandActionResult())


if __name__ == '__main__':
    rospy.init_node('fake_gripper_hw')
    node = FakeGripperCommandNode()
    rospy.spin()
