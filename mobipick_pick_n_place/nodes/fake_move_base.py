#! /usr/bin/env python3

import rospy

import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseActionResult


class FakeMoveBaseNode(object):
    def __init__(self):
        self._as = actionlib.SimpleActionServer(
            'move_base', MoveBaseAction, execute_cb=self.execute_cb, auto_start=False
        )
        self._as.start()
        rospy.loginfo('%s: Initialized' % rospy.get_name())

    def execute_cb(self, _):
        rospy.loginfo('%s: Succeeded' % rospy.get_name())
        self._as.set_succeeded(MoveBaseActionResult())


if __name__ == '__main__':
    rospy.init_node('fake_move_base')
    node = FakeMoveBaseNode()
    rospy.spin()
