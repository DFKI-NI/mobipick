#!/usr/bin/env python3

"""
This node plots the gripper joint angle vs. the opening gap.
"""

from collections import defaultdict
from math import sin, asin

import rospy
import tf2_ros
from sensor_msgs.msg import JointState

NUM_BUCKETS = 100
MIN_ANGLE = 0.0
MAX_ANGLE = 0.755

buckets = defaultdict(float)

ARC_RADIUS = 0.10214673  # approximate distance from gripper_left_inner_knuckle to gripper_left_robotiq_fingertip_65mm


def angle_to_idx(angle):
    return int(round((angle - MIN_ANGLE) * NUM_BUCKETS / (MAX_ANGLE - MIN_ANGLE)))


def calc_opening_gap(angle):
    return 2.0 * ARC_RADIUS * sin(MAX_ANGLE - angle)


def calc_angle(opening_gap):
    return MAX_ANGLE - asin(opening_gap / (2.0 * ARC_RADIUS))


def cb(msg):
    """
    @type msg: JointState
    """
    try:
        trans = tf_buffer.lookup_transform(
            'mobipick/gripper_right_robotiq_fingertip_65mm',
            'mobipick/gripper_left_robotiq_fingertip_65mm',
            msg.header.stamp,
            rospy.Duration(1),
        )
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        raise

    opening_gap = -trans.transform.translation.z - 0.014
    try:
        angle = msg.position[msg.name.index('mobipick/gripper_finger_joint')]
        buckets[angle_to_idx(angle)] = (angle, opening_gap, calc_opening_gap(angle))
    except ValueError:
        pass


if __name__ == '__main__':
    rospy.init_node('gripper_plotter')
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rospy.Subscriber('joint_states', JointState, cb, queue_size=10)
    rospy.spin()
    for bucket in sorted(buckets.values()):
        print(bucket[0], bucket[1], bucket[2])
