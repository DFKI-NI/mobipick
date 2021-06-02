#!/usr/bin/python

import rospy
import robotiq_2f_gripper_control.msg as gripper_msg
from sensor_msgs.msg import JointState


class GripperJointPublisher:
    def __init__(self, prefix):
        self.prefix = prefix
        self.pub = rospy.Publisher('joint_states', JointState, queue_size=10)
        self.sub = rospy.Subscriber('input', gripper_msg.Robotiq2FGripper_robot_input, self.callback)

    def callback(self, data):
        rospy.loginfo('gripper in position: %i' % data.gPO)
        state = JointState()
        #        state.header.stamp = rospy.Time.now()
        state.name = [self.prefix + 'gripper_finger_joint']
        # gPO = 0 when open, 255 when closed
        # srdf: gripper_finger_joint = 0 when open, 0.755 when closed
        # robotiq manual: quasi-linear relationship between gPO and joint state
        state.position = [float(data.gPO) / 255.0 * 0.755]
        state.velocity = [0.0]
        state.effort = [0.0]

        self.pub.publish(state)


if __name__ == '__main__':
    rospy.init_node('robotiq_2f_140_joint_state_publisher')
    prefix = rospy.get_param('~prefix', '')

    gj = GripperJointPublisher(prefix)

    rospy.spin()
