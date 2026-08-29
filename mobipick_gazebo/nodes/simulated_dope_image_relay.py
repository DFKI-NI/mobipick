#!/usr/bin/env python3

"""Publish simulated camera images only while pose perception is active."""

import threading

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool


class SimulatedDopeImageRelay:
    def __init__(self):
        self._lock = threading.Lock()
        self._active = False
        self._image_subscriber = None

        self._image_input_topic = rospy.get_param(
            "~image_input_topic", "/mobipick/eef_main_cam/rgb/image_raw"
        )
        image_output_topic = rospy.get_param(
            "~image_output_topic", "/mobipick/dope/rgb_points"
        )
        perception_active_topic = rospy.get_param(
            "~perception_active_topic", "/pick_pose_selector_node/perception_active"
        )

        self._image_publisher = rospy.Publisher(image_output_topic, Image, queue_size=1)
        self._active_subscriber = rospy.Subscriber(
            perception_active_topic, Bool, self._perception_active_callback, queue_size=1
        )

        rospy.loginfo(
            "Simulated DOPE image relay waiting for perception activity on %s; "
            "images will be relayed from %s to %s",
            perception_active_topic,
            self._image_input_topic,
            image_output_topic,
        )

    def _perception_active_callback(self, message):
        with self._lock:
            if message.data == self._active:
                return

            self._active = message.data
            image_subscriber = self._image_subscriber
            self._image_subscriber = None

        if image_subscriber is not None:
            image_subscriber.unregister()

        if not message.data:
            rospy.loginfo("Simulated DOPE image relay deactivated")
            return

        new_subscriber = rospy.Subscriber(
            self._image_input_topic, Image, self._image_callback, queue_size=1
        )
        with self._lock:
            if self._active:
                self._image_subscriber = new_subscriber
                new_subscriber = None

        if new_subscriber is not None:
            new_subscriber.unregister()
            return

        rospy.loginfo("Simulated DOPE image relay activated")

    def _image_callback(self, message):
        # Hold the lock through publication so that no callback can publish
        # after a deactivation callback has completed.
        with self._lock:
            if self._active:
                self._image_publisher.publish(message)


if __name__ == "__main__":
    rospy.init_node("simulated_dope_image_relay")
    SimulatedDopeImageRelay()
    rospy.spin()
