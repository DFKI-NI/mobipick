#!/usr/bin/env python3
# The one owner of the selected robot in the multi-robot sim (see mobipick_gazebo/selected_robot.py): the only
# publisher of the latched /selected_robot. Switches ask on /selected_robot/request (std_msgs/String); a robot that
# does not run is refused with a logged reason and the current choice is sent again. No Qt: it runs headless too.
import rosgraph
import rospy
from std_msgs.msg import String

from mobipick_gazebo.selected_robot import REQUEST_TOPIC, SELECTED_ROBOT_TOPIC, SelectedRobotOwner


def main() -> None:
    rospy.init_node("selected_robot_owner")
    publisher = rospy.Publisher(SELECTED_ROBOT_TOPIC, String, queue_size=1, latch=True)
    master = rosgraph.Master(rospy.get_name())

    def topic_names():
        try:
            return [name for name, _ in master.getPublishedTopics("/")]
        except Exception as exc:  # no master answer: only the current robot counts as running
            rospy.logwarn(f"selected robot: could not list the robots: {exc}")
            return []

    owner = SelectedRobotOwner(lambda robot: publisher.publish(String(data=robot)), topic_names, rospy.logwarn,
                               rospy.get_param("~initial_robot", ""))
    rospy.loginfo(f"selected robot: {owner.robot}")
    rospy.Subscriber(REQUEST_TOPIC, String, lambda msg: owner.request(msg.data), queue_size=10)
    rospy.spin()


if __name__ == "__main__":
    main()
