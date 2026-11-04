# The single robot selector of the sim GUI: one combo box in the one rqt window; the chosen robot is published
# latched as std_msgs/String on /selected_robot (the only place a robot is chosen: every GUI reads that topic)
# and applied to the third-party panels of the same window (steering topic, joint trajectory controller manager).
import re

import rosgraph
import rospy
from python_qt_binding import QtCore, QtGui, QtWidgets
from rqt_gui_py.plugin import Plugin
from std_msgs.msg import String

from mobipick_gazebo.robot_colours import colour_icon, robot_label

SELECTED_ROBOT_TOPIC = "/selected_robot"
DEFAULT_ROBOT = "mobipick"
_NAV_STATUS = re.compile(r"^/([^/]+)/move_base/status$")


def normalize(namespace):
    return (namespace or "").strip("/") or DEFAULT_ROBOT


def _number(namespace):
    match = re.search(r"(\d+)$", namespace)
    return int(match.group(1)) if match else 1


def robots_from_topics(topic_names, current=DEFAULT_ROBOT):
    """robot namespaces with a running move_base (from the published topic names), natural order, plus `current`"""
    robots = {normalize(current)}
    for topic in topic_names:
        match = _NAV_STATUS.match(topic)
        if match:
            robots.add(match.group(1))
    return sorted(robots, key=lambda r: (_number(r), r))


def published_robot(timeout=0.7):
    """the robot another selector already published (the topic is latched), None when nobody did: a second window or
    a restarted panel must not overwrite the choice with its own default"""
    try:
        return normalize(rospy.wait_for_message(SELECTED_ROBOT_TOPIC, String, timeout=timeout).data)
    except rospy.ROSException:
        return None


def description_key(key, namespace, has_param):
    """where the joint trajectory panel finds the robot model: the global key when it exists, else the chosen robot's"""
    if key != "robot_description" or has_param(key):
        return key
    return f"/{normalize(namespace)}/robot_description"


_selected = [DEFAULT_ROBOT]


def _read_joint_limits_of_the_selected_robot():
    """the joint trajectory panel reads the joint limits from the global robot_description, which a namespaced robot
    does not have (its model is /<ns>/robot_description): without it the panel's controller list stays empty"""
    try:
        from rqt_joint_trajectory_controller import joint_trajectory_controller as panel
    except ImportError:
        return
    original = panel.get_joint_limits
    if getattr(original, "follows_selected_robot", False):
        return

    def get_joint_limits(key="robot_description", *args, **kwargs):
        if key != "robot_description" or rospy.has_param(key):
            return original(key, *args, **kwargs)
        # the panel reads the limits once and keeps them for every robot it is pointed at later: take the joints of
        # every robot model in the sim (their names carry the robot namespace)
        limits = {}
        for name in sorted(rospy.get_param_names()):
            if re.fullmatch(r"/[^/]+/robot_description", name):
                limits.update(original(name, *args, **kwargs))
        return limits or original(description_key(key, _selected[0], rospy.has_param), *args, **kwargs)

    get_joint_limits.follows_selected_robot = True
    panel.get_joint_limits = get_joint_limits


def apply_to_panels(namespace, widgets):
    """point the third-party panels among `widgets` at the robot: the steering topic line and the joint trajectory
    controller's controller manager (when that manager is already listed). True when every panel found follows."""
    namespace = normalize(namespace)
    done = True
    for widget in widgets:
        name = widget.objectName()
        if name == "topic_line_edit" and isinstance(widget, QtWidgets.QLineEdit):
            topic = f"/{namespace}/cmd_vel"
            if widget.text() != topic:
                widget.setText(topic)
        elif name == "cm_combo" and isinstance(widget, QtWidgets.QComboBox):
            wanted = f"/{namespace}/controller_manager"
            index = widget.findText(wanted)
            if index < 0:
                done = False
            elif widget.property("selector_applied") != wanted or widget.currentIndex() != index:
                if widget.currentIndex() != index:
                    widget.setCurrentIndex(index)  # reports the manager to the panel
                else:
                    # the panel fills this combo with its signals blocked, so a manager that is already shown was
                    # never reported to it and its controller list stays empty: report it
                    widget.currentIndexChanged[str].emit(wanted)
                widget.setProperty("selector_applied", wanted)
    return done


class RobotSelectorPlugin(Plugin):
    """The one robot selector. Setting `robot` is the robot chosen last (default mobipick)."""

    def __init__(self, context):
        super().__init__(context)
        self.setObjectName("RobotSelector")
        self._robot = DEFAULT_ROBOT
        self._adopted = published_robot()  # before this selector publishes anything itself
        _read_joint_limits_of_the_selected_robot()
        self._widget = QtWidgets.QWidget()
        self._widget.setObjectName("RobotSelectorUi")
        self._widget.setWindowTitle("Robot")
        row = QtWidgets.QHBoxLayout(self._widget)
        row.addWidget(QtWidgets.QLabel("Robot:"))
        self._combo = QtWidgets.QComboBox()
        self._combo.setSizeAdjustPolicy(QtWidgets.QComboBox.AdjustToContents)
        self._combo.setMinimumContentsLength(16)  # 'orange Mobipick' in full
        self._combo.setToolTip("the robot every panel of this window and the RQT tables demo act on")
        self._combo.activated.connect(lambda _: self.select(self._combo.currentData()))
        row.addWidget(self._combo, 1)
        self._publisher = rospy.Publisher(SELECTED_ROBOT_TOPIC, String, queue_size=1, latch=True)
        self._timer = QtCore.QTimer(self._widget)
        self._timer.timeout.connect(self._refresh)
        self._timer.start(3000)
        context.add_widget(self._widget)
        self._refresh()
        self.select(self._adopted or self._robot)

    def _refresh(self):
        """the robot list follows the running robots; panels not yet ready (controller manager list) are retried"""
        try:
            topics = [name for name, _ in rosgraph.Master("/robot_selector").getPublishedTopics("/")]
        except Exception as exc:  # no master answer: keep the list
            rospy.logwarn_throttle(30, f"could not list the robots: {exc}")
            topics = []
        robots = robots_from_topics(topics, self._robot)
        if robots != [self._combo.itemData(i) for i in range(self._combo.count())]:
            self._combo.clear()
            for namespace in robots:  # the colour name people see; the namespace stays the item data
                icon = colour_icon(namespace) if len(robots) > 1 else None  # one robot: no colour dot
                self._combo.addItem(icon or QtGui.QIcon(), robot_label(namespace, len(robots)), namespace)
        self._combo.setCurrentIndex(max(self._combo.findData(self._robot), 0))
        apply_to_panels(self._robot, QtWidgets.QApplication.allWidgets())

    def select(self, namespace):
        self._robot = normalize(namespace)
        _selected[0] = self._robot
        self._combo.setCurrentIndex(max(self._combo.findData(self._robot), 0))
        self._publisher.publish(String(data=self._robot))
        apply_to_panels(self._robot, QtWidgets.QApplication.allWidgets())

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value("robot", self._robot)

    def restore_settings(self, plugin_settings, instance_settings):
        self.select(self._adopted or str(instance_settings.value("robot") or DEFAULT_ROBOT))

    def shutdown_plugin(self):
        self._timer.stop()
        self._publisher.unregister()
