# The single robot selector of the sim GUI: one combo box in the one rqt window. The chosen robot lives on the latched
# std_msgs/String /selected_robot (every GUI reads that topic); its one writer is the owner node
# (nodes/selected_robot.py), which this combo asks on /selected_robot/request. The combo, the steering topic and the
# joint trajectory controller manager of the same window follow /selected_robot, also when another switch changed it.
# Without an owner node (an older launch) the selector publishes /selected_robot itself and publishes a choice made
# in another GUI again, so every latched message names the same robot.
import re

import rosgraph
import rospy
from python_qt_binding import QtCore, QtGui, QtWidgets
from rqt_gui_py.plugin import Plugin
from std_msgs.msg import String

from mobipick_gazebo.robot_colours import colour_icon, robot_label
from mobipick_gazebo.selected_robot import (DEFAULT_ROBOT, REQUEST_TOPIC, SELECTED_ROBOT_TOPIC,  # noqa: F401
                                            normalize, owner_running, robots_from_topics)


def published_robot(timeout=0.7):
    """the robot another selector already published (the topic is latched), None when nobody did: a second window or
    a restarted panel must not overwrite the choice with its own default"""
    try:
        return normalize(rospy.wait_for_message(SELECTED_ROBOT_TOPIC, String, timeout=timeout).data)
    except rospy.ROSException:
        return None


def to_follow(current, heard):
    """the robot to switch to when /selected_robot carries `heard`, None when it is the robot already chosen. Without
    an owner node every GUI that publishes the topic adopts a choice made elsewhere and publishes it again, so the
    latched message of every publisher names the same robot and a window opened later reads that robot from all of
    them."""
    heard = normalize(heard)
    return None if heard == normalize(current) else heard


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
    """The one robot selector. Setting `robot` is the robot chosen last (default mobipick); it is used only without an
    owner node, which otherwise decides the robot."""

    _robot_published = QtCore.pyqtSignal(str)  # /selected_robot arrives in a rospy thread

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
        self._master = rosgraph.Master("/robot_selector")
        self._publisher = None  # /selected_robot, only while no owner node runs
        self._requests = rospy.Publisher(REQUEST_TOPIC, String, queue_size=1)
        self._robot_published.connect(self._follow)
        self._subscriber = rospy.Subscriber(SELECTED_ROBOT_TOPIC, String,
                                            lambda msg: self._robot_published.emit(msg.data))
        self._timer = QtCore.QTimer(self._widget)
        self._timer.timeout.connect(self._refresh)
        self._timer.start(3000)
        context.add_widget(self._widget)
        self._refresh()
        if self._adopted:
            self._follow(self._adopted)
        elif not self._owner_running():  # with an owner, its latched choice arrives on the subscriber
            self.select(self._robot)

    def _owner_running(self):
        """True when the owner node decides the robot; this window then stops writing /selected_robot itself"""
        try:
            running = owner_running(self._master.getSystemState())
        except Exception:  # no master answer: act alone, as without an owner
            return False
        if running and self._publisher is not None:  # the owner came up after this window: it is the only writer now
            self._publisher.unregister()
            self._publisher = None
        return running

    def _refresh(self):
        """the robot list follows the running robots; panels not yet ready (controller manager list) are retried"""
        try:
            topics = [name for name, _ in self._master.getPublishedTopics("/")]
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
        """a pick in this selector: ask the owner node; without one, publish the robot and follow it at once"""
        namespace = normalize(namespace)
        if self._owner_running():
            # the combo shows the asked robot only when /selected_robot says so (the owner may refuse it)
            self._combo.setCurrentIndex(max(self._combo.findData(self._robot), 0))
            self._requests.publish(String(data=namespace))
            return
        if self._publisher is None:
            self._publisher = rospy.Publisher(SELECTED_ROBOT_TOPIC, String, queue_size=1, latch=True)
        self._publisher.publish(String(data=namespace))
        self._robot = namespace  # chosen here: nothing to publish again when it comes back
        self._follow(namespace)

    def _follow(self, namespace):
        """show and apply the robot /selected_robot names, whoever chose it"""
        if self._publisher is not None and to_follow(self._robot, namespace) is not None and not self._owner_running():
            self._publisher.publish(String(data=normalize(namespace)))  # no owner: agree with the other writers
        self._robot = normalize(namespace)
        _selected[0] = self._robot
        if self._combo.findData(self._robot) < 0:
            self._refresh()  # a robot this list has not seen yet
        self._combo.setCurrentIndex(max(self._combo.findData(self._robot), 0))
        apply_to_panels(self._robot, QtWidgets.QApplication.allWidgets())

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value("robot", self._robot)

    def restore_settings(self, plugin_settings, instance_settings):
        if self._adopted:
            self._follow(self._adopted)
        elif not self._owner_running():
            self.select(str(instance_settings.value("robot") or DEFAULT_ROBOT))

    def shutdown_plugin(self):
        self._timer.stop()
        self._subscriber.unregister()
        self._requests.unregister()
        if self._publisher is not None:
            self._publisher.unregister()
