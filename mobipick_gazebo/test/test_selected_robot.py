import os
import sys
import threading
import unittest
from unittest import mock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src"))

from mobipick_gazebo import selected_robot  # noqa: E402

RUNNING = ["/mobipick/move_base/status", "/mobipick2/move_base/status", "/mobipick2/scan"]


class OwnerTest(unittest.TestCase):
    def owner(self, topics=RUNNING, initial="mobipick"):
        self.sent, self.warned = [], []
        return selected_robot.SelectedRobotOwner(self.sent.append, lambda: topics, self.warned.append, initial)

    def test_starts_on_the_initial_robot_and_publishes_it(self):
        self.owner(initial="/mobipick/")
        self.assertEqual(self.sent, ["mobipick"])
        self.owner(initial="")
        self.assertEqual(self.sent, [selected_robot.DEFAULT_ROBOT])

    def test_a_running_robot_is_accepted(self):
        owner = self.owner()
        self.assertEqual(owner.request("/mobipick2/"), "mobipick2")
        self.assertEqual(self.sent, ["mobipick", "mobipick2"])
        self.assertEqual(self.warned, [])

    def test_a_robot_that_does_not_run_is_refused_with_a_reason_and_the_choice_sent_again(self):
        owner = self.owner()
        self.assertEqual(owner.request("mobipick3"), "mobipick")
        self.assertEqual(self.sent, ["mobipick", "mobipick"])  # the asking switch snaps back
        self.assertEqual(len(self.warned), 1)
        self.assertIn("mobipick3 is not running", self.warned[0])
        self.assertIn("still mobipick", self.warned[0])

    def test_an_empty_request_is_refused(self):
        owner = self.owner()
        owner.request("mobipick2")
        self.assertEqual(owner.request("/"), "mobipick2")
        self.assertIn("empty request", self.warned[0])
        self.assertEqual(self.sent[-1], "mobipick2")

    def test_two_quick_requests_end_on_the_last(self):
        owner = self.owner(topics=RUNNING + ["/mobipick3/move_base/status"])
        owner.request("mobipick2")
        owner.request("mobipick3")
        self.assertEqual(owner.robot, "mobipick3")
        self.assertEqual(self.sent[-1], "mobipick3")

    def test_two_requests_at_the_same_time_publish_the_stored_choice_last(self):
        # request A (mobipick2) is held inside its publish while request B (mobipick3) comes in from another thread;
        # without a lock B publishes first and A's late publish leaves /selected_robot on mobipick2 while the owner
        # holds mobipick3
        sent, warned = [], []
        a_publishing, release_a = threading.Event(), threading.Event()

        def publish(robot):
            if robot == "mobipick2" and not release_a.is_set():
                a_publishing.set()
                release_a.wait(5)
            sent.append(robot)

        owner = selected_robot.SelectedRobotOwner(publish, lambda: RUNNING + ["/mobipick3/move_base/status"],
                                                  warned.append, "mobipick")
        a = threading.Thread(target=owner.request, args=("mobipick2",))
        b = threading.Thread(target=owner.request, args=("mobipick3",))
        a.start()
        self.assertTrue(a_publishing.wait(5))
        b.start()
        b.join(0.3)
        self.assertTrue(b.is_alive(), "request B ran while request A was still publishing")
        release_a.set()
        a.join(5)
        b.join(5)
        self.assertEqual(sent, ["mobipick", "mobipick2", "mobipick3"])
        self.assertEqual(owner.robot, sent[-1])  # a late subscriber gets the latched last message: the stored choice
        self.assertEqual(warned, [])

    def test_a_refused_request_at_the_same_time_snaps_back_to_the_stored_choice(self):
        sent, warned = [], []
        a_publishing, release_a = threading.Event(), threading.Event()

        def publish(robot):
            if robot == "mobipick2" and not release_a.is_set():
                a_publishing.set()
                release_a.wait(5)
            sent.append(robot)

        owner = selected_robot.SelectedRobotOwner(publish, lambda: RUNNING, warned.append, "mobipick")
        a = threading.Thread(target=owner.request, args=("mobipick2",))
        b = threading.Thread(target=owner.request, args=("mobipick7",))
        a.start()
        self.assertTrue(a_publishing.wait(5))
        b.start()
        b.join(0.3)
        self.assertTrue(b.is_alive())
        release_a.set()
        a.join(5)
        b.join(5)
        self.assertEqual(sent, ["mobipick", "mobipick2", "mobipick2"])
        self.assertEqual(owner.robot, "mobipick2")
        self.assertEqual(len(warned), 1)
        self.assertIn("still mobipick2", warned[0])

    def test_the_owner_is_found_by_its_request_subscriber(self):
        state = ([], [[selected_robot.REQUEST_TOPIC, ["/selected_robot_owner"]], ["/selected_robot", ["/a"]]], [])
        self.assertTrue(selected_robot.owner_running(state))
        self.assertFalse(selected_robot.owner_running(([], [["/selected_robot", ["/a"]]], [])))
        self.assertFalse(selected_robot.owner_running(([], [[selected_robot.REQUEST_TOPIC, []]], [])))


class FakeMaster:
    owner = True
    topics = RUNNING

    def __init__(self, _caller_id):
        pass

    def getPublishedTopics(self, _subgraph):
        return [[name, "x"] for name in self.topics]

    def getSystemState(self):
        subscribers = [[selected_robot.REQUEST_TOPIC, ["/selected_robot_owner"]]] if FakeMaster.owner else []
        return [], subscribers, []


class FakePublisher:
    made = []

    def __init__(self, topic, _type, queue_size=None, latch=False):
        self.topic, self.latch, self.sent, self.unregistered = topic, latch, [], False
        FakePublisher.made.append(self)

    def publish(self, msg):
        self.sent.append(msg.data)

    def unregister(self):
        self.unregistered = True


class SelectorClientTest(unittest.TestCase):
    """the sim robot selector asks the owner and follows /selected_robot; alone it publishes as before"""

    @classmethod
    def setUpClass(cls):
        try:
            from python_qt_binding import QtCore, QtWidgets
            from mobipick_gazebo import robot_selector
        except ImportError as exc:
            raise unittest.SkipTest(f"no Qt/ROS here: {exc}")
        cls.qt, cls.sel = QtWidgets, robot_selector

        class Context(QtCore.QObject):  # the plugin context rqt hands a plugin
            def add_widget(self, widget):
                self.widget = widget

        cls.context = Context
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)

    def make(self, owner, latched=None):
        FakeMaster.owner = owner
        FakePublisher.made = []
        self.subscribed = []
        sel = self.sel

        def wait_for_message(*_args, **_kwargs):
            if latched is None:
                raise sel.rospy.ROSException("nobody published")
            return mock.Mock(data=latched)

        patches = [mock.patch.object(sel.rosgraph, "Master", FakeMaster),
                   mock.patch.object(sel.rospy, "Publisher", FakePublisher),
                   mock.patch.object(sel.rospy, "Subscriber",
                                     lambda topic, _t, callback: self.subscribed.append((topic, callback)) or mock.Mock()),
                   mock.patch.object(sel.rospy, "wait_for_message", wait_for_message),
                   mock.patch.object(sel, "colour_icon", lambda _ns: None)]
        for patch in patches:
            patch.start()
            self.addCleanup(patch.stop)
        self.steering = self.qt.QLineEdit()
        self.steering.setObjectName("topic_line_edit")
        self.addCleanup(self.steering.deleteLater)
        self.ctx = self.context()
        plugin = sel.RobotSelectorPlugin(self.ctx)
        self.addCleanup(plugin.shutdown_plugin)
        return plugin

    def publishers(self, topic):
        return [p for p in FakePublisher.made if p.topic == topic]

    def test_with_an_owner_a_pick_is_a_request_and_the_window_never_writes_the_selected_robot(self):
        plugin = self.make(owner=True, latched="mobipick")
        plugin.select("mobipick2")
        self.assertEqual(self.publishers(self.sel.REQUEST_TOPIC)[0].sent, ["mobipick2"])
        self.assertEqual(self.publishers(self.sel.SELECTED_ROBOT_TOPIC), [])
        self.assertEqual(plugin._combo.currentData(), "mobipick")  # until the owner answers
        self.assertEqual(self.steering.text(), "/mobipick/cmd_vel")

    def test_a_new_window_with_an_owner_does_not_ask_for_its_default(self):
        self.make(owner=True, latched=None)  # the latched answer is late
        self.assertEqual(self.publishers(self.sel.REQUEST_TOPIC)[0].sent, [])
        self.assertEqual(self.publishers(self.sel.SELECTED_ROBOT_TOPIC), [])

    def test_a_change_made_elsewhere_moves_the_combo_and_the_panels(self):
        plugin = self.make(owner=True, latched="mobipick")
        topic, callback = self.subscribed[0]
        self.assertEqual(topic, self.sel.SELECTED_ROBOT_TOPIC)
        callback(mock.Mock(data="mobipick2"))
        self.app.processEvents()
        self.assertEqual(plugin._combo.currentData(), "mobipick2")
        self.assertEqual(self.steering.text(), "/mobipick2/cmd_vel")

    def test_without_an_owner_the_window_publishes_the_robot_itself(self):
        plugin = self.make(owner=False)
        plugin.select("mobipick2")
        latched = self.publishers(self.sel.SELECTED_ROBOT_TOPIC)
        self.assertEqual(len(latched), 1)
        self.assertTrue(latched[0].latch)
        self.assertEqual(latched[0].sent, ["mobipick", "mobipick2"])
        self.assertEqual(plugin._combo.currentData(), "mobipick2")
        self.assertEqual(self.steering.text(), "/mobipick2/cmd_vel")

    def test_without_an_owner_a_choice_made_elsewhere_is_published_again_once(self):
        self.make(owner=False)
        _topic, callback = self.subscribed[0]
        callback(mock.Mock(data="mobipick2"))  # another GUI chose robot 2
        self.app.processEvents()
        callback(mock.Mock(data="mobipick2"))  # its echo comes back
        self.app.processEvents()
        self.assertEqual(self.publishers(self.sel.SELECTED_ROBOT_TOPIC)[0].sent, ["mobipick", "mobipick2"])

    def test_with_an_owner_a_choice_made_elsewhere_is_only_followed(self):
        self.make(owner=True, latched="mobipick")
        _topic, callback = self.subscribed[0]
        callback(mock.Mock(data="mobipick2"))
        self.app.processEvents()
        self.assertEqual(self.publishers(self.sel.SELECTED_ROBOT_TOPIC), [])
        self.assertEqual(self.publishers(self.sel.REQUEST_TOPIC)[0].sent, [])

    def test_an_owner_that_comes_up_later_becomes_the_only_writer(self):
        plugin = self.make(owner=False)
        FakeMaster.owner = True
        plugin.select("mobipick2")
        self.assertTrue(self.publishers(self.sel.SELECTED_ROBOT_TOPIC)[0].unregistered)
        self.assertEqual(self.publishers(self.sel.REQUEST_TOPIC)[0].sent, ["mobipick2"])


if __name__ == "__main__":
    unittest.main()
