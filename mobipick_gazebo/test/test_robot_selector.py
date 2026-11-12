import json
import os
import sys
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "nodes"))

import make_sim_gui_perspective as perspective  # noqa: E402


class PerspectiveTest(unittest.TestCase):
    def setUp(self):
        with open(os.path.join(HERE, "..", "config", "sim_gui.perspective")) as handle:
            self.template = json.load(handle)

    def test_one_panel_of_each_kind_for_any_robot_count(self):
        for count in (1, 2, 3):
            out = perspective.build(self.template, count)
            running = eval(out["groups"]["pluginmanager"]["keys"]["running-plugins"]["repr"])  # noqa: S307
            self.assertEqual(running, {"rqt_robot_steering/RobotSteering": [1],
                                       "rqt_joint_trajectory_controller/JointTrajectoryController": [1],
                                       "mobipick_gazebo/SimulatedUrDashboard": [1],
                                       "mobipick_gazebo/RobotSelector": [1]})

    def test_the_e_stop_panel_knows_every_robot_and_the_panels_start_on_robot_one(self):
        groups = perspective.build(self.template, 3)["groups"]["pluginmanager"]["groups"]
        settings = {name: {key: eval(value["repr"]) for key, value in group["groups"]["plugin"]["keys"].items()}
                    for name, group in groups.items()}  # noqa: S307
        self.assertEqual(settings["plugin__mobipick_gazebo__SimulatedUrDashboard__1"],
                         {"robots": "mobipick,mobipick2,mobipick3"})
        self.assertEqual(settings["plugin__rqt_robot_steering__RobotSteering__1"], {"topic": "/mobipick/cmd_vel"})
        self.assertEqual(settings["plugin__mobipick_gazebo__RobotSelector__1"], {"robot": "mobipick"})
        self.assertIn("dock_widget__RobotSelectorUi", groups["plugin__mobipick_gazebo__RobotSelector__1"]["groups"])

    def test_the_saved_perspective_files_are_what_the_generator_writes(self):
        for count in (2, 3):
            with open(os.path.join(HERE, "..", "config", f"sim_gui_x{count}.perspective")) as handle:
                self.assertEqual(json.load(handle), perspective.build(self.template, count))


class SelectionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            from python_qt_binding import QtWidgets
            sys.path.insert(0, os.path.join(HERE, "..", "src"))
            from mobipick_gazebo import robot_selector
        except ImportError as exc:
            raise unittest.SkipTest(f"no Qt/ROS here: {exc}")
        cls.qt, cls.sel = QtWidgets, robot_selector
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)

    def test_robots_are_the_ones_with_a_move_base_in_natural_order(self):
        topics = ["/mobipick10/move_base/status", "/mobipick2/move_base/status", "/mobipick/move_base/status",
                  "/mobipick2/scan", "/move_base/status"]
        self.assertEqual(self.sel.robots_from_topics(topics), ["mobipick", "mobipick2", "mobipick10"])
        self.assertEqual(self.sel.robots_from_topics([], "/mobipick2/"), ["mobipick2"])

    def test_a_robot_chosen_in_another_gui_is_followed_once(self):
        self.assertEqual(self.sel.to_follow("mobipick", "/mobipick2"), "mobipick2")
        self.assertIsNone(self.sel.to_follow("mobipick2", "mobipick2/"))  # its own message comes back: no echo loop
        self.assertEqual(self.sel.to_follow("mobipick2", ""), "mobipick")

    def test_the_steering_and_controller_panels_follow_the_selection(self):
        qt = self.qt
        topic = qt.QLineEdit()
        topic.setObjectName("topic_line_edit")
        managers = qt.QComboBox()
        managers.setObjectName("cm_combo")
        managers.addItems(["/mobipick/controller_manager", "/mobipick2/controller_manager"])
        self.assertTrue(self.sel.apply_to_panels("mobipick2", [topic, managers]))
        self.assertEqual(topic.text(), "/mobipick2/cmd_vel")
        self.assertEqual(managers.currentText(), "/mobipick2/controller_manager")

    def test_the_joint_panel_reads_the_model_of_the_chosen_robot_when_there_is_no_global_one(self):
        key = self.sel.description_key
        self.assertEqual(key("robot_description", "mobipick2", lambda _: False), "/mobipick2/robot_description")
        self.assertEqual(key("robot_description", "mobipick2", lambda _: True), "robot_description")
        self.assertEqual(key("/mobipick/robot_description", "mobipick2", lambda _: False),
                         "/mobipick/robot_description")

    def test_a_manager_already_shown_is_still_reported_to_the_panel_once(self):
        managers = self.qt.QComboBox()
        managers.setObjectName("cm_combo")
        managers.blockSignals(True)  # the panel fills its combo like this
        managers.addItems(["/mobipick/controller_manager", "/mobipick2/controller_manager"])
        managers.setCurrentIndex(1)
        managers.blockSignals(False)
        heard = []
        managers.currentIndexChanged[str].connect(heard.append)
        self.sel.apply_to_panels("mobipick2", [managers])
        self.sel.apply_to_panels("mobipick2", [managers])
        self.assertEqual(heard, ["/mobipick2/controller_manager"])

    def test_a_controller_manager_not_yet_listed_is_retried(self):
        managers = self.qt.QComboBox()
        managers.setObjectName("cm_combo")
        self.assertFalse(self.sel.apply_to_panels("mobipick3", [managers]))
        managers.addItem("/mobipick3/controller_manager")
        self.assertTrue(self.sel.apply_to_panels("mobipick3", [managers]))
        self.assertEqual(managers.currentText(), "/mobipick3/controller_manager")


if __name__ == "__main__":
    unittest.main()
