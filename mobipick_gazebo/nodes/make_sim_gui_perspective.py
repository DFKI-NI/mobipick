#!/usr/bin/env python3
"""Writes config/sim_gui_x<N>.perspective: the sim GUI of config/sim_gui.perspective for N robots in ONE rqt window
with ONE robot selector: one robot selector panel, one steering, one joint trajectory controller and one simulated
e-stop panel (that one serves the e-stop services of every robot and shows the selected robot). The panels start on
the first robot (`mobipick`; robot k is `mobipick<k>`); the selector chooses the robot for all of them.
Usage: make_sim_gui_perspective.py N [NAMESPACE]  (default namespace mobipick); stdlib only."""
import copy
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG = os.path.join(HERE, "..", "config")

PLUGINS = [  # (running-plugins key, group name prefix, dock widget group, title, instance settings of robot 1)
    ("rqt_robot_steering/RobotSteering", "plugin__rqt_robot_steering__RobotSteering__", "dock_widget__RobotSteeringUi",
     "Robot Steering", lambda names: {"topic": f"/{names[0]}/cmd_vel"}),
    ("rqt_joint_trajectory_controller/JointTrajectoryController",
     "plugin__rqt_joint_trajectory_controller__JointTrajectoryController__", "dock_widget__JointTrajectoryControllerUi",
     "Joint trajectory controller", lambda names: {"cm_ns": f"/{names[0]}/controller_manager"}),
    ("mobipick_gazebo/SimulatedUrDashboard", "plugin__mobipick_gazebo__SimulatedUrDashboard__",
     "dock_widget__SimulatedUrDashboardUi", "Simulated E-Stop", lambda names: {"robots": ",".join(names)}),
    ("mobipick_gazebo/RobotSelector", "plugin__mobipick_gazebo__RobotSelector__", "dock_widget__RobotSelectorUi",
     "Robot selector", lambda names: {"robot": names[0]}),
]


def robots(count, namespace="mobipick"):
    return [namespace] + [f"{namespace}{k}" for k in range(2, count + 1)]


def build(template, count, namespace="mobipick"):
    names = robots(count, namespace)
    out = copy.deepcopy(template)
    manager = out["groups"]["pluginmanager"]
    manager["keys"]["running-plugins"]["repr"] = repr({key: [1] for key, *_ in PLUGINS})
    dashboard_prefix = "plugin__mobipick_gazebo__SimulatedUrDashboard__"
    groups = {}
    for key, prefix, dock, title, settings in PLUGINS:
        source = template["groups"]["pluginmanager"]["groups"].get(prefix + "1")
        group = copy.deepcopy(source or template["groups"]["pluginmanager"]["groups"][dashboard_prefix + "1"])
        if source is None:  # the selector is new: its dock is the e-stop one under its own name
            group["groups"][dock] = group["groups"].pop("dock_widget__SimulatedUrDashboardUi")
        group["groups"][dock]["keys"]["dock_widget_title"]["repr"] = repr(title)
        group["groups"]["plugin"]["keys"] = {name: {"type": "repr", "repr": repr(value)}
                                             for name, value in settings(names).items()}
        groups[prefix + "1"] = group
    manager["groups"] = groups
    out["groups"]["mainwindow"]["keys"].pop("state", None)   # the saved dock layout does not know the selector
    return out


def main(argv):
    count = int(argv[1])
    namespace = argv[2] if len(argv) > 2 else "mobipick"
    with open(os.path.join(CONFIG, "sim_gui.perspective")) as handle:
        template = json.load(handle)
    path = os.path.join(CONFIG, f"sim_gui_x{count}.perspective")
    with open(path, "w") as handle:
        json.dump(build(template, count, namespace), handle, indent=2)
        handle.write("\n")
    print(path)


if __name__ == "__main__":
    main(sys.argv)
