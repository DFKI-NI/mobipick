#!/usr/bin/env python3
"""Writes config/sim_gui_x<N>.perspective: the sim GUI of config/sim_gui.perspective for N robots in ONE rqt
window: per robot a steering, a joint trajectory controller and a simulated e-stop panel, each instance saved with the
robot it belongs to (steering topic, controller manager, e-stop namespace). Robot 1 is `mobipick`, robot k is
`mobipick<k>`. Usage: make_sim_gui_perspective.py N [NAMESPACE]  (default namespace mobipick); stdlib only."""
import ast
import copy
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG = os.path.join(HERE, "..", "config")

PLUGINS = [  # (running-plugins key, group name prefix, dock widget group, title, instance setting)
    ("rqt_robot_steering/RobotSteering", "plugin__rqt_robot_steering__RobotSteering__", "dock_widget__RobotSteeringUi",
     "Robot Steering", lambda ns: ("topic", f"/{ns}/cmd_vel")),
    ("rqt_joint_trajectory_controller/JointTrajectoryController",
     "plugin__rqt_joint_trajectory_controller__JointTrajectoryController__", "dock_widget__JointTrajectoryControllerUi",
     "Joint trajectory controller", lambda ns: ("cm_ns", f"/{ns}/controller_manager")),
    ("mobipick_gazebo/SimulatedUrDashboard", "plugin__mobipick_gazebo__SimulatedUrDashboard__",
     "dock_widget__SimulatedUrDashboardUi", "Simulated E-Stop", lambda ns: ("robot_namespace", ns)),
]


def robots(count, namespace="mobipick"):
    return [namespace] + [f"{namespace}{k}" for k in range(2, count + 1)]


def build(template, count, namespace="mobipick"):
    names = robots(count, namespace)
    out = copy.deepcopy(template)
    manager = out["groups"]["pluginmanager"]
    manager["keys"]["running-plugins"]["repr"] = repr({key: list(range(1, count + 1)) for key, *_ in PLUGINS})
    groups = {}
    for key, prefix, dock, title, setting in PLUGINS:
        for index, ns in enumerate(names, start=1):
            group = copy.deepcopy(template["groups"]["pluginmanager"]["groups"][prefix + "1"])
            group["groups"][dock]["keys"]["dock_widget_title"]["repr"] = repr(f"{title} ({ns})")
            name, value = setting(ns)
            group["groups"]["plugin"]["keys"] = {name: {"type": "repr", "repr": repr(value)}}
            groups[f"{prefix}{index}"] = group
    manager["groups"] = groups
    out["groups"]["mainwindow"]["keys"].pop("state", None)   # the dock layout of one robot does not fit N
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
