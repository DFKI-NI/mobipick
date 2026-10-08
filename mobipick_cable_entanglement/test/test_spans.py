""" (oscar 2026-09-30): only the wrist span is modelled by default; the shoulder and elbow spans are kept and
switched on by the config's `enabled` or the ROS parameter cable_model/span_<name> (the monitor launch args), which
every node that builds the model follows (the monitor, the MTC cable guard)."""
import os
import sys
import unittest
from unittest import mock

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src"))

from mobipick_cable_entanglement import cable_model  # noqa: E402
from mobipick_cable_entanglement.cable_model import CableModel, config_from_dict  # noqa: E402


def raw():
    with open(os.path.join(HERE, "..", "config", "cable_model.yaml")) as f:
        return yaml.safe_load(f)


class TestSpans(unittest.TestCase):
    def test_only_the_wrist_span_by_default(self):
        self.assertEqual([s.name for s in config_from_dict(raw()).spans], ["wrist"])

    def test_a_ros_parameter_switches_a_span_on_or_off(self):
        params = {"span_elbow": True, "span_shoulder": "true", "span_wrist": "false"}
        with mock.patch.object(cable_model, "_ros_param", side_effect=lambda name: params.get(name)):
            self.assertEqual([s.name for s in config_from_dict(raw()).spans], ["elbow", "shoulder"])

    def test_the_model_runs_with_the_wrist_span_alone(self):
        urdf = open(os.path.join(HERE, "mobipick_sim_robot_description.urdf")).read()
        model = CableModel(urdf, config_from_dict(raw()))
        state = model.update({}, 0.1, hysteresis=False, with_gradient=False)
        self.assertEqual(list(state.spans), ["wrist"])
        self.assertEqual(list(model.disc_points()), ["wrist"])


if __name__ == "__main__":
    unittest.main()
