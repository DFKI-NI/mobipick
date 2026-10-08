""": the cable model of a second Mobipick in the sim (links mobipick2/...) equals robot 1's."""
import os
import sys
import unittest

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "src"))

from test_cable_model import ANYGRASP, OBSERVE, q  # noqa: E402
from mobipick_cable_entanglement.cable_model import CableModel, config_from_dict  # noqa: E402


class TestTfPrefix(unittest.TestCase):
  def test_prefix_argument_and_config_default(self):
    d = yaml.safe_load(open(os.path.join(HERE, "..", "config", "cable_model.yaml")))
    self.assertEqual(config_from_dict(d).tf_prefix, "mobipick/")
    self.assertEqual(config_from_dict(d, tf_prefix="mobipick2").tf_prefix, "mobipick2/")
    self.assertEqual(config_from_dict(d, tf_prefix="mobipick2/").tf_prefix, "mobipick2/")

  def test_second_robot_model_matches_the_first(self):
    d = yaml.safe_load(open(os.path.join(HERE, "..", "config", "cable_model.yaml")))
    urdf = open(os.path.join(HERE, "mobipick_sim_robot_description.urdf")).read()
    one = CableModel(urdf, config_from_dict(d))
    two = CableModel(urdf.replace('"mobipick/', '"mobipick2/'), config_from_dict(d, tf_prefix="mobipick2"))
    for pose in (OBSERVE, ANYGRASP):
      self.assertAlmostEqual(one.evaluate(q(pose)).max_stretch, two.evaluate(q(pose)).max_stretch, places=6)


if __name__ == "__main__":
  unittest.main()
