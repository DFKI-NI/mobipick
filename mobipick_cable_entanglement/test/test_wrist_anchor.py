""": the wrist span may decide from the real forearm carrier support (support_start, z -0.095 on the guide link,
where the carrier starts) with its refit free length, instead of the tip of the longer URDF guide link. 'auto' picks it
in
the sim and keeps the carrier tip on the real robot until tested there; offline (no ROS node) the carrier tip stays. At
the
MTC guard margin -0.115 the support anchor catches the five known taut poses (carrier tip: E2 only) and the everyday
named
poses stay slack."""
import math
import os
import sys
import types
import unittest
from unittest import mock

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src"))

from mobipick_cable_entanglement.cable_model import CableModel, config_from_dict, resolve_wrist_anchor  # noqa: E402

J = ["shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]
# joint poses (deg) at the real cable stops (event files, UTC): 2026-09-27 E1 17:58:26 and E2 18:32:29 (wrist_2), E4
# 20:31:06 (wrist_3, inside a guarded MTC insert); one second before the stop for E3 19:44:17 and 2026-09-28 T1 13:36:50
# (at the stop instant their span is chord-limited, the guard checks the whole path and catches the way in)
STOPS = {"E1": (-54.2, -48.5, 130.1, -144.0, 103.1, 246.9), "E2": (-64.1, -70.7, 137.2, -128.3, 105.0, 241.2),
         "E4": (-75.4, -36.1, 43.9, -96.8, -118.8, 113.8), "E3 -1 s": (-52.8, -50.0, 115.0, -54.4, 41.1, 269.9),
         "T1 -1 s": (-86.5, -82.6, 110.9, -76.9, 30.4, 217.6)}
# everyday named poses (mobipick_os.srdf.xacro)
NAMED = {"home": (0.0, -149.0, 139.2, -117.5, -90.0, 179.9), "transport": (-51.4, -107.7, 122.6, -104.9, -90.5, 218.3),
         "observe100cm_right": (-90.0, -131.8, 139.8, -54.4, 90.0, 269.9),
         "anygrasp": (-90.0, -90.0, 114.6, -43.0, 90.0, 270.0)}
GUARD = -0.115   # grasplan's MTC cable guard margin (DEFAULT_CABLE_MAX_STRETCH)


def raw():
    from test_cable_model import all_spans
    with open(os.path.join(HERE, "..", "config", "cable_model.yaml")) as f:
        return all_spans(yaml.safe_load(f))


def q(deg):
    return {j: math.radians(v) for j, v in zip(J, deg)}


def span(cfg, name):
    return next(s for s in cfg.spans if s.name == name)


class TestAnchorChoice(unittest.TestCase):
    def test_offline_default_is_the_tube_tip(self):
        cfg = config_from_dict(raw())          # the config says auto; no ROS node here
        self.assertEqual(cfg.wrist_anchor, "carrier_tip")
        self.assertAlmostEqual(span(cfg, "wrist").start.xyz[2], -0.15)
        self.assertAlmostEqual(span(cfg, "wrist").free_length, 0.43)

    def test_support_moves_only_the_wrist_start_and_length(self):
        old, new = config_from_dict(raw(), wrist_anchor="carrier_tip"), config_from_dict(raw(), wrist_anchor="support")
        self.assertAlmostEqual(span(new, "wrist").start.xyz[2], -0.095)
        self.assertAlmostEqual(span(new, "wrist").free_length, 0.45)
        for name in ("elbow", "shoulder"):
            self.assertEqual(span(new, name).free_length, span(old, name).free_length)
            self.assertTrue((span(new, name).start.xyz == span(old, name).start.xyz).all())
        # the simulated carrier keeps its measured start and length
        self.assertTrue((span(new, "wrist").carrier_anchor.xyz == span(old, "wrist").carrier_anchor.xyz).all())
        self.assertEqual(span(new, "wrist").carrier_length, span(old, "wrist").carrier_length)

    def test_auto_in_a_node(self):
        params = {}
        # a fake rospy (the host has none): an initialized node with these parameters
        fake = types.SimpleNamespace(core=types.SimpleNamespace(is_initialized=lambda: True),
                                     get_param=lambda name, default=None: params.get(name, default))
        with mock.patch.dict(sys.modules, {"rospy": fake}):
            for clock in (True, False):
                params["/use_sim_time"] = clock
                self.assertEqual(resolve_wrist_anchor("auto"), "carrier_tip")  # the clock plays no part
            params.update({"/use_sim_time": True, "/cable_model/wrist_anchor": "support"})
            self.assertEqual(resolve_wrist_anchor("auto"), "support")          # the global parameter
            params.update({"/use_sim_time": False, "/cable_model/wrist_anchor": "carrier_tip"})
            self.assertEqual(resolve_wrist_anchor("auto"), "carrier_tip")      # the global override
            self.assertEqual(resolve_wrist_anchor("support"), "support")    # an explicit choice is not overridden

    def test_unknown_choice(self):
        with self.assertRaises(ValueError):
            config_from_dict(dict(raw(), wrist_anchor="tip"))


class TestRealStops(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        urdf = open(os.path.join(HERE, "mobipick_sim_robot_description.urdf")).read()
        cls.tip = CableModel(urdf, config_from_dict(raw(), wrist_anchor="carrier_tip"))
        cls.support = CableModel(urdf, config_from_dict(raw(), wrist_anchor="support"))

    def wrist(self, model, pose):
        return model.evaluate(q(pose)).spans["wrist"].stretch

    def test_the_guard_catches_all_five_taut_poses_instead_of_one(self):
        self.assertEqual([k for k, v in STOPS.items() if self.wrist(self.tip, v) >= GUARD], ["E2"])
        caught = {k: self.wrist(self.support, v) for k, v in STOPS.items()}
        self.assertTrue(all(v >= GUARD + 0.02 for v in caught.values()), caught)   # with some margin (E4: 3 points)

    def test_named_poses_stay_slack(self):
        for name, pose in NAMED.items():
            self.assertLess(self.wrist(self.support, pose), -0.25, name)
            self.assertLessEqual(self.wrist(self.support, pose), self.wrist(self.tip, pose) + 0.1, name)


if __name__ == "__main__":
    unittest.main()
