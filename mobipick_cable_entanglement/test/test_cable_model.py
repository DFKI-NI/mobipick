"""Unit tests of the cable model (pure numpy, no ROS master): python3 -m pytest test/ or catkin run_tests."""
import math
import os
import sys
import unittest

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src"))
from mobipick_cable_entanglement.cable_model import CableModel, ChainBase, config_from_dict  # noqa: E402
from mobipick_cable_entanglement.geometry import cylinder_geodesic, string_pull  # noqa: E402

J = ["shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]
HOME = [0, -149, 139.2, -117.5, -90, 179.9]
OBSERVE = [0, -131.8, 139.8, -54.4, 90, 269.9]
TRANSPORT = [-51.4, -107.7, 122.6, -104.9, -90.5, 218.3]
UNTANGLE_1 = [-90, -131.8, 139.8, -148.4, -90, 222.3]
UNTANGLE_2 = [-90, -131.8, 139.8, -148.4, -90.5, 177.6]
ANYGRASP = [-90, -90, 114.6, -43, 90, 270]


def q(deg):
    return {j: math.radians(v) for j, v in zip(J, deg)}


def all_spans(d):
    """the config dict with every span switched on (shoulder and elbow are off by default; the tests keep
    testing their mechanisms)"""
    for s in d["spans"].values():
        s["enabled"] = True
    return d


def load(carrier=False):
    cfg = config_from_dict(all_spans(yaml.safe_load(open(os.path.join(HERE, "..", "config", "cable_model.yaml")))))
    cfg.cable_carrier.enabled = carrier
    urdf = open(os.path.join(HERE, "mobipick_sim_robot_description.urdf")).read()
    return CableModel(urdf, cfg)


class TestGeometry(unittest.TestCase):
    def test_geodesic_without_contact_is_the_chord(self):
        L = cylinder_geodesic(0.2, 0.2, 0.0, math.radians(60), 0.05)
        self.assertAlmostEqual(L, 0.2, places=6)  # equilateral triangle

    def test_geodesic_half_turn_wraps(self):
        r, rho = 0.05, 0.1
        L = cylinder_geodesic(rho, rho, 0.0, math.pi, r)
        alpha = math.acos(r / rho)
        expected = 2 * math.sqrt(rho ** 2 - r ** 2) + r * (math.pi - 2 * alpha)
        self.assertAlmostEqual(L, expected, places=6)
        self.assertGreater(L, 2 * rho - 1e-9)

    def test_geodesic_grows_with_extra_turns(self):
        a = cylinder_geodesic(0.1, 0.1, 0.02, math.pi, 0.05)
        b = cylinder_geodesic(0.1, 0.1, 0.02, 3 * math.pi, 0.05)
        self.assertAlmostEqual(b - a, 0.05 * 2 * math.pi, delta=0.005)  # one more turn costs about 2*pi*r

    def test_string_pull_drops_free_points_and_keeps_holds(self):
        caps = (np.array([[0.0, 0.0, -1.0]]), np.array([[0.0, 0.0, 1.0]]), np.array([0.05]))
        pts = [np.array([-0.2, 0.0, 0.0]), np.array([0.0, -0.3, 0.0]), np.array([0.2, 0.0, 0.0])]
        self.assertEqual(len(string_pull(pts, *caps, tol=0.0)), 3)  # the straight line would cut the capsule
        pts = [np.array([-0.2, 0.3, 0.0]), np.array([0.0, 0.5, 0.0]), np.array([0.2, 0.3, 0.0])]
        self.assertEqual(len(string_pull(pts, *caps, tol=0.0)), 2)


class TestStatelessModel(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = load(carrier=False)

    def test_everyday_poses_are_ok(self):
        for name, pose in [("home", HOME), ("observe", OBSERVE), ("transport", TRANSPORT),
                           ("untangle_1", UNTANGLE_1), ("untangle_2", UNTANGLE_2), ("anygrasp", ANYGRASP)]:
            st = self.model.evaluate(q(pose))
            self.assertEqual(st.level, "ok", f"{name}: {st.reason}")

    def test_untangle_poses_give_the_wrist_loop_slack(self):
        home = self.model.evaluate(q(HOME)).spans["wrist"].l_req
        self.assertLess(self.model.evaluate(q(UNTANGLE_1)).spans["wrist"].l_req, home)

    def test_large_wrist_3_turn_from_observe_is_entangled(self):
        pose = list(OBSERVE)
        pose[5] = 120.0  # about 150 deg below observe, like the grasp-orientation inserts
        st = self.model.evaluate(q(pose))
        self.assertTrue(st.predicted_stop, st.reason)
        self.assertEqual(st.worst_span, "wrist")
        self.assertGreater(abs(st.torques["wrist_3"]), 0.0)

    def test_joint_limit_edge_of_the_home_family_is_close_to_taut(self):
        pose = list(HOME)
        pose[5] = 30.0  # wrist_3 lower limit ("reduced to avoid overstretching the cable guide")
        st = self.model.evaluate(q(pose))
        self.assertGreater(st.max_stretch, -0.1)
        self.assertLess(st.max_stretch, self.model.cfg.decision.stop_stretch)

    def test_winding_is_multi_turn(self):
        a = q(HOME)
        b = dict(a)
        b["wrist_3"] += 2 * math.pi
        wa = self.model.evaluate(a).spans["wrist"].winding_deg["wrist_3"]
        wb = self.model.evaluate(b).spans["wrist"].winding_deg["wrist_3"]
        self.assertAlmostEqual(wb - wa, 360.0, delta=1e-6)

    def test_untangle_plan_moves_only_wrist_joints_until_slack(self):
        pose = list(OBSERVE)
        pose[5] = 120.0
        steps, st, ok = self.model.untangle_plan(q(pose))
        self.assertTrue(ok, st.reason)
        self.assertLessEqual(st.max_stretch, self.model.cfg.decision.warn_stretch)
        self.assertTrue(steps)
        self.assertTrue(all(j.startswith("wrist") for j, _ in steps))
        self.assertEqual(steps[0][0], "wrist_3")
        self.assertEqual(self.model.untangle_plan(q(HOME))[0], [])  # already slack: nothing to do

    def test_accepts_prefixed_joint_names(self):
        prefixed = {f"mobipick/ur5_{k}_joint": v for k, v in q(HOME).items()}
        self.assertAlmostEqual(self.model.evaluate(prefixed).max_stretch, self.model.evaluate(q(HOME)).max_stretch)


class TestUntangledChain(unittest.TestCase):
    """Oscar's photo at home (2026-09-28): the untangled wrist carrier leaves the forearm bracket and drops in front of
    the wrist_1 cap on its way to the tool; the model's carrier used to lie over the top of the wrist. Since 2026-09-30
    the cable carrier is modelled by its link chain alone (the radius rope was removed)."""

    @classmethod
    def setUpClass(cls):
        cls.model = load(carrier=True)
        cls.model.reset(q(HOME))

    def test_wrist_chain_passes_in_front_of_the_wrist_1_cap(self):
        # 2026-09-30, radius rope removed: dropped at full speed the carrier link chain swung over the top (+16.5 mm);
        # settled
        # slowly (LinkChainConfig.settle_keep) it rests in front of the cap (-41 mm), from any reset pose
        forearm = self.model.kin.link_poses(["mobipick/ur5_forearm_link"], self.model._full_q(q(HOME)),
                                            "mobipick/ur5_base_link")["mobipick/ur5_forearm_link"]
        chain = self.model.disc_points()["wrist"]
        local = (np.linalg.inv(forearm) @ np.c_[chain, np.ones(len(chain))].T).T[:, :3]
        # forearm link: the wrist_1 axis is its z at x -0.392, the cap face at z -0.041 (collision mesh). : the
        # acceptance band of oscar's photo (2 cm): the loop's lowest point 38-78 mm out along the wrist_1 axis (the
        # shapes fitted to the meshes give -39 mm, the old capsules -84 mm, floating)
        lowest = local[:, 2].min()
        print(f"wrist chain at home: lowest point {1000 * lowest:.1f} mm along the wrist_1 axis")
        self.assertTrue(-0.078 <= lowest <= -0.038, lowest)

    def test_no_chain_link_passes_through_a_housing(self):
        _, caps, _ = self.model._frames(self.model._full_q(q(HOME)))
        for span, x in self.model.disc_points().items():
            self.assertFalse(ChainBase.passes_through(x, self.model._disc_capsules(caps)), span)


class TestStatefulModel(unittest.TestCase):
    def test_chain_follows_a_motion_without_false_alarm(self):
        model = load(carrier=True)
        model.reset(q(HOME))
        cur = np.array(HOME, float)
        for target in (OBSERVE, TRANSPORT, HOME):
            target = np.array(target, float)
            steps = int(np.max(np.abs(target - cur)) / 4.0) + 1  # 40 deg/s at 10 Hz
            for i in range(1, steps + 10):
                st = model.update(q(cur + (target - cur) * min(1.0, i / steps)), 0.1)
                self.assertNotEqual(st.level, "entangled", st.reason)
            cur = target
        self.assertEqual(len(model.disc_points()), 3)

    def test_hysteresis_needs_consecutive_predictions(self):
        model = load(carrier=False)
        pose = list(OBSERVE)
        pose[5] = 120.0
        first = model.update(q(pose), 0.1, use_carrier=False)
        self.assertTrue(first.predicted_stop)
        self.assertNotEqual(first.level, "entangled")
        second = model.update(q(pose), 0.1, use_carrier=False)
        self.assertEqual(second.level, "entangled")


if __name__ == "__main__":
    unittest.main()
