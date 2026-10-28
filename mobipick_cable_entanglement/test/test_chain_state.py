"""The chain state across processes: a model restored from another model's chain_state() goes on like it, and the MTC
guard's replay started from the monitor's live chain predicts what the monitor computes along the same path, where the
replay from a fresh chain forgets the motion before the path. A monitor that starts its chain afresh at every
trajectory start is modelled by the fresh replay, so its state is not used."""
import copy
import json
import math
import os
import sys
import unittest

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "src"))

from test_cable_model import ANYGRASP, HOME, load  # noqa: E402
from mobipick_cable_entanglement.cable_model import CableModel, config_from_dict  # noqa: E402
from mobipick_cable_entanglement.path_check import carrier_replay, live_state_problem, resample  # noqa: E402

J = ["shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]
PLACE_END = [-90.4, -131.3, 140.3, -148.3, -91.0, 83.5]   # the chain catches on HOME -> PLACE_END (test_cable_chain_guard)
URDF = os.path.join(HERE, "mobipick_sim_robot_description.urdf")
CONFIG = os.path.join(HERE, "..", "config", "cable_model.yaml")


def read(name):
    with open(name) as f:
        return f.read()


def q(deg):
    return {j: math.radians(v) for j, v in zip(J, deg)}


def path(*waypoints, seconds=3.0):
    return [(seconds * i, q(w)) for i, w in enumerate(waypoints)]


def run(model, samples, rate=10.0):
    """the monitor's way through a path: one update per tick (hysteresis on), the stretch of every tick"""
    out, last = [], None
    for t, qt in resample(samples, rate):
        out.append(model.update(qt, 0.0 if last is None else t - last, use_carrier=True).max_stretch)
        last = t
    return out


class TestChainState(unittest.TestCase):
    @staticmethod
    def monitor():
        """the monitor after the motion from home to the place pose: its chain keeps what that motion did"""
        model = load(carrier=True)
        model.reset(q(HOME))
        run(model, path(HOME, PLACE_END))
        return model

    @classmethod
    def setUpClass(cls):
        cls.state = json.loads(json.dumps(cls.monitor().chain_state()))   # as it crosses the service
        cls.state["stamp"] = 100.0

    @staticmethod
    def fresh():
        return load(carrier=True)

    def test_a_restored_model_goes_on_like_the_original(self):
        original = self.monitor()
        restored = self.fresh()
        restored.restore_chain_state(self.state)
        leg = path(PLACE_END, ANYGRASP)
        a, b = run(original, leg), run(restored, leg)
        self.assertEqual(len(a), len(b))
        # the restore recomputes the chain lengths the original reuses from its cache: equal to 0.1 mm of stretch
        for k, (sa, sb) in enumerate(zip(a, b)):
            self.assertAlmostEqual(sa, sb, delta=1e-4, msg=f"tick {k}")

    def test_the_replay_from_the_live_chain_predicts_the_monitor(self):
        leg = path(PLACE_END, ANYGRASP)
        seen = max(run(self.monitor(), leg))
        live = carrier_replay(self.fresh(), leg, start_state=self.state)
        self.assertAlmostEqual(live.max_stretch, seen, delta=1e-4)
        fresh = carrier_replay(self.fresh(), leg)
        # the fresh chain hangs freely at the place pose: it forgets the catch of the motion before
        self.assertLess(fresh.max_stretch, live.max_stretch - 0.005,
                        f"fresh {fresh.max_stretch:+.4f}, live {live.max_stretch:+.4f}")

    def test_no_start_state_is_the_fresh_chain(self):
        leg = path(PLACE_END, ANYGRASP)
        a = carrier_replay(self.fresh(), leg)
        b = carrier_replay(self.fresh(), leg, start_state=None)
        self.assertEqual((a.max_stretch, a.span, a.t, a.updates, a.stop), (b.max_stretch, b.span, b.t, b.updates, b.stop))

    def test_another_config_refuses_the_state(self):
        other = load(carrier=True)
        cfg = copy.deepcopy(other.cfg)
        cfg.cable_carrier.link_chain.friction += 0.1
        other = CableModel(read(URDF), cfg)
        with self.assertRaises(ValueError):
            other.restore_chain_state(self.state)
        self.assertIn("differs", live_state_problem(other, self.state, q(PLACE_END), 100.1))

    def test_live_state_problem(self):
        model = self.fresh()
        start = {f"mobipick/ur5_{j}_joint": v for j, v in q(PLACE_END).items()}   # names as in a trajectory
        self.assertIsNone(live_state_problem(model, self.state, start, 100.5))
        self.assertIn("stale", live_state_problem(model, self.state, start, 102.0))
        moved = dict(start)
        moved["mobipick/ur5_wrist_3_joint"] += 0.05
        self.assertIn("wrist_3", live_state_problem(model, self.state, moved, 100.5))
        self.assertEqual(live_state_problem(model, None, start, 100.5), "no chain state")

    def test_a_monitor_that_resets_at_the_trajectory_start_is_modelled_by_the_fresh_replay(self):
        resetting = dict(self.state, reset_at_trajectory_start=True)
        start = {f"mobipick/ur5_{j}_joint": v for j, v in q(PLACE_END).items()}
        self.assertIn("afresh", live_state_problem(self.fresh(), resetting, start, 100.5))
        self.assertIsNone(live_state_problem(self.fresh(), dict(self.state, reset_at_trajectory_start=False), start,
                                             100.5))

    def test_the_monitor_and_the_guard_build_the_same_model(self):
        """the monitor: config_from_dict(yaml, tf_prefix=its namespace); the MTC guard: config_from_dict(yaml), its
        robot prefix with a slash and the chain switched on (grasplan CableGuard): one fingerprint, or the live chain
        would never be used"""
        urdf = read(URDF)
        data = yaml.safe_load(read(CONFIG))
        monitor = CableModel(urdf, config_from_dict(copy.deepcopy(data), tf_prefix="mobipick"))
        cfg = config_from_dict(copy.deepcopy(data))
        cfg.tf_prefix = "mobipick/"
        guard_cfg = copy.deepcopy(cfg)
        guard_cfg.cable_carrier.enabled = True
        self.assertEqual(monitor.fingerprint(), CableModel(urdf, guard_cfg).fingerprint())


if __name__ == "__main__":
    unittest.main()
