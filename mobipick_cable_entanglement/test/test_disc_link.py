""": the carrier link chain, the only chain model since 2026-09-30 (the radius rope was removed). A chain of the
carrier's links with a disc on each hangs from the clamps; where it needs more length than the geometric check (L_geo),
or is pulled longer than its links allow, it decides. Without its solver the model refuses to start."""
import math
import os
import sys
import types
import unittest
from unittest import mock

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "src"))

from test_cable_model import HOME, OBSERVE, TRANSPORT, q  # noqa: E402
from mobipick_cable_entanglement import disc_native  # noqa: E402
from mobipick_cable_entanglement.cable_model import (  # noqa: E402
    CableModel, ChainBase, LinkChainConfig, config_from_dict)


def raw():
    from test_cable_model import all_spans
    with open(os.path.join(HERE, "..", "config", "cable_model.yaml")) as f:
        return all_spans(yaml.safe_load(f))


MEASURED_TUBE = 0.48   # [m] oscar's tape measure (the config now has 0.56 m; measures it along the centre line)
FITTING_TUBE = 0.60   # [m] a wrist carrier that fits on every route (the measured 0.48 m does not quite: TestTautChain)


URDF = os.path.join(HERE, "mobipick_sim_robot_description.urdf")


def load(wrist_tube=None, **disc):
    d = raw()
    d["cable_carrier"]["link_chain"].update(disc)
    if wrist_tube:
        d["spans"]["wrist"]["carrier_free_length"] = wrist_tube
    return CableModel(open(URDF).read(), config_from_dict(d))


def loop(*poses, deg_per_tick=4.0, rest=5):
    """joint dicts along straight joint moves through poses (40 deg/s at 10 Hz), resting a few ticks at each"""
    cur = np.array(poses[0], float)
    for target in poses[1:]:
        target = np.array(target, float)
        steps = int(np.max(np.abs(target - cur)) / deg_per_tick) + 1
        for i in range(1, steps + rest):
            yield q(cur + (target - cur) * min(1.0, i / steps))
        cur = target


def fake_rospy(params):
    return types.SimpleNamespace(core=types.SimpleNamespace(is_initialized=lambda: True),
                                 get_param=lambda name, default=None: params.get(name, default))


class TestConfig(unittest.TestCase):
    def test_the_disc_chain_is_the_model(self):
        model = load()
        model.reset(q(HOME))
        state = model.update(q(HOME), 0.1)
        self.assertEqual(set(model.disc_points()), {s.name for s in model.cfg.spans})
        self.assertTrue(all(s.l_disc is not None and s.twist_deg is not None for s in state.spans.values()))
        self.assertTrue(all("disc" in s for s in state.to_dict()["spans"].values()))
        self.assertEqual(set(model.snapshot()), {"disc"})

    def test_disc_sizes(self):
        disc = LinkChainConfig(disc_height=0.005, disc_gap=0.005, disc_diameter=0.035)   # Oscar's first numbers
        self.assertAlmostEqual(disc.pitch, 0.010)
        self.assertAlmostEqual(math.degrees(disc.max_turn), 16.26, places=2)
        self.assertAlmostEqual(disc.bend_radius, 0.0354, places=4)
        # discs that touch at the catalogue bend radius of 50 mm with an 11.3 mm pitch
        gap = 0.0345 * math.tan(math.asin(0.0113 / (2 * 0.05)))
        self.assertAlmostEqual(LinkChainConfig(0.0113 - gap, gap, 0.0345).bend_radius, 0.05, places=6)
        cfg = load(disc_gap=0.003).cfg
        self.assertAlmostEqual(cfg.cable_carrier.link_chain.pitch, 0.008)

    def test_ros_parameters_set_the_disc_sizes(self):
        params = {"cable_model/disc_gap": 0.004, "cable_model/bend_model": "radius"}   # a stale bend_model: ignored
        with mock.patch.dict(sys.modules, {"rospy": fake_rospy(params)}):
            cfg = config_from_dict(raw())
        self.assertAlmostEqual(cfg.cable_carrier.link_chain.disc_gap, 0.004)
        self.assertAlmostEqual(cfg.cable_carrier.link_chain.disc_height, 0.005)

    def test_a_config_asking_for_the_removed_radius_carrier_is_refused(self):
        d = raw()
        d["cable_carrier"]["bend_model"] = "radius"
        with self.assertRaisesRegex(ValueError, "removed"):
            config_from_dict(d)
        # old keys: ignored
        d["cable_carrier"].update(bend_model="disc_link", iterations=12, gravity=9.81, bend_radius=0.09)
        config_from_dict(d)

    def test_without_its_solver_the_model_refuses_to_start(self):
        with mock.patch.object(disc_native, "load", side_effect=RuntimeError("libdisc_chain.so not built")):
            with self.assertRaisesRegex(RuntimeError, "catkin build mobipick_cable_entanglement"):
                load()
            cfg = config_from_dict(raw())
            cfg.cable_carrier.enabled = False           # the geometric check alone needs no solver
            CableModel(open(URDF).read(), cfg).evaluate(q(HOME))


class TestDiscChain(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.disc = load(FITTING_TUBE)
        cls.disc.reset(q(HOME))
        cls.states, cls.worst_length, cls.worst_inside, cls.worst_turn = [], 0.0, 0.0, [0.0, 0.0, 0.0]
        chains = cls.disc._disc
        for pose in loop(HOME, OBSERVE, TRANSPORT, HOME):
            cls.states.append(cls.disc.update(pose, 0.1))
            _, caps, _ = cls.disc._frames(cls.disc._full_q(pose))
            caps = cls.disc._disc_capsules(caps)
            cls.worst_length = max(cls.worst_length, float(np.abs(chains.strain).max()))
            for c in range(len(chains.chains)):
                link = np.diff(chains.chain(c), axis=0)
                link /= np.linalg.norm(link, axis=1)[:, None]
                turn = np.degrees(np.arccos(np.clip((link[:-1] * link[1:]).sum(axis=1), -1.0, 1.0)))
                cls.worst_turn[c] = max(cls.worst_turn[c], float(turn.max()))
            inside = chains.tapered_radius(caps) - ChainBase.closest(chains.x[chains._free], caps)[2]
            cls.worst_inside = max(cls.worst_inside, float(inside.max()))

    def test_the_chain_never_lowers_the_geometric_check(self):
        for state in self.states:
            for span in state.spans.values():
                self.assertGreaterEqual(span.l_req, span.l_geo)

    def test_state_has_the_disc_values(self):
        state = self.states[-1]
        for name, span in state.to_dict()["spans"].items():
            self.assertGreater(span["disc"]["l"], 0.0)
            self.assertLess(abs(span["disc"]["strain"]), 0.01)
            self.assertAlmostEqual(span["disc"]["twist_limit_deg"],
                                   10.0 * (len(self.disc.disc_points()[name]) - 1))
        self.assertEqual(len(self.disc.disc_points()["wrist"]), 61)  # 0.60 m of links of 1 cm

    def test_chain_keeps_its_length_and_stays_out_of_the_capsules(self):
        self.assertTrue(np.isfinite(self.disc._disc.x).all())
        self.assertLess(self.worst_length, 0.01)    # also while the arm moves at 40 deg/s
        self.assertLess(self.worst_inside, 0.002)

    def test_joints_stay_at_their_stop(self):
        stop = math.degrees(self.disc.cfg.cable_carrier.link_chain.max_turn)
        wrist, elbow, shoulder = self.worst_turn
        self.assertLess(elbow, stop + 3.0)
        # one wrist joint goes up to 9 deg over its stop where a capsule pushes the chain right at a clamp
        self.assertLess(wrist, stop + 12.0)
        # the shoulder chain is coiled between clamps 9 cm apart (its pedestal exit is a guess): 10 deg over the stop
        self.assertLess(shoulder, stop + 12.0)

    def test_no_stop_on_everyday_poses(self):
        self.assertTrue(all(b.level != "entangled" for b in self.states))

    def test_twist_is_reported_within_a_few_turns(self):
        # logged only: the value still jumps by tens of degrees between ticks while the chain moves
        for b in self.states:
            self.assertLess(abs(b.spans["wrist"].twist_deg), 3 * 360.0)


class TestTautChain(unittest.TestCase):
    """The links cannot stretch. The measured 0.48 m wrist carrier should fit at observe100cm_right and back at home,
    whichever way the chain got there: the real robot holds both all day. With the shapes fitted to the URDF collision
    meshes it almost does (home -> observe 1.6 %, and back 1.9 %) but not from a reset at observe (16.5 % at
    home: the chain lies around a housing on a side it cannot leave); from 0.52 m on every route stays below 0.4 %
    (with the old capsules: 16 %). Expected failures until someone measures the carrier along its centre line (the length
    comes from the measurement, never from this test). The taut mechanism itself: a carrier that is too short on
    purpose is pulled taut, which counts as that much stretch and a predicted stop."""

    SHORT_TUBE = 0.36   # [m] short on purpose

    def drive(self, start, poses, **kw):
        model = load(**kw)
        model.reset(q(start))
        state = None
        for pose in poses:
            state = model.update(pose, 0.1)
        return state

    def assert_fits(self, state):
        self.assertLess(state.spans["wrist"].disc_strain, 0.01)
        self.assertFalse(state.predicted_stop)

    def test_measured_tube_does_not_stop_at_observe_and_home_from_home(self):
        """the part that holds: no predicted stop at observe and back at home when the chain came from home"""
        for poses in (loop(HOME, OBSERVE, rest=25), loop(HOME, OBSERVE, HOME, rest=25)):
            self.assertFalse(self.drive(HOME, poses, wrist_tube=MEASURED_TUBE).predicted_stop)

    @unittest.expectedFailure   # contradiction: 0.48 m sits at the stop threshold at poses held all day (1.6-2 %)
    def test_measured_tube_fits_from_home_to_observe_and_back(self):
        self.assert_fits(self.drive(HOME, loop(HOME, OBSERVE, rest=25), wrist_tube=MEASURED_TUBE))
        self.assert_fits(self.drive(HOME, loop(HOME, OBSERVE, HOME, rest=25), wrist_tube=MEASURED_TUBE))

    @unittest.expectedFailure   # contradiction: 16.5 % at home after a reset at observe (start-pose dependence)
    def test_measured_tube_fits_from_a_reset_at_observe_to_home(self):
        self.assert_fits(self.drive(OBSERVE, [q(OBSERVE)] * 10, wrist_tube=MEASURED_TUBE))
        self.assert_fits(self.drive(OBSERVE, loop(OBSERVE, HOME, rest=25), wrist_tube=MEASURED_TUBE))

    def test_a_short_tube_is_pulled_taut(self):
        state = self.drive(HOME, loop(HOME, OBSERVE, rest=25), wrist_tube=self.SHORT_TUBE)
        self.assertGreater(state.spans["wrist"].disc_strain, 0.05)
        self.assertEqual(state.spans["wrist"].limiting, "carrier link chain (pulled taut)")
        self.assertAlmostEqual(state.spans["wrist"].stretch, state.spans["wrist"].disc_strain, places=6)
        self.assertTrue(state.predicted_stop)
        self.assertEqual(state.level, "entangled")

    def test_strain_does_not_decide_when_switched_off(self):
        state = self.drive(HOME, loop(HOME, OBSERVE, rest=25), wrist_tube=self.SHORT_TUBE, decide_on_strain=False)
        self.assertGreater(state.spans["wrist"].disc_strain, 0.05)
        self.assertFalse(state.predicted_stop)


ELBOW_UP_114 = [-90, -89, 114, -43, 89, 270]   # upper arm up, elbow 114 deg: held for minutes on 2026-09-27


@unittest.skipUnless(disc_native.available(), "no carrier link chain solver (g++ or libdisc_chain.so)")
class TestReset(unittest.TestCase):
    """a reset starts the carrier link chain hanging below its chord; where that lies on a side of the housings the
    carrier does
    not fit, the chain would stay taut for good (a stop at a pose the real robot holds all day). With the old capsules
    it happened at elbow 114 deg (the chain then started on the radius rope's shape); the reset sweep starts it again
    from the reference pose (LinkChainConfig.reset_sweep_deg)."""

    def strains(self, m):
        return {n: round(float(m._disc.strain[c]), 3) for c, n in enumerate(s.name for s in m.cfg.spans)}

    def test_reset_starts_slack_with_the_upper_arm_up(self):
        m = load()
        m.reset(q(ELBOW_UP_114))
        for _ in range(10):
            state = m.update(q(ELBOW_UP_114), 0.1, hysteresis=False, with_gradient=False)
        self.assertLess(max(self.strains(m).values()), 0.01, self.strains(m))
        self.assertFalse(state.predicted_stop)



SPAWN = [0, -120.3, 139.2, -117.5, -90, 179.9]   # the sim's spawn pose (mobipick_gazebo_common.launch)
PLACE_END = [-90.4, -131.3, 140.3, -148.3, -91.0, 83.5]


@unittest.skipUnless(disc_native.available(), "no carrier link chain solver (g++ or libdisc_chain.so)")
class TestStart(unittest.TestCase):
    """oscar's live test (2026-09-30): the sim spawns the arm at SPAWN and sends it home; the chain must hang in front
    of the wrist_1 cap at home as on his photo, whatever pose it was started at"""

    def lowest_along_wrist_1(self, m):
        Q = m._full_q(q(HOME))
        forearm = m.kin.link_poses(["mobipick/ur5_forearm_link"], Q, "mobipick/ur5_base_link")["mobipick/ur5_forearm_link"]
        x = m.disc_points()["wrist"]
        return (np.linalg.inv(forearm) @ np.c_[x, np.ones(len(x))].T).T[:, 2].min()

    def test_home_after_a_start_elsewhere_is_in_front_of_the_cap(self):
        for start in (SPAWN, OBSERVE, TRANSPORT):
            m = load()
            m.reset(q(start))
            state = None
            for pose in loop(start, start, HOME, rest=25):
                state = m.update(pose, 0.1)
            self.assertTrue(-0.078 <= self.lowest_along_wrist_1(m) <= -0.038, (start, self.lowest_along_wrist_1(m)))
            self.assertLess(state.spans["wrist"].disc_strain, 0.01, start)

    def test_a_start_where_the_way_from_home_is_taut_starts_at_the_pose(self):
        m = load()
        m.reset(q(PLACE_END))   # home -> here along the straight joint path pulls the chain to +18 %
        self.assertLess(max(m._disc.strain), 0.01)


@unittest.skipUnless(disc_native.available(), "no carrier link chain solver (g++ or libdisc_chain.so)")
class TestGeometricSwitch(unittest.TestCase):
    def test_without_the_geometric_check_the_chain_alone_decides(self):
        pose = list(OBSERVE)
        pose[5] = 120.0   # the geometric string over-stretched (test_cable_model: hysteresis test pose)
        both, alone = load(), load()
        alone.cfg.decision.geometric_check = False
        for m in (both, alone):
            m.reset(q(pose))
        a = both.update(q(pose), 0.1, hysteresis=False)
        b = alone.update(q(pose), 0.1, hysteresis=False)
        self.assertTrue(a.predicted_stop)
        self.assertEqual(b.predicted_stop, b.spans["wrist"].disc_strain >= 0.02 or bool(b.spans["wrist"].hooked_on))
        self.assertEqual(alone.evaluate(q(pose)).max_stretch, both.evaluate(q(pose)).max_stretch)   # stays geometric

    def test_ros_parameter(self):
        with mock.patch.dict(sys.modules, {"rospy": fake_rospy({"cable_model/geometric_check": "false"})}):
            self.assertFalse(config_from_dict(raw()).decision.geometric_check)
        self.assertTrue(config_from_dict(raw()).decision.geometric_check)


@unittest.skipUnless(disc_native.available(), "no carrier link chain solver (g++ or libdisc_chain.so)")
class TestFailure(unittest.TestCase):
    """an error in the carrier link chain must not take the decision down (the monitor skips a tick whose update raises:
    a
    failing chain would otherwise stop nothing): that tick the geometric check decides alone, the next update starts
    the chain again, hanging freely"""

    def assert_geometry_alone(self, state, m, pose):
        geometric = m.evaluate(q(pose))
        for n, span in state.spans.items():
            self.assertIsNone(span.l_disc)
            self.assertAlmostEqual(span.stretch, geometric.spans[n].stretch)
        self.assertTrue(m.disc_error)

    def test_failure_in_an_update(self):
        m = load()
        m.reset(q(OBSERVE))
        with mock.patch.object(m._disc, "step", side_effect=FloatingPointError("the carrier link chain is not finite")):
            with self.assertWarns(UserWarning):
                state = m.update(q(OBSERVE), 0.1, hysteresis=False, with_gradient=False)
        self.assert_geometry_alone(state, m, OBSERVE)
        self.assertIsNone(m._disc)
        again = m.update(q(OBSERVE), 0.1, hysteresis=False, with_gradient=False)
        self.assertIsNotNone(m._disc)
        self.assertTrue(all(s.l_disc is not None for s in again.spans.values()))

    def test_failure_in_a_reset(self):
        m = load()
        with mock.patch("mobipick_cable_entanglement.cable_model.CarrierLinkChains.reset",
                        side_effect=RuntimeError("boom")):
            with self.assertWarns(UserWarning):
                m.reset(q(HOME))
            self.assertIsNone(m._disc)
            with self.assertWarns(UserWarning):   # the next update tries again, fails again: geometry alone
                state = m.update(q(HOME), 0.1, hysteresis=False, with_gradient=False)
        self.assert_geometry_alone(state, m, HOME)
        m.update(q(HOME), 0.1, hysteresis=False, with_gradient=False)   # the solver works again: the chain is back
        self.assertEqual(set(m.disc_points()), {s.name for s in m.cfg.spans})


@unittest.skipUnless(disc_native.available(), "no carrier link chain solver (g++ or libdisc_chain.so)")
class TestTickLength(unittest.TestCase):
    def test_chain_rests_whatever_the_tick_length(self):
        """the sim's ticks follow sim time (0.05, 0.1 s, ...): at rest the chain must not move with them"""
        m = load()
        m.reset(q(OBSERVE))
        for dt in [0.1] * 20 + [0.05, 0.1, 0.02, 0.1] * 5:
            m.update(q(OBSERVE), dt, hysteresis=False, with_gradient=False)
        moved = []
        for dt in [0.05, 0.1, 0.03, 0.1] * 5:
            before = m._disc.x.copy()
            m.update(q(OBSERVE), dt, hysteresis=False, with_gradient=False)
            moved.append(float(np.max(np.linalg.norm(m._disc.x - before, axis=1))))
        self.assertLess(max(moved), 0.002, moved)


if __name__ == "__main__":
    unittest.main()
